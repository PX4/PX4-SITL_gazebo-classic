/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Contact plugin
 * Author: Nate Koenig mod by John Hsu
 */

#include "gazebo/physics/physics.hh"
#include "gazebo_lidar_plugin.h"

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include <common.h>

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(RayPlugin)

/////////////////////////////////////////////////
RayPlugin::RayPlugin()
{
}

/////////////////////////////////////////////////
RayPlugin::~RayPlugin()
{
  this->newLaserScansConnection->~Connection();

  this->newLaserScansConnection.reset();

  this->parentSensor_.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void RayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  this->parentSensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parentSensor_)
    gzthrow("RayPlugin requires a Ray Sensor as its parent");

  this->world = physics::get_world(this->parentSensor_->WorldName());

  this->newLaserScansConnection = this->parentSensor_->LaserShape()->ConnectNewLaserScans(
      boost::bind(&RayPlugin::OnNewLaserScans, this));

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_lidar_plugin] Please specify a robotNamespace.\n";

  // get minimum distance
  if (_sdf->HasElement("min_distance")) {
    min_distance_ = _sdf->GetElement("min_distance")->Get<double>();
    if (min_distance_ < kSensorMinDistance) {
      min_distance_ = kSensorMinDistance;
    }
  } else {
    gzwarn << "[gazebo_lidar_plugin] Using default minimum distance: " << kDefaultMinDistance << "\n";
    min_distance_ = kDefaultMinDistance;
  }

  // get maximum distance
  if (_sdf->HasElement("max_distance")) {
    max_distance_ = _sdf->GetElement("max_distance")->Get<double>();
    if (max_distance_ > kSensorMaxDistance) {
      max_distance_ = kSensorMaxDistance;
    }
  } else {
    gzwarn << "[gazebo_lidar_plugin] Using default maximum distance: " << kDefaultMaxDistance << "\n";
    max_distance_ = kDefaultMaxDistance;
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  const string scopedName = _parent->ParentName();
  vector<string> names_splitted;
  boost::split(names_splitted,scopedName,boost::is_any_of("::"));
  string topicName = "~/" + names_splitted[0] + "/link/lidar";

  lidar_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Range>(topicName, 10);

  // Precaution: remove empty strings
  names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted),
                              [](const string& name)
                              { return name.size() == 0; }), end(names_splitted));

  std::string rootModelName = names_splitted.front(); // The first element is the name of the root model

  // Get the pointer to the root model
#if GAZEBO_MAJOR_VERSION >= 9
  physics::ModelPtr rootModel = this->world->ModelByName(rootModelName);
#else
  physics::ModelPtr rootModel = this->world->GetModel(rootModelName);
#endif

  // Get the `base_link` rotation WRT world
  physics::LinkPtr baseLink = nullptr;
  std::vector<physics::LinkPtr> linkList = rootModel->GetLinks(); // Get list of all links in the root model
  for(auto link : linkList) {
    std::string linkName = link->GetName();
    if(linkName.find("::base_link") != std::string::npos) {
      baseLink = rootModel->GetLink(linkName); // Get the pointer to the `base_link`
      break;
    }
  }
  if (!baseLink)
    gzthrow("RayPlugin requires the `base_link` element to be defined");

  // This is the rotation of the 'base_link` WRT world
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Quaterniond q_wb = baseLink->WorldPose().Rot();
#else
  ignition::math::Quaterniond q_wb = ignitionFromGazeboMath(baseLink->GetWorldPose()).Rot();
#endif

  // Get the parent sensor link rotation WRT world
  physics::LinkPtr parentSensorLink = nullptr;
  std::string parentSensorModelName = names_splitted.rbegin()[1]; // the second to the last name is the model name
  for(auto link : linkList) {
    std::string linkName = link->GetName();
    if(linkName.find(parentSensorModelName) != std::string::npos) {
      parentSensorLink = rootModel->GetLink(linkName); // Get the pointer to the parent sensor link
      break;
    }
  }
  if (!parentSensorLink)
    gzthrow("RayPlugin requires a `link` element for its parent sensor to be defined");

  // This is the rotation of the parentSensorLink WRT world
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Quaterniond q_wl = parentSensorLink->WorldPose().Rot();
#else
  ignition::math::Quaterniond q_wl = ignitionFromGazeboMath(parentSensorLink->GetWorldPose()).Rot();
#endif

  // Calculate parent sensor rotation WRT `base_link`
  ignition::math::Quaterniond q_bl = q_wb.Inverse() * q_wl; // This is the rotation of the parent sensor link WRT `base_link`

  ignition::math::Quaterniond q_ls = parentSensor_->Pose().Rot(); // This is the rotation of the parent sensor WRT parent sensor link
  ignition::math::Quaterniond q_bs = (q_bl * q_ls).Inverse(); // This is the rotation of the parent sensor WRT `base_link`

  // set the orientation
  orientation_.set_x(q_bs.X());
  orientation_.set_y(q_bs.Y());
  orientation_.set_z(q_bs.Z());
  orientation_.set_w(q_bs.W());
}

/////////////////////////////////////////////////
void RayPlugin::OnNewLaserScans()
{
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world->SimTime();
#else
  common::Time now = world->GetSimTime();
#endif

  lidar_message.set_time_usec(now.Double() * 1e6);
  lidar_message.set_min_distance(min_distance_);
  lidar_message.set_max_distance(max_distance_);

  double current_distance = parentSensor_->Range(0);

  // set distance to min/max if actual value is smaller/bigger
  if (current_distance < min_distance_ || std::isinf(current_distance)) {
    current_distance = min_distance_;
  } else if (current_distance > max_distance_) {
    current_distance = max_distance_;
  }

  lidar_message.set_current_distance(current_distance);
  lidar_message.set_h_fov(0.0523598776);    // 3 degrees standard
  lidar_message.set_v_fov(0.0523598776);    // 3 degrees standard
  lidar_message.set_allocated_orientation(new gazebo::msgs::Quaternion(orientation_));

  lidar_pub_->Publish(lidar_message);
}
