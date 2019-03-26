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
 * Desc: Distance Sensor plugin
 * Author: Nate Koenig mod by John Hsu and Elia Tarasov
 */

#include <gazebo/physics/physics.hh>
#include <gazebo_lidar_plugin.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdio.h>
#include <boost/algorithm/string.hpp>

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(RayPlugin)

/////////////////////////////////////////////////
RayPlugin::RayPlugin(){}

/////////////////////////////////////////////////
RayPlugin::~RayPlugin() {
  this->new_scans_connection_->~Connection();
  this->new_scans_connection_.reset();
  this->parent_sensor_.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void RayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
  // Get then name of the parent sensor
  this->parent_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_sensor_)
    gzthrow("RayPlugin requires a Ray Sensor as its parent");

  this->world = physics::get_world(this->parent_sensor_->WorldName());

  this->new_scans_connection_ = this->parent_sensor_->LaserShape()->ConnectNewLaserScans(
      boost::bind(&RayPlugin::OnNewLaserScans, this));

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_lidar_plugin] Please specify a robotNamespace.\n";

  // get minimum distance
  if (_sdf->HasElement("min_distance")) {
    min_distance_ = _sdf->GetElement("min_distance")->Get<double>();
    if (min_distance_ < SENSOR_MIN_DISTANCE) {
      min_distance_ = SENSOR_MIN_DISTANCE;
    }
  } else {
    gzwarn << "[gazebo_lidar_plugin] Using default minimum distance: 0.3\n";
    min_distance_ = DEFAULT_MIN_DISTANCE;
  }

  // get maximum distance
  if (_sdf->HasElement("max_distance")) {
    max_distance_ = _sdf->GetElement("max_distance")->Get<double>();
    if (max_distance_ > SENSOR_MAX_DISTANCE) {
      max_distance_ = SENSOR_MAX_DISTANCE;
    }
  } else {
    gzwarn << "[gazebo_lidar_plugin] Using default maximum distance: 15\n";
    max_distance_ = DEFAULT_MAX_DISTANCE;
  }

  // get type of the sensor according to https://github.com/PX4/Firmware/blob/master/msg/distance_sensor.msg
  if (_sdf->HasElement("type")) {
    type_ = _sdf->GetElement("type")->Get<uint8_t>();
  } else {
    gzwarn << "[gazebo_lidar_plugin] Using default type: 0\n";
    type_ = DEFAULT_SENSOR_TYPE;
  }

  // get facing of the sensor according to https://github.com/PX4/Firmware/blob/master/msg/distance_sensor.msg
  if (_sdf->HasElement("facing")) {
    facing_ = _sdf->GetElement("facing")->Get<uint8_t>();
  } else {
    gzwarn << "[gazebo_lidar_plugin] Using default facing: 25\n";
    facing_ = DEFAULT_SENSOR_FACING;
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  const string scopedName = _parent->ParentName();
  vector<string> names_splitted;
  boost::split(names_splitted,scopedName,boost::is_any_of("::"));
  string topicName = "~/" + names_splitted[0] + "/link/distance_sensor";

  distance_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Range>(topicName, 10);
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

  distance_message_.set_time_usec(now.Double() * 1e6);
  distance_message_.set_min_distance(min_distance_);
  distance_message_.set_max_distance(max_distance_);

  double current_distance = parent_sensor_->Range(0);

  // set distance to min/max if actual value is smaller/bigger
  if (current_distance < min_distance_ || std::isinf(current_distance)) {
    current_distance = min_distance_;
  } else if (current_distance > max_distance_) {
    current_distance = max_distance_;
  }

  distance_message_.set_current_distance(current_distance);
  distance_message_.set_type(type_);
  distance_message_.set_facing(facing_);

  distance_pub_->Publish(distance_message_);
}
