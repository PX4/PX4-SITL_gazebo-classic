/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo_irlock_plugin.h"

#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <ignition/math.hh>

using namespace cv;
using namespace std;

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(IRLockPlugin)

IRLockPlugin::IRLockPlugin() : SensorPlugin()
{

}

IRLockPlugin::~IRLockPlugin()
{
  this->camera.reset();
}

void IRLockPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->world = physics::get_world(_sensor->WorldName());

  this->camera = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

  if (!this->camera) {
    gzerr << "IRLockPlugin requires a CameraSensor.\n";
  }

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzwarn << "[gazebo_irlock_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  const string scopedName = _sensor->ParentName();

  string topicName = "~/" + scopedName + "/irlock";
  boost::replace_all(topicName, "::", "/");

  irlock_pub_ = node_handle_->Advertise<sensor_msgs::msgs::IRLock>(topicName, 10);

  this->updateConnection = this->camera->ConnectUpdated(
      std::bind(&IRLockPlugin::OnUpdated, this));

  this->camera->SetActive(true);

}

void IRLockPlugin::OnUpdated()
{
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world->SimTime();
#else
  common::Time now = world->GetSimTime();
#endif

  gazebo::msgs::LogicalCameraImage img = this->camera->Image();

  for (int idx = 0; idx < img.model_size(); idx++) {

    gazebo::msgs::LogicalCameraImage_Model model = img.model(idx);

    if (model.has_name() && model.name() == "irlock_beacon") {

      if (model.has_pose()) {

        // position of the beacon in camera frame
        ignition::math::Vector3d pos;
        pos.X() = model.pose().position().x();
        pos.Y() = model.pose().position().y();
        pos.Z() = model.pose().position().z();

        // the default orientation of the IRLock sensor reports beacon in front of vehicle as -y values, beacon right of vehicle as x values
        // rotate the measurement accordingly
        ignition::math::Vector3d meas(-pos.Y()/pos.X(), -pos.Z()/pos.X(), 1.0);

        // prepare irlock message
        irlock_message.set_time_usec(now.Double() * 1e6);
        irlock_message.set_signature(idx); // unused by beacon estimator
        irlock_message.set_pos_x(meas.X());
        irlock_message.set_pos_y(meas.Y());
        irlock_message.set_size_x(0); // unused by beacon estimator
        irlock_message.set_size_y(0); // unused by beacon estimator

        // Use ignition to send orientation quaternion
        ignition::math::Pose3d modelRelativePose = gazebo::msgs::ConvertIgn(model.pose());
        ignition::math::Quaterniond q_enu_to_ned(3.141f, 0.0f, 0.0f);
        ignition::math::Quaterniond modelRelativeRotNed = q_enu_to_ned * modelRelativePose.Rot();

        irlock_message.set_q_w(modelRelativeRotNed.W());
        irlock_message.set_q_x(modelRelativeRotNed.X());
        irlock_message.set_q_y(modelRelativeRotNed.Y());
        irlock_message.set_q_z(modelRelativeRotNed.Z());

        // send message
        irlock_pub_->Publish(irlock_message);

      }
    }
  }

}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
