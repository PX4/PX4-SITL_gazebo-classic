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

#include <highgui.h>
#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>

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

  this->camera = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

  if (!this->camera) {
    gzerr << "IRLockPlugin requires a CameraSensor.\n";
  }

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzwarn << "Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  const string scopedName = _sensor->ParentName();

  string topicName = "~/" + scopedName + "/irlock";
  boost::replace_all(topicName, "::", "/");

  irlock_pub_ = node_handle_->Advertise<irlock_msgs::msgs::irlock>(topicName, 10);

  this->updateConnection = this->camera->ConnectUpdated(
      std::bind(&IRLockPlugin::OnUpdated, this));

  this->camera->SetActive(true);

}

void IRLockPlugin::OnUpdated()
{
  gazebo::msgs::LogicalCameraImage img = this->camera->Image();

  for (int idx = 0; idx < img.model_size(); idx++) {

    gazebo::msgs::LogicalCameraImage_Model model = img.model(idx);

    if (model.has_name() && model.name() == "irlock_beacon") {

      if (model.has_pose()) {

        // position of the beacon in camera frame
        gazebo::math::Vector3 pos;
        pos.x = model.pose().position().x();
        pos.y = model.pose().position().y();
        pos.z = model.pose().position().z();

        // the default orientation of the IRLock sensor reports beacon in front of vehicle as -y values, beacon right of vehicle as x values
        // rotate the measurement accordingly
        gazebo::math::Vector3 meas(-pos.y/pos.x, -pos.z/pos.x, 1.0);

        // prepare irlock message
        irlock_message.set_time_usec(0); // will be filled in simulator_mavlink.cpp
        irlock_message.set_signature(idx); // unused by beacon estimator
        irlock_message.set_pos_x(meas.x);
        irlock_message.set_pos_y(meas.y);
        irlock_message.set_size_x(0); // unused by beacon estimator
        irlock_message.set_size_y(0); // unused by beacon estimator

        // send message
        irlock_pub_->Publish(irlock_message);

      }
    }
  }

}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */