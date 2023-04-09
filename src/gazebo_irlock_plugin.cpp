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



  this->camera = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

  if (!this->camera) {
    gzerr << "IRLockPlugin requires a CameraSensor.\n";
  }

  // this->world = physics::get_world(_sensor->WorldName());
  this->world = physics::get_world(this->camera->WorldName()); 

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzwarn << "[gazebo_irlock_plugin] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("beaconName")) {
    beacon_name_ = _sdf->GetElement("beaconName")->Get<std::string>();
  } else {
    gzwarn << "[gazebo_irlock_plugin] Please specify a beacon name.\n";
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

  // Get the root model name
  vector<std::string> names_splitted;
  boost::split(names_splitted, scopedName, boost::is_any_of("::"));
  names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted),
                            [](const string& name)
                            { return name.size() == 0; }), end(names_splitted));

  // get the root model name
  const string rootModelName = names_splitted.front();

  // store the model name
  model_name_ = names_splitted.at(0);

  std::cout << "[Gazebo Irlock Plugin] Irlock camera attached to: "<< model_name_ << std::endl; 
}

void IRLockPlugin::OnUpdated()
{

  // Store the pointer to the model.
  if (model_ == NULL)
#if GAZEBO_MAJOR_VERSION >= 9
    model_ = this->world->ModelByName(model_name_);
#else
    model_ = this->world->GetModel(model_name_);
#endif

  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world->SimTime();
  ignition::math::Pose3d T_W_I = model_->WorldPose();
#else
  common::Time now = world->GetSimTime();
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(model_->GetWorldPose());
#endif

  /* Get the attitude of the drone to convert observation from body to vehicle-carried NED frame */
  ignition::math::Quaterniond& att_W_I = T_W_I.Rot();

  gazebo::msgs::LogicalCameraImage img = this->camera->Image();

  for (int idx = 0; idx < img.model_size(); idx++) {

    gazebo::msgs::LogicalCameraImage_Model model = img.model(idx);

    if (model.has_name() && model.name() == beacon_name_) {

      if (model.has_pose()) {

        /* Save the attitude of the drone when the frame was grabbed, will allow to transform from FRD to vc-NED */
        ignition::math::Quaterniond q_FLU_to_NED = q_ENU_to_NED * att_W_I;
        ignition::math::Quaterniond q_nb = q_FLU_to_NED * q_FLU_to_FRD.Inverse();

        irlock_message.set_attitude_q_w(q_nb.W());
        irlock_message.set_attitude_q_x(q_nb.X());
        irlock_message.set_attitude_q_y(q_nb.Y());
        irlock_message.set_attitude_q_z(q_nb.Z());

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
