/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
/* Desc: A basic gimbal controller
 * Author: John Hsu
 */

#ifndef _GAZEBO_ACS_CONTROLLER_PLUGIN_HH_
#define _GAZEBO_ACS_CONTROLLER_PLUGIN_HH_

#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <ignition/math.hh>

#include "Groundtruth.pb.h"
#include "PID.h"
#include <iostream>
#include <time.h>

// custom compiled messages ----------------------------------------------------
#include "NewXYStatus.pb.h"
#include "RollPitchStatus.pb.h"
#include "RollPitchSetpoint.pb.h"
#include "ThrusterStatus.pb.h"
// -----------------------------------------------------------------------------

namespace gazebo {

class GAZEBO_VISIBLE ACSControllerPlugin : public ModelPlugin {

public:
  ACSControllerPlugin();

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();

private:
  void OnUpdate();

  physics::ModelPtr _model;
  sdf::ElementPtr _sdf;

  std::vector<event::ConnectionPtr> connections;

  sensors::ImuSensorPtr imuSensor;

  transport::NodePtr node_handle_;

  // update time used for PID controller
  std::string namespace_;
  common::Time lastUpdateTime;

  // status variables to keep track of outputs
  std::vector<double> _thrusterStatus = {0, 0, 0, 0};
  double _newX;
  double _newY;
  double _rollTarget;
  double _pitchTarget;
  double _rollSetpoint;
  double _pitchSetpoint;

  // status topics
  std::string new_xy_pub_topic_;
  std::string roll_pitch_pub_topic_;
  std::string roll_pitch_setpoint_pub_topic_;
  std::string thruster_pub_topic_;

  // status publishers
  transport::PublisherPtr new_xy_status_pub_;
  transport::PublisherPtr roll_pitch_status_pub_;
  transport::PublisherPtr roll_pitch_setpoint_pub_;
  transport::PublisherPtr thruster_status_pub_;
};

class actuator {
public:
  gazebo::physics::LinkPtr link;
  const ignition::math::Vector3<double> &force = {0, 0, 0};
  std::string path;
  actuator(std::string name_actuator) { path = name_actuator; }
};
} // namespace gazebo
#endif
