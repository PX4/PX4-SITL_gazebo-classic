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
#include "ThrusterStatus.pb.h"
// -----------------------------------------------------------------------------

namespace gazebo {
// Default PID gains
static double kPIDPitchP = 5.0;
static double kPIDPitchI = 0.0;
static double kPIDPitchD = 0.0;

static double kPIDPitchIMax = 0.0;
static double kPIDPitchIMin = 0.0;
static double kPIDPitchCmdMax = 0.3;
static double kPIDPitchCmdMin = -0.3;

static double kPIDRollP = 5.0;
static double kPIDRollI = 0.0;
static double kPIDRollD = 0.0;
static double kPIDRollIMax = 0.0;
static double kPIDRollIMin = 0.0;
static double kPIDRollCmdMax = 0.3;
static double kPIDRollCmdMin = -0.3;

static double kPIDYawP = 1.0;
static double kPIDYawI = 0.0;
static double kPIDYawD = 0.0;
static double kPIDYawIMax = 0.0;
static double kPIDYawIMin = 0.0;
static double kPIDYawCmdMax = 1.0;
static double kPIDYawCmdMin = -1.0;

// Default rotation directions
static double kRollDir = -1.0;
static double kPitchDir = -1.0;
static double kYawDir = 1.0;

typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;

class GAZEBO_VISIBLE ACSControllerPlugin : public ModelPlugin {
  /// \brief Constructor
public:
  ACSControllerPlugin();

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();

private:
  void OnUpdate();
  void GroundTruthCallback(GtPtr &imu_message);

#if GAZEBO_MAJOR_VERSION > 7 ||                                                \
    (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION >= 4)
  /// only gazebo 7.4 and above support Any
  void OnPitchStringMsg(ConstAnyPtr &_msg);
  void OnRollStringMsg(ConstAnyPtr &_msg);
  void OnYawStringMsg(ConstAnyPtr &_msg);
#else
private:
  void OnPitchStringMsg(ConstGzStringPtr &_msg);
  void OnRollStringMsg(ConstGzStringPtr &_msg);
  void OnYawStringMsg(ConstGzStringPtr &_msg);
#endif

  std::mutex cmd_mutex;
  sdf::ElementPtr sdf;
  std::vector<event::ConnectionPtr> connections;

  transport::SubscriberPtr imuSub;
  transport::SubscriberPtr pitchSub;
  transport::SubscriberPtr rollSub;
  transport::SubscriberPtr yawSub;
  transport::PublisherPtr pitchPub;
  transport::PublisherPtr rollPub;
  transport::PublisherPtr yawPub;

  physics::ModelPtr model;

  /// \brief yaw camera
  physics::JointPtr yawJoint;

  /// \brief camera roll joint
  physics::JointPtr rollJoint;

  /// \brief camera pitch joint
  physics::JointPtr pitchJoint;

  sensors::ImuSensorPtr cameraImuSensor;

  double vehicleYaw;

  std::string status;

  double rDir;
  double pDir;
  double yDir;

  // control commands
  double pitchCommand;
  double yawCommand;
  double rollCommand;

  transport::NodePtr node;

  common::PID pitchPid;
  common::PID rollPid;
  common::PID yawPid;

  // Custom Properties ---------------------------------------------------------
  // update time used for PID controller
  common::Time lastUpdateTime;

  // status variables to keep track of outputs
  std::vector<double> _thrusterStatus = {0, 0, 0, 0};
  double _newX;
  double _newY;
  double _rollTarget;
  double _pitchTarget;

  // status topics
  std::string new_xy_pub_topic_;
  std::string roll_pitch_pub_topic_;
  std::string thruster_pub_topic_;

  // status publishers
  transport::PublisherPtr new_xy_status_pub_;
  transport::PublisherPtr roll_pitch_status_pub_;
  transport::PublisherPtr thruster_status_pub_;
  // ---------------------------------------------------------------------------
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
