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

#include "common.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_gimbal_controller_plugin.hh>
#include <errno.h>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)

/* Keep these functions in the 'detail' namespace so that they
 * can be called from unit tests. */
namespace detail {
/////////////////////////////////////////////////
ignition::math::Vector3d ThreeAxisRot(
  double r11, double r12, double r21, double r31, double r32)
{
  return ignition::math::Vector3d(
    atan2( r31, r32 ),
    asin ( r21 ),
    atan2( r11, r12 ));
}

/////////////////////////////////////////////////
/// \TODO something to move into Angle class
/// \brief returns _angle1 normalized about
/// (_reference - M_PI, _reference + M_PI]
/// \param[in] _angle1 input angle
/// \param[in] _reference reference input angle for normalization
/// \return normalized _angle1 about _reference
double NormalizeAbout(double _angle, double reference)
{
  double diff = _angle - reference;
  // normalize diff about (-pi, pi], then add reference
  while (diff <= -M_PI)
  {
    diff += 2.0*M_PI;
  }
  while (diff > M_PI)
  {
    diff -= 2.0*M_PI;
  }
  return diff + reference;
}

/////////////////////////////////////////////////
/// \TODO something to move into Angle class
/// \brief returns shortest angular distance from _from to _to
/// \param[in] _from starting anglular position
/// \param[in] _to end angular position
/// \return distance traveled from starting to end angular positions
double ShortestAngularDistance(double _from, double _to)
{
  return NormalizeAbout(_to, _from) - _from;
}

/////////////////////////////////////////////////
ignition::math::Vector3d QtoZXY(
  const ignition::math::Quaterniond &_q)
{
  // taken from
  // http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
  // case zxy:
  ignition::math::Vector3d result = detail::ThreeAxisRot(
    -2*(_q.X()*_q.Y() - _q.W()*_q.Z()),
    _q.W()*_q.W() - _q.X()*_q.X() + _q.Y()*_q.Y() - _q.Z()*_q.Z(),
    2*(_q.Y()*_q.Z() + _q.W()*_q.X()),
    -2*(_q.X()*_q.Z() - _q.W()*_q.Y()),
    _q.W()*_q.W() - _q.X()*_q.X() - _q.Y()*_q.Y() + _q.Z()*_q.Z());
  return result;
}
}


/////////////////////////////////////////////////
GimbalControllerPlugin::GimbalControllerPlugin()
  :status("closed")
{
  /// defaults if sdf xml doesn't contain any pid gains
  this->pitchPid.Init(kPIDPitchP, kPIDPitchI, kPIDPitchD, kPIDPitchIMax, kPIDPitchIMin, kPIDPitchCmdMax, kPIDPitchCmdMin);
  this->rollPid.Init(kPIDRollP, kPIDRollI, kPIDRollD, kPIDRollIMax, kPIDRollIMin, kPIDRollCmdMax, kPIDRollCmdMin);
  this->yawPid.Init(kPIDYawP, kPIDYawI, kPIDYawD, kPIDYawIMax, kPIDYawIMin, kPIDYawCmdMax, kPIDYawCmdMin);
  this->rDir = kRollDir;
  this->pDir = kPitchDir;
  this->yDir = kYawDir;
}

GimbalControllerPlugin::~GimbalControllerPlugin()
{
  shouldExit = true;
  if (rxThread) {
    rxThread->join();
  }
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->sdf = _sdf;

  // Create axis name -> pid map. It may be empty or not fully defined
  std::map<std::string, common::PID> pids_;

  if (_sdf->HasElement("control_gimbal_channels"))
  {
    sdf::ElementPtr control_channels = _sdf->GetElement("control_gimbal_channels");
    sdf::ElementPtr channel = control_channels->GetElement("channel");
    while(channel)
    {
      if (channel->HasElement("joint_axis"))
      {
        std::string joint_axis = channel->Get<std::string>("joint_axis");

        // setup joint control pid to control joint
        if (channel->HasElement("joint_control_pid"))
        {
          sdf::ElementPtr pid = channel->GetElement("joint_control_pid");
          double p = 0;
          if (pid->HasElement("p"))
            p = pid->Get<double>("p");
          double i = 0;
          if (pid->HasElement("i"))
            i = pid->Get<double>("i");
          double d = 0;
          if (pid->HasElement("d"))
            d = pid->Get<double>("d");
          double iMax = 0;
          if (pid->HasElement("iMax"))
            iMax = pid->Get<double>("iMax");
          double iMin = 0;
          if (pid->HasElement("iMin"))
            iMin = pid->Get<double>("iMin");
          double cmdMax = 0;
          if (pid->HasElement("cmdMax"))
            cmdMax = pid->Get<double>("cmdMax");
          double cmdMin = 0;
          if (pid->HasElement("cmdMin"))
            cmdMin = pid->Get<double>("cmdMin");

          // insert pid gains into map for the respective named joint axis
          pids_.insert(std::pair<std::string, common::PID>(joint_axis, common::PID(p, i, d, iMax, iMin, cmdMax, cmdMin)));
        }
        channel = channel->GetNextElement("channel");
      }
    }
  }
  else
  {
    gzwarn << "Control channels for gimbal not found. Using default pid gains\n";
  }

  std::string yawJointName = "cgo3_vertical_arm_joint";
  this->yawJoint = this->model->GetJoint(yawJointName);
  if (this->sdf->HasElement("joint_yaw"))
  {
    // Add names to map
    yawJointName = sdf->Get<std::string>("joint_yaw");
    if (this->model->GetJoint(yawJointName))
    {
      this->yawJoint = this->model->GetJoint(yawJointName);

      // Try to find yaw rotation direction
      sdf::ElementPtr sdfElem = this->yawJoint->GetSDF();
      if(sdfElem->HasElement("axis"))
      {
        // Rotation is found
#if GAZEBO_MAJOR_VERSION >= 9
        yDir = this->yawJoint->LocalAxis(0)[2];
#else
        yDir = this->yawJoint->GetLocalAxis(0)[2];
#endif
      }
      else
      {
        // If user do not defines axis for yaw joint explicitly
        // then display warning
        gzwarn << "joint_yaw [" << yawJointName << "] axis do not defined?\n";
      }

      // Try to find respective pid for the named axis control
      std::map<std::string, common::PID>::iterator it = pids_.find("joint_yaw");
      if(it != pids_.end())
      {
        // Found pid for this axis (and therefore for this joint)
        this->yawPid = it->second;
      }
      else
      {
        // If user defines control channels for gimbal but don't define yaw gains explicitly
        // then display warning
        gzwarn << "joint_yaw [" << yawJointName << "] pid control gains do not defined?\n";
      }
    }
    else
    {
      gzwarn << "joint_yaw [" << yawJointName << "] does not exist?\n";
    }
  }
  if (!this->yawJoint)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get yaw joint '"
          << yawJointName << "' " << endl;
  }

  std::string rollJointName = "cgo3_horizontal_arm_joint";
  this->rollJoint = this->model->GetJoint(rollJointName);
  if (this->sdf->HasElement("joint_roll"))
  {
    // Add names to map
    rollJointName = sdf->Get<std::string>("joint_roll");
    if (this->model->GetJoint(rollJointName))
    {
      this->rollJoint = this->model->GetJoint(rollJointName);

      // Try to find roll rotation direction
      sdf::ElementPtr sdfElem = this->rollJoint->GetSDF();
      if(sdfElem->HasElement("axis"))
      {
        // Rotation is found
#if GAZEBO_MAJOR_VERSION >= 9
        rDir = this->rollJoint->LocalAxis(0)[0];
#else
        rDir = this->rollJoint->GetLocalAxis(0)[0];
#endif
      }
      else
      {
        // If user do not defines axis for roll joint explicitly
        // then display warning
        gzwarn << "joint_roll [" << rollJointName << "] axis do not defined?\n";
      }

      // Try to find respective pid for the named axis control
      std::map<std::string, common::PID>::iterator it = pids_.find("joint_roll");
      if(it != pids_.end())
      {
        // Found pid for this axis (and therefore for this joint)
        this->rollPid = it->second;
      }
      else
      {
        // If user defines control channels for gimbal but don't define roll gains explicitly
        // then display warning
        gzwarn << "joint_roll [" << rollJointName << "] pid control gains do not defined?\n";
      }
    }
    else
    {
      gzwarn << "joint_roll [" << rollJointName << "] does not exist?\n";
    }
  }
  if (!this->rollJoint)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get roll joint '"
          << rollJointName << "' " << endl;
  }

  std::string pitchJointName = "cgo3_camera_joint";
  this->pitchJoint = this->model->GetJoint(pitchJointName);
  if (this->sdf->HasElement("joint_pitch"))
  {
    // Add names to map
    pitchJointName = sdf->Get<std::string>("joint_pitch");
    if (this->model->GetJoint(pitchJointName))
    {
      this->pitchJoint = this->model->GetJoint(pitchJointName);

      // Try to find pitch rotation direction
      sdf::ElementPtr sdfElem = this->pitchJoint->GetSDF();
      if(sdfElem->HasElement("axis"))
      {
        // Rotation is found
#if GAZEBO_MAJOR_VERSION >= 9
        pDir = this->pitchJoint->LocalAxis(0)[1];
#else
        pDir = this->pitchJoint->GetLocalAxis(0)[1];
#endif
      }
      else
      {
        // If user do not defines axis for pitch joint explicitly
        // then display warning
        gzwarn << "joint_pitch [" << pitchJointName << "] axis do not defined?\n";
      }

      // Try to find respective pid for the named axis
      std::map<std::string, common::PID>::iterator it = pids_.find("joint_pitch");
      if(it != pids_.end())
      {
        // Found pid for this axis (and therefore for this joint)
        this->pitchPid = it->second;
      }
      else
      {
        // If user defines control channels for gimbal but don't define pitch gains explicitly
        // then display warning
        gzwarn << "joint_pitch [" << pitchJointName << "] pid control gains do not defined?\n";
      }
    }
    else
    {
      gzwarn << "joint_pitch [" << pitchJointName << "] does not exist?\n";
    }
  }
  if (!this->pitchJoint)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get pitch joint '"
          << pitchJointName << "' " << endl;
  }

  // get imu sensors
  std::string cameraImuSensorName = "camera_imu";
  if (this->sdf->HasElement("gimbal_imu"))
  {
    // Add names to map
    cameraImuSensorName = sdf->Get<std::string>("gimbal_imu");
  }
#if GAZEBO_MAJOR_VERSION >= 7
  this->cameraImuSensor = std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(_model->SensorScopedName(cameraImuSensorName)[0]));
#elif GAZEBO_MAJOR_VERSION >= 6
  this->cameraImuSensor = boost::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(cameraImuSensorName));
#endif
  if (!this->cameraImuSensor)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get imu sensor '"
          << cameraImuSensorName << "' " << endl;
  }

  const char *host_ip = std::getenv("PX4_VIDEO_HOST_IP");
  if (host_ip) {
    this->udp_gimbal_host_ip = std::string(host_ip);
  } else if (this->sdf->HasElement("udp_gimbal_host_ip")) {
    this->udp_gimbal_host_ip =  _sdf->Get<std::string>("udp_gimbal_host_ip");
  } else {
    this->udp_gimbal_host_ip = "127.0.0.1";
  }

  if (this->sdf->HasElement("udp_gimbal_port_remote")) {
    this->udp_gimbal_port_remote = _sdf->Get<int>("udp_gimbal_port_remote");
  } else {
    this->udp_gimbal_port_remote = 13030;
  }
  gzwarn << "[gazebo_gimbal_controller_plugin] Streaming gimbal mavlink stream to ip: " << this->udp_gimbal_host_ip  << " port: " << this->udp_gimbal_port_remote << std::endl;
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::Init()
{
  this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION >= 9
  this->node->Init(this->model->GetWorld()->Name());
  this->lastUpdateTime = this->model->GetWorld()->SimTime();
#else
  this->node->Init(this->model->GetWorld()->GetName());
  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();
#endif

  // plugin update
  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GimbalControllerPlugin::OnUpdate, this)));

  if (InitUdp()) {
    rxThread = std::make_unique<std::thread>(&GimbalControllerPlugin::RxThread, this);
  }
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnUpdate()
{
  const std::lock_guard<std::mutex> lock(cmd_mutex);

  if (!this->pitchJoint || !this->rollJoint || !this->yawJoint)
    return;

#if GAZEBO_MAJOR_VERSION >= 9
  common::Time time = this->model->GetWorld()->SimTime();
#else
  common::Time time = this->model->GetWorld()->GetSimTime();
#endif
  if (time < this->lastUpdateTime)
  {
    gzerr << "time reset event\n";
    this->lastUpdateTime = time;
    return;
  }
  else if (time > this->lastUpdateTime)
  {
    double dt = (time - this->lastUpdateTime).Double();

    {
      const std::lock_guard<std::mutex> lock(setpointMutex);

      const auto maybeNewRollSetpoint = calcSetpoint(
          dt, this->lastRollSetpoint, this->rollSetpoint, this->rollRateSetpoint);
      if (maybeNewRollSetpoint) {
        this->lastRollSetpoint = maybeNewRollSetpoint.value();
      }

      // Without the minus, the gimbal pitch is inverted.
      const auto maybeNewPitchSetpoint = calcSetpoint(
          dt, this->lastPitchSetpoint, -this->pitchSetpoint, -this->pitchRateSetpoint);
      if (maybeNewPitchSetpoint) {
        this->lastPitchSetpoint = maybeNewPitchSetpoint.value();
      }

      // Without this minus, the gimbal yaw is inverted.
      const auto maybeNewYawSetpoint = calcSetpoint(
          dt, this->lastYawSetpoint, -this->yawSetpoint, -this->yawRateSetpoint);
      if (maybeNewYawSetpoint) {
        this->lastYawSetpoint = maybeNewYawSetpoint.value();
      }
    }

    // truncate command inside joint angle limits
#if GAZEBO_MAJOR_VERSION >= 9
    double rollLimited = ignition::math::clamp(this->lastRollSetpoint,
      rDir*this->rollJoint->UpperLimit(0),
    rDir*this->rollJoint->LowerLimit(0));
    double pitchLimited = ignition::math::clamp(this->lastPitchSetpoint,
      pDir*this->pitchJoint->UpperLimit(0),
      pDir*this->pitchJoint->LowerLimit(0));
    double yawLimited = ignition::math::clamp(this->lastYawSetpoint + (this->yawLock ? M_PI/2.0 : this->vehicleYawRad),
      yDir*this->yawJoint->LowerLimit(0),
    yDir*this->yawJoint->UpperLimit(0));
#else
    double rollLimited = ignition::math::clamp(this->lastRollSetpoint,
      rDir*this->rollJoint->GetUpperLimit(0).Radian(),
    rDir*this->rollJoint->GetLowerLimit(0).Radian());
    double pitchLimited = ignition::math::clamp(this->lastPitchSetpoint,
      pDir*this->pitchJoint->GetUpperLimit(0).Radian(),
      pDir*this->pitchJoint->GetLowerLimit(0).Radian());
    double yawLimited = ignition::math::clamp(this->lastYawSetpoint + (this->yawLock ? M_PI/2.0 : this->vehicleYawRad),
      yDir*this->yawJoint->GetLowerLimit(0).Radian(),
    yDir*this->yawJoint->GetUpperLimit(0).Radian());
#endif

    /// currentAngleYPRVariable is defined in roll-pitch-yaw-fixed-axis
    /// and gimbal is constructed using yaw-roll-pitch-variable-axis
    ignition::math::Vector3d currentAngleYPRVariable(
      this->cameraImuSensor->Orientation().Euler());

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Vector3d currentAnglePRYVariable(
      detail::QtoZXY(ignition::math::Quaterniond(currentAngleYPRVariable)));
#else
    ignition::math::Vector3d currentAnglePRYVariable(
      detail::QtoZXY(currentAngleYPRVariable));
#endif

    /// get joint limits (in sensor frame)
    /// TODO: move to Load() if limits do not change
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d lowerLimitsPRY
      (pDir*this->pitchJoint->LowerLimit(0),
       rDir*this->rollJoint->LowerLimit(0),
       yDir*this->yawJoint->LowerLimit(0));
    ignition::math::Vector3d upperLimitsPRY
      (pDir*this->pitchJoint->UpperLimit(0),
       rDir*this->rollJoint->UpperLimit(0),
       yDir*this->yawJoint->UpperLimit(0));
#else
    ignition::math::Vector3d lowerLimitsPRY
      (pDir*this->pitchJoint->GetLowerLimit(0).Radian(),
       rDir*this->rollJoint->GetLowerLimit(0).Radian(),
       yDir*this->yawJoint->GetLowerLimit(0).Radian());
    ignition::math::Vector3d upperLimitsPRY
      (pDir*this->pitchJoint->GetUpperLimit(0).Radian(),
       rDir*this->rollJoint->GetUpperLimit(0).Radian(),
       yDir*this->yawJoint->GetUpperLimit(0).Radian());
#endif

    // normalize errors
    double pitchError = detail::ShortestAngularDistance(
      pitchLimited, currentAnglePRYVariable.X());
    double rollError = detail::ShortestAngularDistance(
      rollLimited, currentAnglePRYVariable.Y());
    double yawError = detail::ShortestAngularDistance(
      yawLimited, currentAnglePRYVariable.Z());

    // Clamp errors based on current angle and estimated errors from rotations:
    // given error = current - target, then
    // if target (current angle - error) is outside joint limit, truncate error
    // so that current angle - error is within joint limit, i.e.:
    // lower limit < current angle - error < upper limit
    // or
    // current angle - lower limit > error > current angle - upper limit
    // re-expressed as clamps:
    // hardcoded negative joint axis for pitch and roll
    if (lowerLimitsPRY.X() < upperLimitsPRY.X())
    {
      pitchError = ignition::math::clamp(pitchError,
        currentAnglePRYVariable.X() - upperLimitsPRY.X(),
        currentAnglePRYVariable.X() - lowerLimitsPRY.X());
    }
    else
    {
      pitchError = ignition::math::clamp(pitchError,
        currentAnglePRYVariable.X() - lowerLimitsPRY.X(),
        currentAnglePRYVariable.X() - upperLimitsPRY.X());
    }
    if (lowerLimitsPRY.Y() < upperLimitsPRY.Y())
    {
      rollError = ignition::math::clamp(rollError,
        currentAnglePRYVariable.Y() - upperLimitsPRY.Y(),
        currentAnglePRYVariable.Y() - lowerLimitsPRY.Y());
    }
    else
    {
      rollError = ignition::math::clamp(rollError,
        currentAnglePRYVariable.Y() - lowerLimitsPRY.Y(),
        currentAnglePRYVariable.Y() - upperLimitsPRY.Y());
    }
    if (lowerLimitsPRY.Z() < upperLimitsPRY.Z())
    {
      yawError = ignition::math::clamp(yawError,
        currentAnglePRYVariable.Z() - upperLimitsPRY.Z(),
        currentAnglePRYVariable.Z() - lowerLimitsPRY.Z());
    }
    else
    {
      yawError = ignition::math::clamp(yawError,
        currentAnglePRYVariable.Z() - lowerLimitsPRY.Z(),
        currentAnglePRYVariable.Z() - upperLimitsPRY.Z());
    }

    // apply forces to move gimbal
    double pitchForce = this->pitchPid.Update(pitchError, dt);
    this->pitchJoint->SetForce(0, pDir*pitchForce);

    double rollForce = this->rollPid.Update(rollError, dt);
    this->rollJoint->SetForce(0, rDir*rollForce);

    double yawForce = this->yawPid.Update(yawError, dt);
    this->yawJoint->SetForce(0, yDir*yawForce);

    //gzdbg << "rF: " << rollForce << ", pF: " << pitchForce << ", yF: " << yawForce << "\n";

    this->lastUpdateTime = time;
  }

  if (time > this->lastHeartbeatSentTime + heartbeatIntervalS) {
    SendHeartbeat();
    this->lastHeartbeatSentTime = time;
  }
  if (sendingAttitudeStatus && time > this->lastAttitudeStatusSentTime + attitudeStatusIntervalS) {
    SendGimbalDeviceAttitudeStatus();
    this->lastAttitudeStatusSentTime = time;
  }
}

bool GimbalControllerPlugin::InitUdp()
{
  if ((this->sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    gzerr << "socket creation failed: " << strerror(errno) << endl;
    return false;
  }

  this->myaddr = {};
  this->myaddr.sin_family = AF_INET;
  this->myaddr.sin_addr.s_addr = htonl(INADDR_ANY);

  this->myaddr.sin_port = htons(0);
  if (::bind(this->sock, reinterpret_cast<sockaddr *>(&this->myaddr), sizeof(this->myaddr)) < 0) {
    gzerr << "bind failed: " << strerror(errno) << endl;
    return false;
  }

  struct timeval tv {};
  tv.tv_sec = 0;
  tv.tv_usec = 100000;
  if (setsockopt(this->sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    gzerr << "setsockopt failed: " << strerror(errno) << endl;
    return false;
  }

  mavlink_status_t* chan_state = mavlink_get_channel_status(mavlinkChannel);
  chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);

  return true;
}

void GimbalControllerPlugin::SendHeartbeat()
{
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack_chan(
    ourSysid,
    ourCompid,
    mavlinkChannel,
    &msg,
    MAV_TYPE_GIMBAL,
    MAV_AUTOPILOT_INVALID,
    0,
    0,
    0);
  SendMavlinkMessage(msg);
}

void GimbalControllerPlugin::SendGimbalDeviceInformation()
{
  const common::Time time = this->model->GetWorld()->SimTime();
  const uint32_t timeMs = time.sec * 1000 + time.nsec / 1000000;

  const uint8_t firmwareDevVersion = 0;
  const uint8_t firmwarePatchVersion = 0;
  const uint8_t firmwareMinorVersion = 0;
  const uint8_t firmwareMajorVersion = 2;
  const uint32_t firmwareVersion =
    (firmwareDevVersion & 0xff) << 24 |
    (firmwarePatchVersion & 0xff) << 16 |
    (firmwareMinorVersion & 0xff) << 8 |
    (firmwareMajorVersion & 0xff);

  const uint8_t hardwareDevVersion = 0;
  const uint8_t hardwarePatchVersion = 0;
  const uint8_t hardwareMinorVersion = 0;
  const uint8_t hardwareMajorVersion = 1;
  const uint32_t hardwareVersion =
    (hardwareDevVersion & 0xff) << 24 |
    (hardwarePatchVersion & 0xff) << 16 |
    (hardwareMinorVersion & 0xff) << 8 |
    (hardwareMajorVersion & 0xff);

  const uint64_t uid = 0x2f0b7a92295a7b3c; // random.org gave me something.

  const uint16_t capFlags =
    GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL |
    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS |
    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK |
    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS |
    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK |
    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS |
    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW |
    GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW;

  const float rollMax = this->rollJoint->UpperLimit(0);
  const float rollMin = this->rollJoint->LowerLimit(0);
  const float pitchMax = this->pitchJoint->UpperLimit(0);
  const float pitchMin = this->pitchJoint->LowerLimit(0);
  const float yawMax = this->yawJoint->LowerLimit(0);
  const float yawMin = this->yawJoint->UpperLimit(0);

  mavlink_message_t msg;
  mavlink_msg_gimbal_device_information_pack_chan(
    ourSysid,
    ourCompid,
    mavlinkChannel,
    &msg,
    timeMs,
    std::string("PX4").c_str(),
    std::string("Gazebo SITL").c_str(),
    std::string("").c_str(), // custom_name
    firmwareVersion,
    hardwareVersion,
    uid,
    capFlags,
    0, // custom_cap_flags
    rollMin,
    rollMax,
    pitchMin,
    pitchMax,
    yawMin,
    yawMax);
  SendMavlinkMessage(msg);
}

void GimbalControllerPlugin::SendGimbalDeviceAttitudeStatus()
{
  const common::Time time = this->model->GetWorld()->SimTime();
  const uint32_t timeMs = time.sec * 1000 + time.nsec / 1000000;

  const uint16_t flags =
    GIMBAL_DEVICE_FLAGS_ROLL_LOCK |
    GIMBAL_DEVICE_FLAGS_PITCH_LOCK |
    (this->yawLock ? GIMBAL_DEVICE_FLAGS_YAW_LOCK : 0);

  auto q = q_ENU_to_NED * this->cameraImuSensor->Orientation() * q_FLU_to_FRD.Inverse();

  if (!this->yawLock) {
    // In follow mode we need to transform the absolute camera orientation to an orientation
    // relative to the vehicle because that's what the gimbal protocol suggests.
    const auto q_vehicle = q_ENU_to_NED * ignition::math::Quaterniond(0.0, 0.0, this->vehicleYawRad) * q_FLU_to_FRD.Inverse();
    const auto e = q.Euler();
    q.Euler(e[0], e[1], e[2] - q_vehicle.Euler()[2]);
  }

  const float qArr[4] = {
    static_cast<float>(q.W()),
    static_cast<float>(q.X()),
    static_cast<float>(q.Y()),
    static_cast<float>(q.Z())
  };

  // auto e = q.Euler();
  // gzdbg << "r: " << degrees(e[0]) << ", p: " << degrees(e[1]) << ", y: " << degrees(e[2]) << "\n";

  const auto angularVelocity = q_FLU_to_FRD.RotateVector(this->cameraImuSensor->AngularVelocity());

  const uint16_t failureFlags {0};

  mavlink_message_t msg;
  mavlink_msg_gimbal_device_attitude_status_pack_chan(
    ourSysid,
    ourCompid,
    mavlinkChannel,
    &msg,
    0, // broadcast
    0, // broadcast
    timeMs,
    flags,
    qArr,
    angularVelocity.X(),
    angularVelocity.Y(),
    angularVelocity.Z(),
    failureFlags,
    NAN, // per mavlink spec - NAN if unknown
    NAN); // per mavlink spec - NAN if unknown
  SendMavlinkMessage(msg);
}

void GimbalControllerPlugin::SendResult(uint8_t target_sysid, uint8_t target_compid, uint16_t command, MAV_RESULT result)
{
  mavlink_message_t msg;
  mavlink_msg_command_ack_pack_chan(
    ourSysid,
    ourCompid,
    mavlinkChannel,
    &msg,
    command,
    result,
    255, // progress
    0, // result_param2
    target_sysid,
    target_compid);
  SendMavlinkMessage(msg);
}

void GimbalControllerPlugin::SendMavlinkMessage(const mavlink_message_t &msg)
{
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  const int packetlen = mavlink_msg_to_send_buffer(buffer, &msg);

  sockaddr_in dest_addr {};
  dest_addr.sin_family = AF_INET;
  inet_pton(AF_INET, this->udp_gimbal_host_ip.c_str(), &dest_addr.sin_addr.s_addr);
  dest_addr.sin_port = htons(this->udp_gimbal_port_remote);

  const ssize_t len = sendto(this->sock, buffer, packetlen, 0, reinterpret_cast<sockaddr *>(&dest_addr), sizeof(dest_addr));
  if (len <= 0) {
    gzerr << "Failed sending mavlink message: " << strerror(errno) << endl;
  }
}

void GimbalControllerPlugin::RxThread()
{
  while (!shouldExit) {
    sockaddr srcaddr;
    socklen_t addrlen = sizeof(srcaddr);
    unsigned char buffer[2048]; // enough for MTU 1500
    const int len = recvfrom(this->sock, buffer, sizeof(buffer), 0, reinterpret_cast<sockaddr *>(&srcaddr), &addrlen);
    if (len > 0) {
      for (unsigned i = 0; i < len; ++i)
      {
        mavlink_status_t status;
        mavlink_message_t msg;
        if (mavlink_parse_char(mavlinkChannel, buffer[i], &msg, &status))
        {
          HandleMessage(msg);
        }
      }
    }
  }
}

void GimbalControllerPlugin::HandleMessage(const mavlink_message_t& msg)
{
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE:
      HandleGimbalDeviceSetAttitude(msg);
      break;
    case MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE:
      HandleAutopilotStateForGimbalDevice(msg);
      break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
      HandleCommandLong(msg);
      break;
  }
}

void GimbalControllerPlugin::HandleCommandLong(const mavlink_message_t& msg)
{
  mavlink_command_long_t command_long;
  mavlink_msg_command_long_decode(&msg, &command_long);

  const bool addressedToUs =
    (command_long.target_system == ourSysid && command_long.target_component == ourCompid);


  switch (command_long.command) {
    case MAV_CMD_REQUEST_MESSAGE:
      HandleRequestMessage(msg.sysid, msg.compid, command_long);
      break;
    case MAV_CMD_SET_MESSAGE_INTERVAL:
      HandleSetMessageInterval(msg.sysid, msg.compid, command_long);
      break;
    default:
      if (addressedToUs) {
        SendResult(msg.sysid, msg.compid, command_long.command, MAV_RESULT_UNSUPPORTED);
      }
      break;
  }
}

void GimbalControllerPlugin::HandleGimbalDeviceSetAttitude(const mavlink_message_t& msg)
{
  mavlink_gimbal_device_set_attitude_t set_attitude;
  mavlink_msg_gimbal_device_set_attitude_decode(&msg, &set_attitude);

  if ((set_attitude.flags & GIMBAL_DEVICE_FLAGS_NEUTRAL) != 0) {
    const std::lock_guard<std::mutex> lock(setpointMutex);
    this->rollSetpoint = 0.0f;
    this->pitchSetpoint = 0.0f;
    this->yawSetpoint = 0.0f;
    this->yawLock = false;
    this->rollRateSetpoint = NAN;
    this->pitchRateSetpoint = NAN;
    this->yawRateSetpoint = NAN;

  } else {
    const auto euler = detail::QtoZXY(ignition::math::Quaterniond(
			    set_attitude.q[0], set_attitude.q[1], set_attitude.q[2], set_attitude.q[3]));
    const float pitchRad = euler[0];
    const float rollRad = euler[1];
    const float yawRad = euler[2];

    const std::lock_guard<std::mutex> lock(setpointMutex);
    this->rollSetpoint = rollRad;
    this->pitchSetpoint = pitchRad;
    this->yawSetpoint = yawRad;
    this->yawLock = (set_attitude.flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK);
    this->rollRateSetpoint = set_attitude.angular_velocity_x;
    this->pitchRateSetpoint = set_attitude.angular_velocity_y;
    this->yawRateSetpoint = set_attitude.angular_velocity_z;
  }
}


void GimbalControllerPlugin::HandleAutopilotStateForGimbalDevice(const mavlink_message_t& msg)
{
  mavlink_autopilot_state_for_gimbal_device_t autopilot_state;
  mavlink_msg_autopilot_state_for_gimbal_device_decode(&msg, &autopilot_state);

  float rollRad, pitchRad, yawRad;
  mavlink_quaternion_to_euler(&autopilot_state.q[0], &rollRad, &pitchRad, &yawRad);

  this->vehicleYawRad = -yawRad + M_PI/2.0; // Convert from NED to ENU.
}

void GimbalControllerPlugin::HandleRequestMessage(uint8_t target_sysid, uint8_t target_compid, const mavlink_command_long_t& command_long)
{
  switch (static_cast<uint16_t>(command_long.param1)) {
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
      SendResult(target_sysid, target_compid, command_long.command, MAV_RESULT_ACCEPTED);
      SendGimbalDeviceInformation();
      break;
    default:
      // Ignore messages that we don't support.
      break;
  }
}

void GimbalControllerPlugin::HandleSetMessageInterval(uint8_t target_sysid, uint8_t target_compid, const mavlink_command_long_t& command_long)
{
  switch (static_cast<uint16_t>(command_long.param1)) {
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
      if (command_long.param2 == -1.0f) {
        sendingAttitudeStatus = false;
      } else if (command_long.param2 == 0.0f) {
        sendingAttitudeStatus = true;
        sendingAttitudeStatus = defaultAttitudeStatusIntervalS;
      } else {
        sendingAttitudeStatus = true;
        sendingAttitudeStatus = command_long.param2 / 1000000.0f; // microseconds to seconds
      }

      SendResult(target_sysid, target_compid, command_long.command, MAV_RESULT_ACCEPTED);
      break;
    default:
      SendResult(target_sysid, target_compid, command_long.command, MAV_RESULT_DENIED);
      break;
  }
}

std::optional<double> GimbalControllerPlugin::calcSetpoint(double dt, double lastSetpoint, double newSetpoint, double newRateSetpoint)
{
  const bool setpointValid = std::isfinite(newSetpoint);
  const bool rateSetpointValid = std::isfinite(newRateSetpoint);

  if (rateSetpointValid) {
    const double rateDiff = dt * newRateSetpoint;
    const double setpointFromRate = lastSetpoint + rateDiff;

    if (setpointValid) {
      // In this case angle and rate are valid, so we use the rate but constrain it by the angle.
      if (rateDiff > 0.0) {
        return {std::min(newSetpoint, setpointFromRate)};
      } else {
        return {std::max(newSetpoint, setpointFromRate)};
      }

    } else {
      // Only the rate is valid, so we just use it.
      return {setpointFromRate};
    }

  } else if (setpointValid) {
    // Only the angle is valid.
    return {newSetpoint};

  } else {
    // Neither is valid.
    return {};
  }
}
