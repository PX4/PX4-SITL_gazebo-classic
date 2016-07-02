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

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_gimbal_controller_plugin.hh>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalControllerPlugin)

/////////////////////////////////////////////////
GimbalControllerPlugin::GimbalControllerPlugin()
  :status("closed")
{
  this->pitchPid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
  this->rollPid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
  this->yawPid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
  this->pitchCommand = 0;  // 0.5* M_PI is problematic because of singularity
  this->rollCommand = 0;
  this->yawCommand = 0;
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->sdf = _sdf;

  std::string yawJointName = "cgo3_vertical_arm_joint";
  this->yawJoint = this->model->GetJoint(yawJointName);
  if (this->sdf->HasElement("joint_yaw"))
  {
    // Add names to map
    yawJointName = sdf->Get<std::string>("joint_yaw");
    if (this->model->GetJoint(yawJointName))
    {
      this->yawJoint = this->model->GetJoint(yawJointName);
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


  // get imu sensor
  std::string imuSensorName = "camera_imu";
  this->imuSensor = std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor(imuSensorName));
  if (this->sdf->HasElement("imu"))
  {
    // Add names to map
    imuSensorName = sdf->Get<std::string>("imu");
    if (this->model->GetJoint(imuSensorName))
    {
      this->imuSensor = std::static_pointer_cast<sensors::ImuSensor>(
        sensors::SensorManager::Instance()->GetSensor(imuSensorName));
    }
    else
    {
      gzwarn << "imu [" << imuSensorName << "] does not exist?\n";
    }
  }
  if (!this->imuSensor)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get imu sensor '"
          << imuSensorName << "' " << endl;
  }
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::Init()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();

  // receive pitch command via gz transport
  std::string pitchTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_pitch_cmd";
  this->pitchSub = this->node->Subscribe(pitchTopic,
     &GimbalControllerPlugin::OnPitchStringMsg, this);
  // receive roll command via gz transport
  std::string rollTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_roll_cmd";
  this->rollSub = this->node->Subscribe(rollTopic,
     &GimbalControllerPlugin::OnRollStringMsg, this);
  // receive yaw command via gz transport
  std::string yawTopic = std::string("~/") +  this->model->GetName() +
    "/gimbal_yaw_cmd";
  this->yawSub = this->node->Subscribe(yawTopic,
     &GimbalControllerPlugin::OnYawStringMsg, this);

  // plugin update
  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GimbalControllerPlugin::OnUpdate, this)));

  // publish pitch status via gz transport
  pitchTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_pitch_status";
  this->pitchPub = node->Advertise<gazebo::msgs::GzString>(pitchTopic);

  // publish roll status via gz transport
  rollTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_roll_status";
  this->rollPub = node->Advertise<gazebo::msgs::GzString>(rollTopic);

  // publish yaw status via gz transport
  yawTopic = std::string("~/") +  this->model->GetName()
    + "/gimbal_yaw_status";
  this->yawPub = node->Advertise<gazebo::msgs::GzString>(yawTopic);

  gzmsg << "GimbalControllerPlugin::Init" << std::endl;
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnPitchStringMsg(ConstGzStringPtr &_msg)
{
  gzmsg << "pitch command received " << _msg->data() << std::endl;
  this->pitchCommand = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnRollStringMsg(ConstGzStringPtr &_msg)
{
  gzmsg << "roll command received " << _msg->data() << std::endl;
  this->rollCommand = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnYawStringMsg(ConstGzStringPtr &_msg)
{
  gzmsg << "yaw command received " << _msg->data() << std::endl;
  this->yawCommand = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnUpdate()
{
  if (!this->pitchJoint || !this->rollJoint || !this->yawJoint)
    return;


  ignition::math::Vector3d eulers = this->imuSensor->Orientation().Euler();
  // double pitchAngle = this->pitchJoint->GetAngle(0).Radian();
  // double rollAngle = this->rollJoint->GetAngle(0).Radian();
  // double yawAngle = this->yawJoint->GetAngle(0).Radian();
  double rollAngle = this->NormalizeAbout(-eulers.X(), 0.0);
  double pitchAngle = this->NormalizeAbout(-eulers.Y(), 0.0);
  double yawAngle = this->NormalizeAbout(eulers.Z(), 0.0);
  // gzerr << "[" << eulers << "] [" << rollAngle
  //       << ", " << pitchAngle << ", " << yawAngle << "]\n";

  common::Time time = this->model->GetWorld()->GetSimTime();
  if (time < this->lastUpdateTime)
  {
    gzerr << "time reset event\n";
    this->lastUpdateTime = time;
    return;
  }
  else if (time > this->lastUpdateTime)
  {
    double dt = (this->lastUpdateTime - time).Double();

    double pitchError =
      this->ShortestAngularDistance(this->pitchCommand, pitchAngle );
    double pitchForce = this->pitchPid.Update(pitchError, dt);
    this->pitchJoint->SetForce(0, pitchForce);

    double rollError =
      this->ShortestAngularDistance(this->rollCommand, rollAngle );
    double rollForce = this->rollPid.Update(rollError, dt);
    this->rollJoint->SetForce(0, rollForce);

    double yawError =
      this->ShortestAngularDistance(this->yawCommand, yawAngle );
    double yawForce = this->yawPid.Update(yawError, dt);
    this->yawJoint->SetForce(0, yawForce);

    this->lastUpdateTime = time;
  }

  static int i =1000;
  if (++i>100)
  {
    i = 0;
    std::stringstream ss;
    gazebo::msgs::GzString m;

    ss << this->pitchJoint->GetAngle(0).Radian();
    m.set_data(ss.str());
    this->pitchPub->Publish(m);

    ss << this->rollJoint->GetAngle(0).Radian();
    m.set_data(ss.str());
    this->rollPub->Publish(m);

    ss << this->yawJoint->GetAngle(0).Radian();
    m.set_data(ss.str());
    this->yawPub->Publish(m);
  }
}

/////////////////////////////////////////////////
double GimbalControllerPlugin::NormalizeAbout(double _angle, double reference)
{
  double diff = _angle - reference;
  // normalize diff about (-pi, pi], then add reference
  while (diff <= -M_PI)
  {
    diff += M_PI;
  }
  return fmod(diff, (2.0*M_PI)) + reference;
}

/////////////////////////////////////////////////
double GimbalControllerPlugin::ShortestAngularDistance(double _from, double _to)
{
  return this->NormalizeAbout(_to, _from) - _from;
}
