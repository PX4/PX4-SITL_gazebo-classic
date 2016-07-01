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
  this->pitchCommand = 0;
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
  std::string topic = std::string("~/") +  this->model->GetName() +
    "/gimbal_pitch_cmd";
  this->sub = this->node->Subscribe(topic,
     &GimbalControllerPlugin::OnStringMsg, this);

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GimbalControllerPlugin::OnUpdate, this)));

  topic = std::string("~/") +  this->model->GetName() + "/gimbal_pitch_status";
  this->pub = node->Advertise<gazebo::msgs::GzString>(topic);

  gzmsg << "GimbalControllerPlugin::Init" << std::endl;
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnStringMsg(ConstGzStringPtr &_msg)
{
  gzmsg << "pitch command received " << _msg->data() << std::endl;
  this->pitchCommand = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnUpdate()
{
  if (!this->pitchJoint)
    return;

  double pitchAngle = this->pitchJoint->GetAngle(0).Radian();
  double rollAngle = this->rollJoint->GetAngle(0).Radian();
  double yawAngle = this->yawJoint->GetAngle(0).Radian();

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

    double pitchError = pitchAngle - this->pitchCommand;
    double pitchForce = this->pitchPid.Update(pitchError, dt);
    this->pitchJoint->SetForce(0, pitchForce);

    double rollError = rollAngle - this->rollCommand;
    double rollForce = this->rollPid.Update(rollError, dt);
    this->rollJoint->SetForce(0, rollForce);

    double yawError = yawAngle - this->yawCommand;
    double yawForce = this->yawPid.Update(yawError, dt);
    this->yawJoint->SetForce(0, yawForce);

    this->lastUpdateTime = time;
  }

  static int i =1000;
  if (++i>100)
  {
    i = 0;
    std::stringstream ss;
    ss << pitchAngle;
    gazebo::msgs::GzString m;
    m.set_data(ss.str());
    this->pub->Publish(m);
  }
}


