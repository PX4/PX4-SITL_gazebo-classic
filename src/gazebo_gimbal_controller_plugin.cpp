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
  :status("closed"), command(false)
{
  this->pitchPid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
  this->rollPid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
  this->yawPid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
  this->command = M_PI/2.0;
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;


  std::string yawJointName = "cgo3_vertical_arm_joint";
  std::string rollJointName = "cgo3_horizontal_arm_joint";
  std::string pitchJointName = "cgo3_camera_joint";

  this->pitchJoint = this->model->GetJoint(pitchJointName);
  if (!this->pitchJoint)
  {
    gzerr << "GimbalControllerPlugin::Load ERROR! Can't get pitch(tilt) joint '"
          << pitchJointName << "' " << endl;
  }
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::Init()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  this->lastUpdateTime = this->model->GetWorld()->GetSimTime();

  std::string topic = std::string("~/") +  this->model->GetName() +
    "/gimbal_tilt_cmd";
  this->sub = this->node->Subscribe(topic,
                                       &GimbalControllerPlugin::OnStringMsg,
                                       this);

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GimbalControllerPlugin::OnUpdate, this)));

  topic = std::string("~/") +  this->model->GetName() + "/gimbal_tilt_status";
  this->pub = node->Advertise<gazebo::msgs::GzString>(topic);

  gzmsg << "GimbalControllerPlugin::Init" << std::endl;
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnStringMsg(ConstGzStringPtr &_msg)
{
  gzmsg << "Command received " << _msg->data() << std::endl;
  this->command = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalControllerPlugin::OnUpdate()
{
  if (!this->pitchJoint)
    return;

  double angle = this->pitchJoint->GetAngle(0).Radian();

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
    double error = angle - this->command;
    double force = this->pitchPid.Update(error, dt);
    this->pitchJoint->SetForce(0, force);
    this->lastUpdateTime = time;
  }

  static int i =1000;
  if (++i>100)
  {
    i = 0;
    std::stringstream ss;
    ss << angle;
    gazebo::msgs::GzString m;
    m.set_data(ss.str());
    this->pub->Publish(m);
  }
}


