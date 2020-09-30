/*
 * Copyright 2015  Aurelien Roy
 * 
 * This file is a modified version of github.com/AurelienRoy/ardupilot_sitl_gazebo_plugin.git
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
 */

 /**
  * @brief Parachute Plugin
  *
  * This plugin simulates parachute deployment
  *
  * @author Aurelien Roy  <aurroy@hotmail.com>
  */

#include "gazebo_parachute_plugin.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(ParachutePlugin)

ParachutePlugin::ParachutePlugin() : ModelPlugin()
{
}

ParachutePlugin::~ParachutePlugin()
{
  update_connection_->~Connection();
}

void ParachutePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_parachute_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("motorNumber"))
    motor_number_ = sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_parachute_plugin] Please specify a motorNumber.\n";

  getSdfParam<std::string>(sdf, "commandSubTopic", trigger_sub_topic_, trigger_sub_topic_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&ParachutePlugin::OnUpdate, this, _1));

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  trigger_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + trigger_sub_topic_, &ParachutePlugin::VelocityCallback, this);
}

void ParachutePlugin::OnUpdate(const common::UpdateInfo&){

  physics::ModelPtr parachute_model = GetModelPtr("parachute_small");
  //Trigger parachute if flight termination
  if(ref_motor_rot_vel_ <= terminate_rot_vel_ ) LoadParachute();

  if(!attached_parachute_ && parachute_model){
    AttachParachute(parachute_model); //Attach parachute to model
    attached_parachute_ = true;
  }
}

void ParachutePlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else ref_motor_rot_vel_ = std::min(static_cast<double>(rot_velocities->motor_speed(motor_number_)), static_cast<double>(max_rot_velocity_));
}

void ParachutePlugin::LoadParachute(){
  // Don't create duplicate paracutes
  physics::ModelPtr parachute_model = GetModelPtr("parachute_small");
  if(parachute_model) return;

  // Insert parachute model
  world_->InsertModelFile("model://parachute_small");

  msgs::Int request;
  request.set_data(0);
  
}

physics::ModelPtr ParachutePlugin::GetModelPtr(std::string model_name){
  physics::ModelPtr model;

  #if GAZEBO_MAJOR_VERSION >= 9
    model = world_->ModelByName(model_name);
  #else
    model = world_->GetModel(model_name);
  #endif
  return model;

}

void ParachutePlugin::AttachParachute(physics::ModelPtr &parachute_model){

#if GAZEBO_MAJOR_VERSION >= 9
  const ignition::math::Pose3d vehicle_pose = model_->WorldPose();
#else
  const ignition::math::Pose3d vehicle_pose = ignitionFromGazeboMath(model_->GetWorldPose()); //TODO(burrimi): Check tf.
#endif
  parachute_model->SetWorldPose(ignition::math::Pose3d(vehicle_pose.Pos().X(), vehicle_pose.Pos().Y(), vehicle_pose.Pos().Z()+0.3, 0, 0, 0));        // or use uavPose.ros.GetYaw() ?

#if GAZEBO_MAJOR_VERSION >= 9
  gazebo::physics::JointPtr parachute_joint = world_->Physics()->CreateJoint("fixed", model_);
#else
  gazebo::physics::JointPtr parachute_joint = world_->GetPhysicsEngine()->CreateJoint("fixed", model_);
#endif

  parachute_joint->SetName("parachute_joint");
  
  // Attach parachute to base_link
  gazebo::physics::LinkPtr base_link = model_->GetLink("base_link");
  gazebo::physics::LinkPtr parachute_link = parachute_model->GetLink("chute");
  parachute_joint->Attach(base_link, parachute_link);

  // load the joint, and set up its anchor point
  parachute_joint->Load(base_link, parachute_link, ignition::math::Pose3d(0, 0, 0.3, 0, 0, 0));
}
} // namespace gazebo