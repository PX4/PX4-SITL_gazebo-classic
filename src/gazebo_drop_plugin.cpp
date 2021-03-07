/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Drop Plugin
 *
 * This plugin simulates dropping a payload
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "gazebo_drop_plugin.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(DropPlugin)

DropPlugin::DropPlugin() : ModelPlugin()
{
}

DropPlugin::~DropPlugin()
{
  update_connection_->~Connection();
}

void DropPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
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
  getSdfParam<std::string>(sdf, "modelName", model_name_, model_name_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&DropPlugin::OnUpdate, this, _1));

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  trigger_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + trigger_sub_topic_, &DropPlugin::VelocityCallback, this);
}

void DropPlugin::OnUpdate(const common::UpdateInfo&){

  physics::ModelPtr package_model_ptr = GetModelPtr(model_name_);
  //Trigger parachute if flight termination
  if(ref_motor_rot_vel_ > release_rot_vel_ ) LoadPackage();
  if(!package_spawned && package_model_ptr) {

#if GAZEBO_MAJOR_VERSION >= 9
    const ignition::math::Pose3d vehicle_pose = model_->WorldPose();
#else
    const ignition::math::Pose3d vehicle_pose = ignitionFromGazeboMath(model_->GetWorldPose()); //TODO(burrimi): Check tf.
#endif
    package_model_ptr->SetWorldPose(ignition::math::Pose3d(vehicle_pose.Pos().X(), vehicle_pose.Pos().Y(), vehicle_pose.Pos().Z()-0.5, 0, 0, 0));        // or use uavPose.ros.GetYaw() ?

    package_spawned = true;
  }
}

void DropPlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else ref_motor_rot_vel_ = std::min(static_cast<double>(rot_velocities->motor_speed(motor_number_)), static_cast<double>(max_rot_velocity_));
}

void DropPlugin::LoadPackage(){
  // Don't create duplicate the payload
  physics::ModelPtr parachute_model = GetModelPtr(model_name_);
  if(parachute_model) return;

  // Insert parachute model
  std::string model_uri = "model://" + model_name_;
  world_->InsertModelFile(model_uri);

  msgs::Int request;
  request.set_data(0);
  
}

physics::ModelPtr DropPlugin::GetModelPtr(std::string model_name){
  physics::ModelPtr model;

  #if GAZEBO_MAJOR_VERSION >= 9
    model = world_->ModelByName(model_name);
  #else
    model = world_->GetModel(model_name);
  #endif
  return model;

}
} // namespace gazebo