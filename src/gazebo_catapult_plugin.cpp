/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @brief Catapult Plugin
 *
 * This plugin simulates a catapult / handlaunch for fixed wing vehicles
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#include "gazebo_catapult_plugin.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(CatapultPlugin)

CatapultPlugin::CatapultPlugin() : ModelPlugin()
{
}

CatapultPlugin::~CatapultPlugin()
{
  _updateConnection->~Connection();
}

void CatapultPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_catapult_plugin] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("link_name")) {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    std::string linkName = elem->Get<std::string>();
    this->link_ = this->model_->GetLink(linkName);

    if (!this->link_) {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The Catapult plugin will not be able to launch the vehicle\n";
    }
  } else {
    gzerr << "[gazebo_catapult_plugin] link_name needs to be provided";
  }

  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_catapult_plugin] Please specify a motorNumber.\n";

  getSdfParam<std::string>(_sdf, "commandSubTopic", trigger_sub_topic_, trigger_sub_topic_);
  getSdfParam<double>(_sdf, "force", force_magnitude_, force_magnitude_);
  getSdfParam<std::string>(_sdf, "direction", trigger_sub_topic_, trigger_sub_topic_);
  getSdfParam<double>(_sdf, "duration", launch_duration_, launch_duration_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&CatapultPlugin::OnUpdate, this, _1));

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  trigger_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + trigger_sub_topic_, &CatapultPlugin::VelocityCallback, this);
}

void CatapultPlugin::OnUpdate(const common::UpdateInfo&){

    //Launch vehicle if the vehilce is armed
    if(ref_motor_rot_vel_ > arm_rot_vel_ && launch_status_ != VEHICLE_LAUNCHED ) {
      if(launch_status_ == VEHICLE_STANDBY) {
      #if GAZEBO_MAJOR_VERSION >= 9
        trigger_time_ = world_->SimTime();
      #else
        trigger_time_ = world_->GetSimTime();
      #endif
        launch_status_ = VEHICLE_INLAUNCH;
        std::cout << "[gazebo_catapult_plugin] Catapult armed " << std::endl;
      
      } else { // launch_status = VEHICLE_INLAUNCH
        //Define launch direction
        ignition::math::Vector3d direction(1.0, 0.0, 2.0);
        direction.Normalize();

        //Apply force to the vehicle
        ignition::math::Vector3d force = force_magnitude_ * direction;
        this->link_->AddForce(force);     
        #if GAZEBO_MAJOR_VERSION >= 9
          common::Time curr_time = world_->SimTime();
        #else
          common::Time curr_time = world_->GetSimTime();
        #endif
        if (curr_time - trigger_time_  > launch_duration_) launch_status_ = VEHICLE_LAUNCHED;
      }
    }
}

void CatapultPlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else ref_motor_rot_vel_ = std::min(static_cast<double>(rot_velocities->motor_speed(motor_number_)), static_cast<double>(max_rot_velocity_));
}

} // namespace gazebo