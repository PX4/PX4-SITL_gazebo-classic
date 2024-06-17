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

#include "gazebo_monocopter_dynamics_plugin.h"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(MonocopterDynamicsPlugin)

MonocopterDynamicsPlugin::MonocopterDynamicsPlugin() : ModelPlugin()
{
}

MonocopterDynamicsPlugin::~MonocopterDynamicsPlugin()
{
  _updateConnection->~Connection();
}

void MonocopterDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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
    gzerr << "[gazebo_monocopter_dynamics_plugin] link_name needs to be provided";
  }

  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_catapult_plugin] Please specify a motorNumber.\n";

  getSdfParam<double>(_sdf, "force", force_magnitude_, force_magnitude_);
  getSdfParam<double>(_sdf, "duration", launch_duration_, launch_duration_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MonocopterDynamicsPlugin::OnUpdate, this, _1));

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

}

void MonocopterDynamicsPlugin::OnUpdate(const common::UpdateInfo&){

  // Compute forces
  ignition::math::Vector3d total_force;
  ///TODO: Compute motor thrust
  ignition::math::Vector3d force_motor;
  ///TODO: Compute force induced by aerodynamics
  ignition::math::Vector3d force_aerodynamic;
  total_force = force_motor + force_aerodynamic;
  this->link_->AddRelativeForce(total_force);

  /// Compute moments
  ignition::math::Vector3d total_torque;
  ///TODO: Compute coriolis force components
  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d linear_velocity = link_->RelativeLinearVel();
    ignition::math::Vector3d angular_velocity = link_->RelativeAngularVel();
  #else
    ignition::math::Vector3d linear_velocity = ignitionFromGazeboMath(link_->GetRelativeLinearVel());
    ignition::math::Vector3d angular_velocity = ignitionFromGazeboMath(link_->GetRelativeAngularVel());
  #endif
  ///TODO: Get moment of inertia
  ignition::math::Matrix3d I_;

  ignition::math::Vector3d torque_coriolis = angular_velocity.Cross(I_ * angular_velocity);

  ///TODO: Compute motor torque
  ignition::math::Vector3d torque_motor;
  ///TODO: Compute moment induced by aerodynamics
  ignition::math::Vector3d torque_aerodynamic;

  total_torque = torque_coriolis + torque_motor + torque_aerodynamic;
  this->link_->AddRelativeTorque(total_torque);
}

void MonocopterDynamicsPlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else ref_motor_rot_vel_ = std::min(static_cast<double>(rot_velocities->motor_speed(motor_number_)), static_cast<double>(max_rot_velocity_));
}

} // namespace gazebo