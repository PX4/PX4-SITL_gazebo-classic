/****************************************************************************
 *
 *   Copyright (c) 2024 Jaeyoung Lim, Autonomous Systems Lab, ETH Zurich.
 *                   All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
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


#include "gazebo_coldgas_thruster_plugin.h"
#include <ignition/math.hh>

namespace gazebo {

GazeboColdGasThrusterPlugin::~GazeboColdGasThrusterPlugin() {
  updateConnection_->~Connection();
}

void GazeboColdGasThrusterPlugin::InitializeParams() {}

void GazeboColdGasThrusterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_ << "\".");


  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<double>(_sdf, "pwmFrequency", pwm_frequency_, 10.0);
  getSdfParam<double>(_sdf, "maxThrust", max_thrust_, 1.4);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboColdGasThrusterPlugin::OnUpdate, this, _1));

  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + command_sub_topic_, &GazeboColdGasThrusterPlugin::VelocityCallback, this);
}

// This gets called by the world update start event.
void GazeboColdGasThrusterPlugin::OnUpdate(const common::UpdateInfo& _info) {
  ///TODO: Compute duty cycle time
  if (sampling_time_ >= 1.0/pwm_frequency_) {
    cycle_start_time_ = _info.simTime.Double();
  }
  sampling_time_ = _info.simTime.Double() - cycle_start_time_;
  double period = sampling_time_ *pwm_frequency_;
  ///TODO: Get duty cycle from control input
  UpdateForcesAndMoments(ref_duty_cycle_, period);
}

void GazeboColdGasThrusterPlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout  << "You tried to access index " << motor_number_
      << " of the MotorSpeed message array which is of size " << rot_velocities->motor_speed_size() << "." << std::endl;
  } else ref_duty_cycle_ = std::min(static_cast<double>(rot_velocities->motor_speed(motor_number_)), 1.0);
}

void GazeboColdGasThrusterPlugin::UpdateForcesAndMoments(const double &duty_cycle, const double &period) {
  // Thrust is only generated uring the duty cycle
  double force = duty_cycle > period ? max_thrust_ : 0.0;
  // std::cout << "duty cycle: " << duty_cycle << " period: " << period << " thrust: " << force << std::endl;
  link_->AddRelativeForce(ignition::math::Vector3d(0, 0, force));
}

GZ_REGISTER_MODEL_PLUGIN(GazeboColdGasThrusterPlugin);
}
