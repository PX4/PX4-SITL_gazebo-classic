/*
 * Copyright 2016 Austin Buchan, Nils Rottmann TUHH Hamburg, Germany
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * This scripts simulates underwater froces and torques for the hippocampus model. The HippoCampus is an autonomous
 * underwater vehicle (AUV) designed by the Technical University Hamburg-Harburg (TUHH).
 * https://www.tuhh.de/mum/forschung/forschungsgebiete-und-projekte/flow-field-estimation-with-a-swarm-of-auvs.html
 */


#include "gazebo_uuv_plugin.h"
#include <ignition/math.hh>
#include <iostream>
#include <iomanip>

namespace gazebo {

GazeboUUVPlugin::~GazeboUUVPlugin() {
  updateConnection_->~Connection();
}

// Load necessary data from the .sdf file
void GazeboUUVPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  namespace_.clear();

  namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  joint_0_name_ = _sdf->GetElement("joint_0_name")->Get<std::string>();
  joint_0_ = model_->GetJoint(joint_0_name_);
  joint_1_name_ = _sdf->GetElement("joint_1_name")->Get<std::string>();
  joint_1_ = model_->GetJoint(joint_1_name_);
  joint_2_name_ = _sdf->GetElement("joint_2_name")->Get<std::string>();
  joint_2_ = model_->GetJoint(joint_2_name_);
  joint_3_name_ = _sdf->GetElement("joint_3_name")->Get<std::string>();
  joint_3_ = model_->GetJoint(joint_3_name_);

  link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  link_ = model_->GetLink(link_name_);

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);

  // get force and torque parameters for force and torque calculations of the rotors from motor_speed
  getSdfParam<double>(_sdf, "motorForceConstant", motor_force_constant_, motor_force_constant_);
  getSdfParam<double>(_sdf, "motorTorqueConstant", motor_torque_constant_, motor_torque_constant_);

  // parameters for added mass and damping
  ignition::math::Vector3d added_mass_linear(0,0,0);
  getSdfParam<ignition::math::Vector3d>(
    _sdf, "addedMassLinear", added_mass_linear, added_mass_linear);
  X_udot_ = added_mass_linear[0];
  Y_vdot_ = added_mass_linear[1];
  Z_wdot_ = added_mass_linear[2];

  ignition::math::Vector3d added_mass_angular(0,0,0);
  getSdfParam<ignition::math::Vector3d>(
    _sdf, "addedMassAngular", added_mass_angular, added_mass_angular);
  K_pdot_ = added_mass_angular[0];
  M_qdot_ = added_mass_angular[1];
  N_rdot_ = added_mass_angular[2];

  ignition::math::Vector3d damping_linear(0,0,0);
  getSdfParam<ignition::math::Vector3d>(
    _sdf, "dampingLinear", damping_linear, damping_linear);
  X_u_ = damping_linear[0];
  Y_v_ = damping_linear[1];
  Z_w_ = damping_linear[2];

  ignition::math::Vector3d damping_angular(0,0,0);
  getSdfParam<ignition::math::Vector3d>(
    _sdf, "dampingAngular", damping_angular, damping_angular);
  K_p_ = damping_angular[0];
  M_q_ = damping_angular[1];
  N_r_ = damping_angular[2];

  //Get rotor turning directions
  std::string direction_rotor_0 = _sdf->GetElement("direction_rotor_0")->Get<std::string>();
  if (direction_rotor_0 == "cw") direction_[0] = turning_direction::CW;
  else if (direction_rotor_0 == "ccw") direction_[0] = turning_direction::CCW;
  else gzerr << "Please only use 'cw' or 'ccw' as turningDirection.\n";

  std::string direction_rotor_1 = _sdf->GetElement("direction_rotor_1")->Get<std::string>();
  if (direction_rotor_1 == "cw") direction_[1] = turning_direction::CW;
  else if (direction_rotor_1 == "ccw") direction_[1] = turning_direction::CCW;
  else gzerr << "Please only use 'cw' or 'ccw' as turningDirection.\n";

  std::string direction_rotor_2 = _sdf->GetElement("direction_rotor_2")->Get<std::string>();
  if (direction_rotor_2 == "cw") direction_[2] = turning_direction::CW;
  else if (direction_rotor_2 == "ccw") direction_[2] = turning_direction::CCW;
  else gzerr << "Please only use 'cw' or 'ccw' as turningDirection.\n";

  std::string direction_rotor_3 = _sdf->GetElement("direction_rotor_3")->Get<std::string>();
  if (direction_rotor_3 == "cw") direction_[3] = turning_direction::CW;
  else if (direction_rotor_3 == "ccw") direction_[3] = turning_direction::CCW;
  else gzerr << "Please only use 'cw' or 'ccw' as turningDirection.\n";

  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboUUVPlugin::OnUpdate, this, _1));
  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + command_sub_topic_, &GazeboUUVPlugin::VelocityCallback, this);
}

// function to get the motor speed
void GazeboUUVPlugin::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  for (int i = 0; i < 4; i++) {
    command_[i] = static_cast<double>(rot_velocities->motor_speed(i));
  }
  /**std::cout << std::setprecision(4)<<"UUV Command Callback:"
    << command_[0] << ","
    << command_[1] << ","
    << command_[2] << ","
    << command_[3] << "\n";*/

}

// Update function, this runs in every circle
void GazeboUUVPlugin::OnUpdate(const common::UpdateInfo& _info) {
  double now = _info.simTime.Double();
  time_delta_ =  now - last_time_;
  last_time_ = now;
  time_ = time_ + time_delta_;

  std::cout << std::setprecision(4)<<"UUV Command Callback:"
      << command_[0] << ","
      << command_[1] << ","
      << command_[2] << ","
      << command_[3] << "\n";

   /**
  //std::cout << "UUV Update at " << now << ", delta " << time_delta_ << "\n";

  double forces[4];
  double torques[4];

  double real_motor_velocity[4];

  command_[0] = joint_0_->GetVelocity(0);
  command_[1] = joint_1_->GetVelocity(0);
  command_[2] = joint_2_->GetVelocity(0);
  command_[3] = joint_3_->GetVelocity(0);

  //std::cout << std::showpos << std::fixed << std::setprecision(4) << "Velocities: " << command_[0] << ", " << command_[1] << ", " << command_[2] << ", " << command_[3] << "\n";

  for(int i = 0; i < 4; i++){
      //real_motor_velocity[i] = command_[i] * rotor_velocity_slowdown_sim_;
      real_motor_velocity[i] = command_[i];
      //real_motor_velocity[i] = command_[i]*500* rotor_velocity_slowdown_sim_;
  }
    //std::cout << std::showpos << std::fixed << std::setprecision(4) << "Real Velocities: " << real_motor_velocity[0] << ", " << real_motor_velocity[1] << ", " << real_motor_velocity[2] << ", " << real_motor_velocity[3] << "\n";

    for(int i = 0; i < 4; i++) {
      // Currently a rotor index hack to get over IMU link being first, since rotor_links_[0] would be the IMU
      //ignition::math::Vector3d rotor_force(direction_[i] * motor_force_constant_ * real_motor_velocity[i] * std::abs(real_motor_velocity[i]), 0, 0);
      ignition::math::Vector3d rotor_force(motor_force_constant_ * real_motor_velocity[i] * std::abs(real_motor_velocity[i]), 0, 0);
      rotor_links_[i+1]->AddRelativeForce(rotor_force);
      forces[i] = rotor_force[0];

      // CCW 1, CW 2, CCW 3 and CW 4. Apply drag torque
      // directly to main body X axis
      //ignition::math::Vector3d rotor_torque(-direction_[i] * motor_torque_constant_ * real_motor_velocity[i] * std::abs(real_motor_velocity[i]), 0, 0);
      ignition::math::Vector3d rotor_torque(-direction_[i] * motor_torque_constant_ * forces[i], 0, 0);

      link_->AddRelativeTorque(rotor_torque);
      //rotor_links_[i+1]->AddRelativeTorque(rotor_torque);
      torques[i] = rotor_torque[0];
    }
    //std::cout << std::showpos << std::fixed << std::setprecision(4) << "Force:" << forces[0] << "\n";

  // Calculate and apply body Coriolis and Drag forces and torques
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d linear_velocity = link_->RelativeLinearVel();
#else
  ignition::math::Vector3d linear_velocity = ignitionFromGazeboMath(link_->GetRelativeLinearVel());
#endif
  double u = linear_velocity[0];
  double v = linear_velocity[1];
  double w = linear_velocity[2];

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d angular_velocity = link_->RelativeAngularVel();
#else
  ignition::math::Vector3d angular_velocity = ignitionFromGazeboMath(link_->GetRelativeAngularVel());
#endif
  double p = angular_velocity[0];
  double q = angular_velocity[1];
  double r = angular_velocity[2];

  //std::cout << "Vels:" << linear_velocity << ":" << angular_velocity << "\n";

  // Linear Damping Matrix, with minus already multiplied
  ignition::math::Matrix3d D_FL(
    -X_u_, 0, 0,
    0, -Y_v_, 0,
    0, 0, -Z_w_
  );

  // Angular Damping Matrix, with minus already multiplied
  ignition::math::Matrix3d D_FA(
    -K_p_, 0, 0,
    0, -M_q_, 0,
    0, 0, -N_r_
  );

  ignition::math::Vector3d damping_force =
        D_FL*linear_velocity;
  ignition::math::Vector3d damping_torque =
        D_FA*angular_velocity;

  // Corriolis Forces and Torques
  // upper right and bottom left matrix, with minus already multiplied
  ignition::math::Matrix3d C_AD_FA(
    0, Z_wdot_ * w, -Y_vdot_ * v,
    -Z_wdot_ * w, 0, X_udot_ * u,
    Y_vdot_ * v, -X_udot_ * u, 0
  );

  // Torques from angular velocity, with minus already multiplied
  ignition::math::Matrix3d C_AD_TA(
    0, N_rdot_ * r,  -M_qdot_ * q,
    -N_rdot_ * r, 0, K_pdot_ * p,
    M_qdot_ * q, -K_pdot_ * p, 0
  );

  ignition::math::Vector3d coriolis_force =
    C_AD_FA*angular_velocity;
  ignition::math::Vector3d coriolis_torque =
    (C_AD_FA*linear_velocity) + (C_AD_TA*angular_velocity);

  //std::cout << C_AD_FA << "\n";
  //std::cout << "Linear:" << coriolis_force << "\n";
  //std::cout << "Angular:" << angular_velocity << "\n";

  link_->AddRelativeForce(damping_force + coriolis_force);
  link_->AddRelativeTorque(damping_torque + coriolis_torque);
 */
}

GZ_REGISTER_MODEL_PLUGIN(GazeboUUVPlugin)
}
