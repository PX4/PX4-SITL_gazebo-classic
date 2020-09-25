/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @brief LTA Dynamics Plugin
 *
 * This plugin simulates the aerodynamics of an airship.
 * The equations are based on the paper:
 * Airship Dynamics Modeling: A Literature Review
 * Authors: Yuwen Li, Meyer Nahon, Inna Sharf
 *
 * @author Anton Erasmus <anton@flycloudline.com>
 */

#include "gazebo_airship_dynamics_plugin.h"
#include <ignition/math.hh>
#include <iostream>

#define EPS 1e-4

namespace gazebo {

GazeboLTADynamicsPlugin::~GazeboLTADynamicsPlugin() {
  updateConnection_->~Connection();
}

void GazeboLTADynamicsPlugin::Publish() {}

void GazeboLTADynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Wind
  getSdfParam<std::string>(_sdf, "windSubTopic", wind_sub_topic_, wind_sub_topic_);

  // Link
  if(_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a linkName.\n";
  }
  link_ = model_->GetLink(link_name_);

  // Volume
  if(_sdf->HasElement("hullVolume")) {
    param_hull_volume_ = _sdf->GetElement("hullVolume")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a hullVolume.\n";
  }

  // Air Density
  if(_sdf->HasElement("airDensity")) {
    param_air_density_ = _sdf->GetElement("airDensity")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a airDensity.\n";
  }

  // Added mass terms
  if(_sdf->HasElement("m11")) {
    param_m11_ = _sdf->GetElement("m11")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a m11.\n";
  }
  if(_sdf->HasElement("m22")) {
    param_m22_ = _sdf->GetElement("m22")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a m22.\n";
  }
  if(_sdf->HasElement("m26")) {
    param_m26_ = _sdf->GetElement("m26")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a m26.\n";
  }
  if(_sdf->HasElement("m33")) {
    param_m33_ = _sdf->GetElement("m33")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a m33.\n";
  }
  if(_sdf->HasElement("m35")) {
    param_m35_ = _sdf->GetElement("m35")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a m35.\n";
  }
  if(_sdf->HasElement("m44")) {
    param_m44_ = _sdf->GetElement("m44")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a m44.\n";
  }
  if(_sdf->HasElement("m53")) {
    param_m53_ = _sdf->GetElement("m53")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a m53.\n";
  }
  if(_sdf->HasElement("m55")) {
    param_m55_ = _sdf->GetElement("m55")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a m55.\n";
  }
  if(_sdf->HasElement("m62")) {
    param_m62_ = _sdf->GetElement("m62")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a m62.\n";
  }
  if(_sdf->HasElement("m66")) {
    param_m66_ = _sdf->GetElement("m66")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a m66.\n";
  }

  // Properties
  if(_sdf->HasElement("distCOV")) {
    param_cov_ = _sdf->GetElement("distCOV")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a distCOV.\n";
  }
  if(_sdf->HasElement("distPotentialFlow")) {
    param_eps_v_ = _sdf->GetElement("distPotentialFlow")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a distPotentialFlow.\n";
  }
  if(_sdf->HasElement("distFinCenter")) {
    param_dist_fin_x_ = _sdf->GetElement("distFinCenter")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a distFinCenter.\n";
  }
  if(_sdf->HasElement("distQuarterChord")) {
    param_dist_fin_quarter_chord_ = _sdf->GetElement("distQuarterChord")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a distQuarterChord.\n";
  }

  // Aerodynamic terms
  if(_sdf->HasElement("forceHullInviscidFlowCoeff")) {
    param_f_hif_coeff_ = _sdf->GetElement("forceHullInviscidFlowCoeff")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a forceHullInviscidFlowCoeff.\n";
  }
  if(_sdf->HasElement("forceHullViscousFlowCoeff")) {
    param_f_hvf_coeff_ = _sdf->GetElement("forceHullViscousFlowCoeff")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a forceHullViscousFlowCoeff.\n";
  }
  if(_sdf->HasElement("momentHullInviscidFlowCoeff")) {
    param_m_hif_coeff_ = _sdf->GetElement("momentHullInviscidFlowCoeff")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a momentHullInviscidFlowCoeff.\n";
  }
  if(_sdf->HasElement("momentHullViscousFlowCoeff")) {
    param_m_hvf_coeff_ = _sdf->GetElement("momentHullViscousFlowCoeff")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a momentHullViscousFlowCoeff.\n";
  }
  if(_sdf->HasElement("finNormalForceCoeff")) {
    param_f_fnf_coeff_ = _sdf->GetElement("finNormalForceCoeff")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a finNormalForceCoeff.\n";
  }
  if(_sdf->HasElement("finStallAngle")) {
    param_fin_stall_angle_ = _sdf->GetElement("finStallAngle")->Get<double>() * M_PI / 180.0f;
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a finStallAngle.\n";
  }
  if(_sdf->HasElement("axialDragCoeff")) {
    param_axial_drag_coeff_ = _sdf->GetElement("axialDragCoeff")->Get<double>();
  } else {
    gzerr << "[gazebo_lta_dynamics_plugin] Please specify a axialDragCoeff.\n";
  }

  // Set initial wind as zero
  wind_.X() = 0;
  wind_.Y() = 0;
  wind_.Z() = 0;

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboLTADynamicsPlugin::OnUpdate, this, _1));

  // Subscribe to wind topic.
  wind_sub_ = node_handle_->Subscribe<physics_msgs::msgs::Wind>("~/" + model_->GetName() + wind_sub_topic_, &GazeboLTADynamicsPlugin::WindVelocityCallback, this);
}

// This gets called by the world update start event.
void GazeboLTADynamicsPlugin::OnUpdate(const common::UpdateInfo& _info) {
  UpdateForcesAndMoments();
}

void GazeboLTADynamicsPlugin::WindVelocityCallback(WindPtr &wind) {
  // Get world wind velocity.
  ignition::math::Vector3d wind_world = ignition::math::Vector3d(wind->velocity().x(), wind->velocity().y(), wind->velocity().z());

  // Rotate to body frame
  #if GAZEBO_MAJOR_VERSION >= 9
    wind_ = link_->WorldPose().Rot().Inverse().RotateVector(wind_world);
  #else
    wind_ = ignitionFromGazeboMath(link_->GetWorldPose()).Rot().Inverse().RotateVector(wind_world);
  #endif
}

double GazeboLTADynamicsPlugin::Sign(double val) {
  if(val >= 0) {
    return 1;
  } else {
    return -1;
  }
}

ignition::math::Vector3d GazeboLTADynamicsPlugin::LocalVelocity(ignition::math::Vector3d lin_vel, ignition::math::Vector3d ang_vel, ignition::math::Vector3d dist) {
  return lin_vel + ang_vel.Cross(ignition::math::Vector3d(param_cov_ - dist[0], dist[1], dist[2]));
}

double GazeboLTADynamicsPlugin::DynamicPressure(ignition::math::Vector3d vec) {
  ignition::math::Vector3d airspeed = vec - wind_;
  return 0.5f * param_air_density_ * (airspeed[0]*airspeed[0] + airspeed[1]*airspeed[1] + airspeed[2]*airspeed[2]);
}

void GazeboLTADynamicsPlugin::UpdateForcesAndMoments() {
  // Obtain gravity and the link's linear and angular velocity and acceleration
  #if GAZEBO_MAJOR_VERSION < 5
    gzerr << "[gazebo_lta_dynamics_plugin] Unsupported Gazebo version";
  #elif GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d gravity_w = world_->Gravity();
    ignition::math::Vector3d linear_acc = link_->RelativeLinearAccel();
    ignition::math::Vector3d linear_vel = link_->RelativeLinearVel();
    ignition::math::Vector3d angular_acc = link_->RelativeAngularAccel();
    ignition::math::Vector3d angular_vel = link_->RelativeAngularVel();
  #else
    ignition::math::Vector3d gravity_w = ignitionFromGazeboMath(world_->GetPhysicsEngine()->GetGravity());
    ignition::math::Vector3d linear_acc = ignitionFromGazeboMath(link_->GetRelativeLinearAccel());
    ignition::math::Vector3d linear_vel = ignitionFromGazeboMath(link_->GetRelativeLinearVel());
    ignition::math::Vector3d angular_acc = ignitionFromGazeboMath(link_->GetRelativeAngularAccel());
    ignition::math::Vector3d angular_vel = ignitionFromGazeboMath(link_->GetRelativeAngularVel());
  #endif

  //Calculate buoyancy force
  ignition::math::Vector3d bouyancy_force_world = - param_air_density_ * param_hull_volume_ * gravity_w;
  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d bouyancy_force_body = link_->WorldPose().Rot().RotateVectorReverse(bouyancy_force_world);
  #else
    ignition::math::Vector3d bouyancy_force_body = ignitionFromGazeboMath(link_->GetWorldPose()).Rot().RotateVectorReverse(bouyancy_force_world);
  #endif

  // Construct added mass matrices
  ignition::math::Matrix3d M11 = ignition::math::Matrix3d(param_m11_, 0, 0, 0, param_m22_, 0, 0, 0, param_m33_);
  ignition::math::Matrix3d M12 = ignition::math::Matrix3d(0, 0, 0, 0, 0, param_m26_, 0, param_m35_, 0);
  ignition::math::Matrix3d M21 = ignition::math::Matrix3d(0, 0, 0, 0, 0, param_m53_, 0, param_m62_, 0);
  ignition::math::Matrix3d M22 = ignition::math::Matrix3d(param_m44_, 0, 0, 0, param_m55_, 0, 0, 0, param_m66_);

  // Calculate added mass forces and moments
  ignition::math::Vector3d am_comp1 = M11 * linear_acc + M12 * angular_acc;
  ignition::math::Vector3d am_comp2 = M21 * linear_acc + M22 * angular_acc;
  ignition::math::Vector3d munk_comp1 = M11 * linear_vel + M12 * angular_vel;
  ignition::math::Vector3d munk_comp2 = M21 * linear_vel + M22 * angular_vel;
  ignition::math::Vector3d added_mass_force = -am_comp1 - (angular_vel.Cross(munk_comp1));
  ignition::math::Vector3d added_mass_moment = -am_comp2 - (linear_vel.Cross(munk_comp1) + angular_vel.Cross(munk_comp2));

  // Calculate viscous forces
  ignition::math::Vector3d vel_eps_v = LocalVelocity(linear_vel, angular_vel, ignition::math::Vector3d(param_eps_v_, 0, 0));
  double q0_eps_v = DynamicPressure(vel_eps_v);
  double gamma_eps_v = 0.0f;
  if(std::abs(vel_eps_v[0]) > EPS) {
    gamma_eps_v = atan2(sqrt(vel_eps_v[1]*vel_eps_v[1] + vel_eps_v[2]*vel_eps_v[2]), vel_eps_v[0]);
  }
  double force_visc_mag = q0_eps_v*(-param_f_hif_coeff_*sin(2*gamma_eps_v) + param_f_hvf_coeff_*sin(gamma_eps_v)*sin(gamma_eps_v));
  double moment_visc_mag = q0_eps_v*(-param_m_hif_coeff_*sin(2*gamma_eps_v) + param_m_hvf_coeff_*sin(gamma_eps_v)*sin(gamma_eps_v));
  double visc_normal_mag = sqrt(vel_eps_v[1]*vel_eps_v[1] + vel_eps_v[2]*vel_eps_v[2]);
  double visc_normal_y = 0.0f;
  double visc_normal_z = 0.0f;
  if(visc_normal_mag > EPS) {
    visc_normal_y = vel_eps_v[1]/visc_normal_mag;
    visc_normal_z = vel_eps_v[2]/visc_normal_mag;
  }
  ignition::math::Vector3d force_visc = -force_visc_mag*ignition::math::Vector3d(0, visc_normal_y, visc_normal_z);
  ignition::math::Vector3d moment_visc = moment_visc_mag*ignition::math::Vector3d(0, visc_normal_z, -visc_normal_y);

  // Calculate fin normal forces
  double q0 = DynamicPressure(linear_vel);
  ignition::math::Vector3d wind_vel_y = ignition::math::Vector3d(0, wind_[1], 0);
  ignition::math::Vector3d wind_vel_z = ignition::math::Vector3d(0, 0, wind_[2]);
  ignition::math::Vector3d fin1_vel = LocalVelocity(linear_vel, angular_vel, ignition::math::Vector3d(param_dist_fin_x_, 0, -param_dist_fin_quarter_chord_)) + wind_vel_y;
  ignition::math::Vector3d fin2_vel = LocalVelocity(linear_vel, angular_vel, ignition::math::Vector3d(param_dist_fin_x_, 0, param_dist_fin_quarter_chord_)) + wind_vel_y;
  ignition::math::Vector3d fin3_vel = LocalVelocity(linear_vel, angular_vel, ignition::math::Vector3d(param_dist_fin_x_, -param_dist_fin_quarter_chord_, 0)) + wind_vel_z;
  ignition::math::Vector3d fin4_vel = LocalVelocity(linear_vel, angular_vel, ignition::math::Vector3d(param_dist_fin_x_, param_dist_fin_quarter_chord_, 0)) + wind_vel_z;
  double angle_fin1 = 0.0f;
  if(std::abs(fin1_vel[0]) > EPS) {
    angle_fin1 = atan2(fin1_vel[1], fin1_vel[0]);
  }
  if(std::abs(angle_fin1) > param_fin_stall_angle_) {
    angle_fin1 = Sign(angle_fin1)*param_fin_stall_angle_;
  }
  double angle_fin2 = 0.0f;
  if(std::abs(fin2_vel[0]) > EPS) {
    angle_fin2 = atan2(fin2_vel[1], fin2_vel[0]);
  }
  if(std::abs(angle_fin2) > param_fin_stall_angle_) {
    angle_fin2 = Sign(angle_fin2)*param_fin_stall_angle_;
  }
  double angle_fin3 = 0.0f;
  if(std::abs(fin3_vel[0]) > EPS) {
    angle_fin3 = atan2(fin3_vel[2], fin3_vel[0]);
  }
  if(std::abs(angle_fin3) > param_fin_stall_angle_) {
    angle_fin3 = Sign(angle_fin3)*param_fin_stall_angle_;
  }
  double angle_fin4 = 0.0f;
  if(std::abs(fin4_vel[0]) > EPS) {
    angle_fin4 = atan2(fin4_vel[2], fin4_vel[0]);
  }
  if(std::abs(angle_fin4) > param_fin_stall_angle_) {
    angle_fin4 = Sign(angle_fin4)*param_fin_stall_angle_;
  }
  double h_angle = angle_fin3 + angle_fin4;
  double v_angle = angle_fin1 + angle_fin2;
  ignition::math::Vector3d force_fins = -q0*param_f_fnf_coeff_*ignition::math::Vector3d(0, v_angle, h_angle);
  ignition::math::Vector3d moment_fins = q0*param_f_fnf_coeff_*ignition::math::Vector3d(param_dist_fin_quarter_chord_*(angle_fin1 + angle_fin4 - angle_fin2 - angle_fin3), param_dist_fin_x_*h_angle, -param_dist_fin_x_*v_angle);

  // Axial drag
  double angle_of_attack = 0;
  if(std::abs(linear_vel[0]) > EPS) {
    angle_of_attack = atan2(linear_vel[2], linear_vel[0]);
  }
  ignition::math::Vector3d force_axial_drag = ignition::math::Vector3d(-q0*param_axial_drag_coeff_*cos(angle_of_attack)*cos(angle_of_attack), 0, 0);

  // Apply forces and torques at CoV
  link_->AddLinkForce(bouyancy_force_body + added_mass_force + force_visc + force_fins + force_axial_drag);
  link_->AddRelativeTorque(added_mass_moment + moment_visc + moment_fins);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboLTADynamicsPlugin);
}
