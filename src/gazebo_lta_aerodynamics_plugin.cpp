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
 * @brief LTA Aerodynamics Plugin
 *
 * This plugin simulates the aerodynamics of an airship.
 * The equations are based on the paper:
 * Dynamic Modelling of the Airship with MATLAB using Geometric Aerodynamic Parameters
 * Authors: M.Z. Ashraf, M.A. Choudhry
 *
 * @author Anton Erasmus <anton@flycloudline.com>
 */

#include "gazebo_lta_aerodynamics_plugin.h"
#include <ignition/math.hh>
#include <iostream>

namespace gazebo {

GazeboLTAAerodynamicsPlugin::~GazeboLTAAerodynamicsPlugin() {
  updateConnection_->~Connection();
}

void GazeboLTAAerodynamicsPlugin::Publish() {}

void GazeboLTAAerodynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  //Get link name
  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify a linkName.\n";
  }
  link_ = model_->GetLink(link_name_);

  // Get the air density
  if (_sdf->HasElement("air_density")) {
    air_density_ = _sdf->GetElement("air_density")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify air_density.\n";
  }

  // Get airship dimensions
  if (_sdf->HasElement("length")) {
    length_ = _sdf->GetElement("length")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify length.\n";
  }
  if (_sdf->HasElement("diameter")) {
    diameter_ = _sdf->GetElement("diameter")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify diameter.\n";
  }

  // Get added mass components
  if (_sdf->HasElement("m11")) {
    m11_ = _sdf->GetElement("m11")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify m11.\n";
  }
  if (_sdf->HasElement("m22")) {
    m22_ = _sdf->GetElement("m22")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify m22.\n";
  }
  if (_sdf->HasElement("m26")) {
    m26_ = _sdf->GetElement("m26")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify m26.\n";
  }
  if (_sdf->HasElement("m33")) {
    m33_ = _sdf->GetElement("m33")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify m33.\n";
  }
  if (_sdf->HasElement("m35")) {
    m35_ = _sdf->GetElement("m35")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify m35.\n";
  }
  if (_sdf->HasElement("m44")) {
    m44_ = _sdf->GetElement("m44")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify m44.\n";
  }
  if (_sdf->HasElement("m53")) {
    m53_ = _sdf->GetElement("m53")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify m53.\n";
  }
  if (_sdf->HasElement("m55")) {
    m55_ = _sdf->GetElement("m55")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify m55.\n";
  }
  if (_sdf->HasElement("m62")) {
    m62_ = _sdf->GetElement("m62")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify m62.\n";
  }
  if (_sdf->HasElement("m66")) {
    m66_ = _sdf->GetElement("m66")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify m66.\n";
  }

  // Get aerodynamic coefficients
  if(_sdf->HasElement("cx1")) {
    cx1_ = _sdf->GetElement("cx1")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cx1.\n";
  }
  if(_sdf->HasElement("cx2")) {
    cx2_ = _sdf->GetElement("cx2")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cx2.\n";
  }
  if(_sdf->HasElement("cx3")) {
    cx3_ = _sdf->GetElement("cx3")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cx3.\n";
  }
  if(_sdf->HasElement("cy1")) {
    cy1_ = _sdf->GetElement("cy1")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cy1.\n";
  }
  if(_sdf->HasElement("cy2")) {
    cy2_ = _sdf->GetElement("cy2")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cy2.\n";
  }
  if(_sdf->HasElement("cy3")) {
    cy3_ = _sdf->GetElement("cy3")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cy3.\n";
  }
  if(_sdf->HasElement("cz1")) {
    cz1_ = _sdf->GetElement("cz1")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cz1.\n";
  }
  if(_sdf->HasElement("cz2")) {
    cz2_ = _sdf->GetElement("cz2")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cz2.\n";
  }
  if(_sdf->HasElement("cz3")) {
    cz3_ = _sdf->GetElement("cz3")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cz3.\n";
  }
  if(_sdf->HasElement("cl2")) {
    cl2_ = _sdf->GetElement("cl2")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cl2.\n";
  }
  if(_sdf->HasElement("cl3")) {
    cl3_ = _sdf->GetElement("cl3")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cl3.\n";
  }
  if(_sdf->HasElement("cl4")) {
    cl4_ = _sdf->GetElement("cl4")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cl4.\n";
  }
  if(_sdf->HasElement("cm1")) {
    cm1_ = _sdf->GetElement("cm1")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cm1.\n";
  }
  if(_sdf->HasElement("cm2")) {
    cm2_ = _sdf->GetElement("cm2")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cm2.\n";
  }
  if(_sdf->HasElement("cm3")) {
    cm3_ = _sdf->GetElement("cm3")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cm3.\n";
  }
  if(_sdf->HasElement("cm5")) {
    cm5_ = _sdf->GetElement("cm5")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cm5.\n";
  }
  if(_sdf->HasElement("cn1")) {
    cn1_ = _sdf->GetElement("cn1")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cn1.\n";
  }
  if(_sdf->HasElement("cn2")) {
    cn2_ = _sdf->GetElement("cn2")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cn2.\n";
  }
  if(_sdf->HasElement("cn3")) {
    cn3_ = _sdf->GetElement("cn3")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cn3.\n";
  }
  if(_sdf->HasElement("cn5")) {
    cn5_ = _sdf->GetElement("cn5")->Get<float>();
  } else {
    gzerr << "[gazebo_lta_aerodynamics_plugin] Please specify cn5.\n";
  }

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboLTAAerodynamicsPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboLTAAerodynamicsPlugin::OnUpdate(const common::UpdateInfo& _info) {
  UpdateForcesAndMoments();
}

// Transformation functions
ignition::math::Vector3d GazeboLTAAerodynamicsPlugin::TransformNED2XYZ(ignition::math::Vector3d vec)
{
  ignition::math::Vector3d transformed = vec;
  transformed[1] = -transformed[1];
  transformed[2] = -transformed[2];
  return transformed;
}

ignition::math::Vector3d GazeboLTAAerodynamicsPlugin::TransformXYZ2NED(ignition::math::Vector3d vec)
{
  ignition::math::Vector3d transformed = vec;
  transformed[1] = -transformed[1];
  transformed[2] = -transformed[2];
  return transformed;
}

void GazeboLTAAerodynamicsPlugin::UpdateForcesAndMoments() {
  // Obtain the link's linear and angular velocity and acceleration
  #if GAZEBO_MAJOR_VERSION < 5
    gzerr << "[gazebo_lta_aerodynamics_plugin] Unsupported Gazebo version";
  #elif GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Vector3d linear_acc = link_->RelativeLinearAccel();
    ignition::math::Vector3d linear_vel = link_->RelativeLinearVel();
    ignition::math::Vector3d angular_acc = link_->RelativeAngularAccel();
    ignition::math::Vector3d angular_vel = link_->RelativeAngularVel();
  #else
    ignition::math::Vector3d linear_acc = ignitionFromGazeboMath(link_->GetRelativeLinearAccel());
    ignition::math::Vector3d linear_vel = ignitionFromGazeboMath(link_->RelativeLinearVel());
    ignition::math::Vector3d angular_acc = ignitionFromGazeboMath(link_->RelativeAngularAccel());
    ignition::math::Vector3d angular_vel = ignitionFromGazeboMath(link_->GetRelativeAngularVel());
  #endif

  float u = linear_vel[0];
  float v = linear_vel[1];
  float w = linear_vel[2];
  float p = angular_vel[0];
  float q = angular_vel[1];
  float r = angular_vel[2];

  // Transform vectors from XYZ to NED
  linear_acc = TransformXYZ2NED(linear_acc);
  linear_vel = TransformXYZ2NED(linear_vel);
  angular_acc = TransformXYZ2NED(angular_acc);
  angular_vel = TransformXYZ2NED(angular_vel);

  // Construct added mass matrices
  ignition::math::Matrix3d M11 = ignition::math::Matrix3d(m11_, 0, 0, 0, m22_, 0, 0, 0, m33_);
  ignition::math::Matrix3d M12 = ignition::math::Matrix3d(0, 0, 0, 0, 0, m26_, 0, m35_, 0);
  ignition::math::Matrix3d M21 = ignition::math::Matrix3d(0, 0, 0, 0, 0, m53_, 0, m62_, 0);
  ignition::math::Matrix3d M22 = ignition::math::Matrix3d(m44_, 0, 0, 0, m55_, 0, 0, 0, m66_);

  // Construct angular velocity semi symmetric matrix
  ignition::math::Matrix3d omega_x = ignition::math::Matrix3d(0, -angular_vel[2], angular_vel[1], angular_vel[2], 0, -angular_vel[0], -angular_vel[1], angular_vel[0], 0);

  // Calculate added mass forces and moments
  ignition::math::Vector3d added_mass_force;
  ignition::math::Vector3d added_mass_moment;

  added_mass_force = -(M11 * linear_acc + M12 * angular_acc) - (omega_x * (M11 * linear_vel + M12 * angular_vel));
  added_mass_moment = -(M21 * linear_acc + M22 * angular_acc) - (omega_x * (M21 * linear_vel + M22 * angular_vel));

  // Calculate dynamic pressure
  float v_total = sqrt(linear_vel[0]*linear_vel[0] + linear_vel[1]*linear_vel[1] + linear_vel[2]*linear_vel[2]);
  float q0 = 0.5f * air_density_ * linear_vel[0]*linear_vel[0];

  // Calculate aerodynamic forces
  float a = length_/2.0f;
  float b = diameter_/2.0f;
  float e = sqrt((a*a - b*b)/a);
  float S = IGN_PI * b * (b + a*a*sin(e)/e);

  float threshold = 1e-6;
  float alpha = 0;
  if(w >= threshold && u >= threshold)
    alpha = atan(w/u);
  float beta = 0;
  if(v_total >= threshold && v >= threshold)
    beta = asin(v/v_total);

  // gzmsg << "alpha: " << alpha << "\n";
  // gzmsg << "beta: " << beta << "\n\n";

  ignition::math::Vector3d aero_force;
  ignition::math::Vector3d aero_moment;

  aero_force[0] = q0*S*(cx1_*cos(alpha)*cos(alpha)*cos(beta)*cos(beta) + cx2_*(sin(2*alpha)*sin(alpha/2) + sin(2*beta)*sin(beta/2) + cx3_));
  aero_force[1] = q0*S*(cy1_*cos(beta/2)*sin(2*beta) + cy2_*sin(2*beta) + cy3_*sin(beta)*sin(abs(beta)));
  aero_force[2] = q0*S*(cz1_*cos(alpha/2)*sin(2*alpha) + cz2_*sin(2*alpha) + cz3_*sin(alpha)*sin(abs(alpha)));

  // gzmsg << "aero_force[0]: " << aero_force[0] << "\n";
  // gzmsg << "aero_force[1]: " << aero_force[1] << "\n";
  // gzmsg << "aero_force[2]: " << aero_force[2] << "\n\n";

  aero_moment[0] = q0*length_*S*(cl2_*sin(beta)*sin(abs(beta)) + 0.5*air_density_*S*(cl3_*r*abs(r) + cl4_*p*abs(p)));
  aero_moment[1] = q0*length_*S*(cm1_*cos(alpha/2)*sin(2*alpha) + cm2_*sin(2*alpha) + cm3_*sin(alpha)*sin(abs(alpha)) + 0.5*air_density_*S*cm5_*q*abs(q));
  aero_moment[2] = q0*length_*S*(cn1_*cos(beta/2)*sin(2*beta) + cn2_*sin(2*beta) + cn3_*sin(beta)*sin(abs(beta)) + 0.5*air_density_*S*cm5_*r*abs(r));

  // gzmsg << "aero_moment[0]: " << aero_moment[0] << "\n";
  // gzmsg << "aero_moment[1]: " << aero_moment[1] << "\n";
  // gzmsg << "aero_moment[2]: " << aero_moment[2] << "\n\n\n";

  // Apply forces and torques at CoV after transforming back to XYZ
  link_->AddLinkForce(TransformNED2XYZ(added_mass_force) - aero_force);
  link_->AddRelativeTorque(TransformNED2XYZ(added_mass_moment) - aero_moment);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboLTAAerodynamicsPlugin);
}
