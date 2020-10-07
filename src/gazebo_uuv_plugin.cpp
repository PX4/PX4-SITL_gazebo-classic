/*
 * Copyright 2020 Daniel Duecker, TU Hamburg, Germany
 * Copyright 2020 Philipp Hastedt, TU Hamburg, Germany
 * based on prior work by Nils Rottmann and Austin Buchan
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
 */

/**
 *  This plugin is an extension with regard to hydrodynamic effects.
 * It
 *  - computes the hydrodynamic effects (according to FOSSEN 2011 - Handbook of Marine Craft Hydrodynamics and Motion Control)
 *  - applies the Hydro effects  as forces/moments and are than applied to the vehicle
 *  - computes the forces/moments based on the motor commands
 */


#include "gazebo_uuv_plugin.h"
#include <ignition/math.hh>
#include <iostream>
#include <iomanip>
namespace gazebo {

GazeboUUVPlugin::~GazeboUUVPlugin() {
  updateConnection_->~Connection();
}

void GazeboUUVPlugin::InitializeParams() {}

void GazeboUUVPlugin::Publish() {}

void GazeboUUVPlugin::ParseBuoyancy(sdf::ElementPtr _sdf) {
  for (auto buoyancy_element = _sdf->GetFirstElement(); buoyancy_element != NULL; buoyancy_element = buoyancy_element->GetNextElement()) {
    // skip element if it is not a buoyancy-element
    if (buoyancy_element->GetName() != "buoyancy") {
      continue;
    }

    physics::LinkPtr link_ptr;
    std::string link_name = "";
    // check if link_name is specified. Otherwise skip this buoyancy-element.
    if (!getSdfParam(buoyancy_element, "link_name", link_name, link_name)) {
      gzwarn << "Skipping buoyancy element with unspecified 'link_name' tag!\n";
      continue;
    }
    link_ptr = model_->GetChildLink(link_name);
    // Check if link with specified name exists. Otherwise skip this
    // buoyancy-element.
    if (link_ptr == NULL) {
      gzerr << "Model has no link with name '" << link_name << "'!\n";
      continue;
    }
    buoyancy_s buoyancy_link;
    buoyancy_link.model_name = model_->GetName();
    buoyancy_link.link = link_ptr;
    buoyancy_link.buoyancy_force = ignition::math::Vector3d(0, 0, 0);
    buoyancy_link.cob = ignition::math::Vector3d(0, 0, 0);
    buoyancy_link.height_scale_limit = 0.1;
    double compensation = 0.0;

    if (buoyancy_element->HasElement("origin")) {
      buoyancy_link.cob = buoyancy_element->Get<ignition::math::Vector3d>("origin");
    }
    if (buoyancy_element->HasElement("compensation")) {
      compensation = buoyancy_element->Get<double>("compensation");
    }
    if (buoyancy_element->HasElement("height_scale_limit")) {
      buoyancy_link.height_scale_limit = std::abs(buoyancy_element->Get<double>("height_scale_limit"));
    }
    #if GAZEBO_MAJOR_VERSION >= 9
      buoyancy_link.buoyancy_force = -compensation * link_ptr->GetInertial()->Mass() * model_->GetWorld()->Gravity();
    #else
      buoyancy_link.buoyancy_force = -compensation * link_ptr->GetInertial()->GetMass() * model_->GetWorld()->Gravity();
    #endif
    buoyancy_links_.push_back(buoyancy_link);
    gzmsg << "Added buoyancy element for link '" << model_->GetName() << "::" << link_name << "'.\n";
  }

}

void GazeboUUVPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  namespace_.clear();


  namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  //Get links
  link_base_ = _sdf->GetElement("baseLinkName")->Get<std::string>();
  baseLink_ = model_->GetLink(link_base_);

  ParseBuoyancy(_sdf);

  //Get parameters for added mass and damping
  ignition::math::Vector3d added_mass_linear(0,0,0);
  getSdfParam<ignition::math::Vector3d>(_sdf, "addedMassLinear", added_mass_linear, added_mass_linear);
  X_udot_ = added_mass_linear[0];
  Y_vdot_ = added_mass_linear[1];
  Z_wdot_ = added_mass_linear[2];
  ignition::math::Vector3d added_mass_angular(0,0,0);
  getSdfParam<ignition::math::Vector3d>( _sdf, "addedMassAngular", added_mass_angular, added_mass_angular);
  K_pdot_ = added_mass_angular[0];
  M_qdot_ = added_mass_angular[1];
  N_rdot_ = added_mass_angular[2];
  ignition::math::Vector3d damping_linear(0,0,0);
  getSdfParam<ignition::math::Vector3d>(_sdf, "dampingLinear", damping_linear, damping_linear);
  X_u_ = damping_linear[0];
  Y_v_ = damping_linear[1];
  Z_w_ = damping_linear[2];
  ignition::math::Vector3d damping_angular(0,0,0);
  getSdfParam<ignition::math::Vector3d>(_sdf, "dampingAngular", damping_angular, damping_angular);
  K_p_ = damping_angular[0];
  M_q_ = damping_angular[1];
  N_r_ = damping_angular[2];

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboUUVPlugin::OnUpdate, this, _1));
  }

// This gets called by the world update start event.
void GazeboUUVPlugin::OnUpdate(const common::UpdateInfo& _info) {
  ApplyBuoyancy();
  UpdateForcesAndMoments(); // Hydrodynamics are computed here
  Publish();
}

void GazeboUUVPlugin::ApplyBuoyancy() {
  ignition::math::Vector3d force, cob;
  for (std::vector<buoyancy_s>::iterator entry = buoyancy_links_.begin(); entry != buoyancy_links_.end(); ++entry) {
    #if GAZEBO_MAJOR_VERSION >= 9
      ignition::math::Pose3d pose = entry->link->WorldPose();
    #else
      ignition::math::Pose3d pose = ignitionFromGazeboMath(entry->link->GetWorldPose());
    #endif
    cob = pose.Pos() + pose.Rot().RotateVector(entry->cob);
    force = entry->buoyancy_force;
    // apply linear scaling on buoyancy force if center of buoyancy z-coordinate
    // is in range [-height_scale_limit, +height_scale limit].
    double scale = std::abs((cob.Z()-entry->height_scale_limit) / (2*entry->height_scale_limit));
    if (cob.Z() > entry->height_scale_limit)
      scale = 0.0;
    scale = ignition::math::clamp(scale, 0.0, 1.0);
    force *= scale;
    entry->link->AddForceAtWorldPosition(force, cob);
  }
}

void GazeboUUVPlugin::UpdateForcesAndMoments() {
/**
 *  This method:
 *  - computes the hydrodynamic effects (according to FOSSEN 2011 - Handbook of Marine Craft Hydrodynamics and Motion Control)
 *  - applies the Hydro effects  as forces/moments and are than applied to the vehicle
 *  - computes the forces/moments based on the motor commands
 */

    //Calculate and add hydrodynamic forces
    #if GAZEBO_MAJOR_VERSION >= 9
      ignition::math::Vector3d linear_velocity = baseLink_->RelativeLinearVel();
      ignition::math::Vector3d angular_velocity = baseLink_->RelativeAngularVel();
    #else
      ignition::math::Vector3d linear_velocity = ignitionFromGazeboMath(baseLink_->GetRelativeLinearVel());
      ignition::math::Vector3d angular_velocity = ignitionFromGazeboMath(baseLink_->GetRelativeAngularVel());
    #endif
      double u = linear_velocity[0];
      double v = linear_velocity[1];
      double w = linear_velocity[2];
      double p = angular_velocity[0];
      double q = angular_velocity[1];
      double r = angular_velocity[2];

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

      ignition::math::Vector3d damping_force = D_FL*linear_velocity;
      ignition::math::Vector3d damping_torque = D_FA*angular_velocity;

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

      ignition::math::Vector3d coriolis_force = C_AD_FA*angular_velocity;
      ignition::math::Vector3d coriolis_torque = (C_AD_FA*linear_velocity) + (C_AD_TA*angular_velocity);

      //std::cout << C_AD_FA << "\n";
      //std::cout << "Linear:" << coriolis_force << "\n";
      //std::cout << "Angular:" << angular_velocity << "\n";

      baseLink_->AddRelativeForce(damping_force + coriolis_force);
      baseLink_->AddRelativeTorque(damping_torque + coriolis_torque);
    }

GZ_REGISTER_MODEL_PLUGIN(GazeboUUVPlugin);
}
