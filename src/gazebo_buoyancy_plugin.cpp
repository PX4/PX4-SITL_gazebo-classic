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
 * @brief Buoyancy Plugin
 *
 * This plugin simulates buoyancy.
 *
 * @author Anton Erasmus <anton@flycloudline.com>
 */

#include "gazebo_buoyancy_plugin.h"
#include <ignition/math.hh>
#include <iostream>

namespace gazebo {

GazeboBuoyancyPlugin::~GazeboBuoyancyPlugin() {
  updateConnection_->~Connection();
}

void GazeboBuoyancyPlugin::Publish() {}

void GazeboBuoyancyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_buoyancy_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  //Get link name
  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    gzerr << "[gazebo_buoyancy_plugin] Please specify a linkName.\n";
  }
  link_ = model_->GetLink(link_name_);

  // Get link volume
  if (_sdf->HasElement("linkVolume")) {
    link_volume_ = _sdf->GetElement("linkVolume")->Get<float>();
  } else {
    gzerr << "[gazebo_buoyancy_plugin] Please specify a linkVolume.\n";
  }

  // Get fluid density
  if (_sdf->HasElement("fluidDensity")) {
    fluid_density_ = _sdf->GetElement("fluidDensity")->Get<float>();
  } else {
    gzerr << "[gazebo_buoyancy_plugin] Please specify a fluidDensity.\n";
  }

  // Obtain the gravitational acceleration constant
  #if GAZEBO_MAJOR_VERSION >= 9
    gravity_w_ = world_->Gravity();
  #else
    gravity_w_ = ignitionFromGazeboMath(world_->GetPhysicsEngine()->GetGravity());
  #endif

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboBuoyancyPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboBuoyancyPlugin::OnUpdate(const common::UpdateInfo& _info) {
  UpdateForcesAndMoments();
}

void GazeboBuoyancyPlugin::UpdateForcesAndMoments() {
  //Calculate buoyancy force
  ignition::math::Vector3d bouyancy_force_world = - fluid_density_ * link_volume_ * gravity_w_;
  ignition::math::Vector3d bouyancy_force_body = link_->WorldPose().Rot().Inverse().RotateVector(bouyancy_force_world);
  link_->AddLinkForce(bouyancy_force_body);

}

GZ_REGISTER_MODEL_PLUGIN(GazeboBuoyancyPlugin);
}
