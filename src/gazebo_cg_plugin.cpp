/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
 *
 */

#include "gazebo_cg_plugin.h"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(CGPlugin)

CGPlugin::CGPlugin() {}

void CGPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->_model = _model;
  this->_sdf = _sdf;

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_cg_plugin] Please specify a robotNamespace.\n";
  }

  // Get all links in model
  model_links = _model->GetLinks();

  // initialize cg and mass to 0
  cg_x, cg_y, cg_z = 0;
  total_mass = 0;

}

void CGPlugin::Init() {
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CGPlugin::OnUpdate, this));

  // First Iteration to get all cg values and accumulate mass
  for (auto & link : model_links) {
    gazebo::physics::InertialPtr inertia = link->GetInertial();

    // Save masses to compute weighed sum
    link_masses.push_back(inertia->Mass());

    // Accumulate total mass to computer weighted sum
    total_mass += inertia->Mass();
  }

  // Second iteration do weighted average to calculate CoG
  int i = 0;
  for (auto & link : model_links) {
    const ignition::math::Pose3d cg = link->WorldCoGPose();

    // add weighted cg positions
    cg_x += cg.Pos().X() * link_masses[i] / total_mass;
    cg_y += cg.Pos().Y() * link_masses[i] / total_mass;
    cg_z += cg.Pos().Z() * link_masses[i] / total_mass;

    // use engine center as origin
    if (link->GetName() == "gimbal_ring_inner") engine_cg = cg;

    i++;
  }

  // Offset calculated cg based off engine cg
  cg_x -= engine_cg.Pos().X();
  cg_y -= engine_cg.Pos().Y();
  cg_z -= engine_cg.Pos().Z();

  cout << "Center of Gravity: " << cg_x << " -- " << cg_y << " -- "<< cg_z << endl;
}

void CGPlugin::OnUpdate() {};
