/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
/* Desc: A basic cg calculation of a full gazebo model
 * Author: Garrett Gibo
 */

#ifndef _GAZEBO_CG_PLUGIN_HH_
#define _GAZEBO_CG_PLUGIN_HH_

#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <ignition/math.hh>

#include <iostream>
#include <time.h>

using namespace gazebo;

class GAZEBO_VISIBLE CGPlugin : public ModelPlugin {

public:
  CGPlugin();

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();

private:
  void OnUpdate();

  physics::ModelPtr _model;
  sdf::ElementPtr _sdf;

  std::string namespace_;

  event::ConnectionPtr updateConnection;

  // vector of all links in model
  std::vector<gazebo::physics::LinkPtr> model_links;

  // vector of masses for each link in model
  std::vector<double> link_masses;

  // accumulated mass of full model
  double total_mass;

  // cg position relative to center of model
  double cg_x, cg_y, cg_z;

  // pose of center of model
  ignition::math::Pose3d engine_cg;


};
#endif
