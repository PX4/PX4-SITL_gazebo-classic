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



#include <stdio.h>
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_model/motor_model.hpp>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "Float.pb.h"
#include "common.h"

namespace gazebo {

class GazeboUUVPlugin : public ModelPlugin {
 public:
  GazeboUUVPlugin() : ModelPlugin(){}

  virtual ~GazeboUUVPlugin();
  virtual void InitializeParams();
  void ParseBuoyancy(sdf::ElementPtr _sdf);
  void ApplyBuoyancy();
  virtual void Publish();
 protected:
  virtual void UpdateForcesAndMoments();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);

 private:
  struct buoyancy_s {
    std::string model_name;
    physics::LinkPtr link;
    ignition::math::Vector3d buoyancy_force;
    ignition::math::Vector3d cob;
    double height_scale_limit;
  };
  std::vector<buoyancy_s> buoyancy_links_;
  std::string namespace_;
  std::string link_base_;

  /* Hydro coefficients - FOSSEN 2011 */
  double X_u_;
  double Y_v_;
  double Z_w_;
  double K_p_;
  double M_q_;
  double N_r_;

  double X_udot_;
  double Y_vdot_;
  double Z_wdot_;
  double K_pdot_;
  double M_qdot_;
  double N_rdot_;

  transport::NodePtr node_handle_;
  physics::ModelPtr model_;
  physics::LinkPtr baseLink_;
  event::ConnectionPtr updateConnection_;

};
}
