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
#include "CommandMotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "MotorSpeed.pb.h"
#include "Float.pb.h"
#include "common.h"


namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {
typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

class GazeboUUVPlugin : public MotorModel, public ModelPlugin {
 public:
  GazeboUUVPlugin()
      : ModelPlugin(),
        MotorModel(){
  }

  virtual ~GazeboUUVPlugin();
  virtual void InitializeParams();
  virtual void Publish();
 protected:
  virtual void UpdateForcesAndMoments();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);

 private:
  std::string namespace_;
  std::string command_sub_topic_;
  std::string link_base_;
  std::string link_prop_0_;
  std::string link_prop_1_;
  std::string link_prop_2_;
  std::string link_prop_3_;

  int turning_directions_[4];
  double motor_commands_[4];
  double motor_force_constant_;
  double motor_torque_constant_;
  double dead_zone_;

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
  transport::SubscriberPtr command_sub_;
  physics::ModelPtr model_;
  physics::LinkPtr baseLink_;
  physics::LinkPtr prop0Link_;
  physics::LinkPtr prop1Link_;
  physics::LinkPtr prop2Link_;
  physics::LinkPtr prop3Link_;
  event::ConnectionPtr updateConnection_;
  boost::thread callback_queue_thread_;
  void QueueThread();
  void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);

};
}
