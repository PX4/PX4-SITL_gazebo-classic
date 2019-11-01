/*
 * Copyright 2016 Austin Buchan, Nils Rottmann, TUHH Hamburg, Germany
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

#include <string>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "common.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "CommandMotorSpeed.pb.h"

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
}

namespace gazebo {

// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandSubTopic = "/gazebo/command/motor_speed";
static const std::string kDefaultLinkName = "base_link";        // the link name of the base hippocampus, see sdf file

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

// define class GazeboUUVPlugin
class GazeboUUVPlugin : public ModelPlugin {

  public:
    GazeboUUVPlugin() :
      ModelPlugin(),
      namespace_(kDefaultNamespace),                        // definde namespace, topic and link_name
      command_sub_topic_(kDefaultCommandSubTopic),
      link_name_(kDefaultLinkName) {}

    virtual ~GazeboUUVPlugin();

  protected:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void OnUpdate(const common::UpdateInfo&);

  private:
    std::string command_sub_topic_;
    std::string joint_0_name_;
    std::string joint_1_name_;
    std::string joint_2_name_;
    std::string joint_3_name_;
    std::string link_name_;
    std::string namespace_;

    transport::NodePtr node_handle_;
    transport::SubscriberPtr command_sub_;
    physics::ModelPtr model_;

    physics::JointPtr joint_0_;
    physics::JointPtr joint_1_;
    physics::JointPtr joint_2_;
    physics::JointPtr joint_3_;


    event::ConnectionPtr updateConnection_;
    physics::LinkPtr link_;

    double command_[4];
    double time_;
    double last_time_;
    double time_delta_;
    double motor_force_constant_;
    double motor_torque_constant_;
    double direction_[4];
    double ref_motor_rot_vel_;
    double rotor_velocity_slowdown_sim_;

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

    void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
};
}
