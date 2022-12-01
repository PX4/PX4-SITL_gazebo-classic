/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include "CommandMotorSpeed.pb.h"
#include "Float.pb.h"
#include "MotorSpeed.pb.h"
#include "Wind.pb.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rotors_model/motor_model.hpp>

#include "common.h"

namespace turning_direction {
const static int CCW = 1;
const static int CW = -1;
} // namespace turning_direction

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandSubTopic =
    "/gazebo/command/motor_speed";
static const std::string kDefaultMotorFailureNumSubTopic =
    "/gazebo/motor_failure_num";
static const std::string kDefaultMotorVelocityPubTopic = "/motor_speed";
std::string wind_sub_topic_ = "/world_wind";

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed>
    CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;

class GazeboMotorModel : public MotorModel, public ModelPlugin {
public:
  GazeboMotorModel() : ModelPlugin(), MotorModel() {}

  virtual ~GazeboMotorModel();

  virtual void InitializeParams();
  virtual void Publish();
  // void testProto(MotorSpeedPtr &msg);
protected:
  virtual void UpdateForcesAndMoments();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
  std::string command_sub_topic_{kDefaultCommandSubTopic};
  std::string joint_name_;
  std::string link_name_;
  std::string motor_speed_pub_topic_{kDefaultMotorVelocityPubTopic};
  std::string namespace_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_pub_;
  transport::SubscriberPtr command_sub_;
  transport::SubscriberPtr
      motor_failure_sub_; /*!< Subscribing to motor_failure_sub_topic_;
                             receiving motor number to fail, as an integer */
  transport::SubscriberPtr wind_sub_;

  physics::ModelPtr model_;
  physics::JointPtr joint_;

  physics::LinkPtr link_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;
};
} // namespace gazebo
