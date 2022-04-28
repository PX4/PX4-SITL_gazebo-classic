/*
 * Copyright 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
 * Copyright 2017 Siddharth Patel, NTU Singapore
 * Copyright 2022 SungTae Moon, KOREATECH Korea
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

#include "gazebo_motor_model.h"

#include <stdio.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "common.h"

// ROS Topic subscriber
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace gazebo {

// Default values
static const std::string kDefaultROSMotorNumSubTopic = "/motor_failure/motor_number";
static const std::string kDefaultMotorFailureNumPubTopic = "/gazebo/motor_failure_num";

class GazeboMotorFailure : public ModelPlugin {
 public:
  GazeboMotorFailure();

  void motorFailNumCallBack(const std_msgs::msg::Int32::SharedPtr msg);

  virtual ~GazeboMotorFailure();

 protected:

  /// \brief Create subscription to ROS topic that triggers the motor failure
  /// \details Inits a rosnode in case there's not one and subscribes to ROS_motor_num_sub_topic_
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /// \brief Updates and publishes the motor_Failure_Number_ to motor_failure_num_pub_topic_
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:

  event::ConnectionPtr updateConnection_;

  std::string ROS_motor_num_sub_topic_;
  std::string motor_failure_num_pub_topic_;
  std::string namespace_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_failure_pub_; /*!< Publish the motor_Failure_num to gazebo topic motor_failure_num_pub_topic_ */

  boost::thread callback_queue_thread_;

  msgs::Int motor_failure_msg_;
  int32_t motor_Failure_Number_;

  // ROS2 communication
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription;

};
}
