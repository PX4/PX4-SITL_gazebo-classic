/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#ifndef _GZRS_PLUGIN_HH_
#define _GZRS_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>
#include <string>
// ROS2
#include "raptor_interface/srv/set_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
namespace gazebo {
// Forward declare private data class
struct ServoPluginPrivate;

/// \brief A plugin that simulates Real Sense camera streams.
class GAZEBO_VISIBLE ServoPlugin : public ModelPlugin {
  /// \brief Constructor.
public:
  ServoPlugin();
  ~ServoPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate();

private:
  // callbacks
  void set_angle(const std_msgs::msg::Float64::SharedPtr msg);
  void set_servo(
      const std::shared_ptr<raptor_interface::srv::SetServo::Request> request,
      std::shared_ptr<raptor_interface::srv::SetServo::Response> response);

  // data pointer
  std::unique_ptr<ServoPluginPrivate> dataPtr;

  // Sdf parameters
  std::string subTopic_;
  std::string joint_name_;
  std::string link_name_;

  // objects
  physics::ModelPtr model_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;
  physics::JointController *joint_controller_;

  // state variables
  double angle_;
  bool td_;

  // ROS2 communication
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Service<raptor_interface::srv::SetServo>::SharedPtr service_;
};
} // namespace gazebo
#endif
