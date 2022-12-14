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

#include "opencv2/opencv.hpp"
#include <opencv2/core/mat.hpp>

// ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace gazebo {
// Forward declare private data class
struct VideoPluginPrivate;

/// \brief A plugin that simulates Real Sense camera streams.
class GAZEBO_VISIBLE VideoPlugin : public ModelPlugin {
  /// \brief Constructor.
public:
  VideoPlugin();
  ~VideoPlugin();
  void OnUpdate();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnNewColorFrame(const rendering::CameraPtr cam,
                               const transport::PublisherPtr pub) const;

  /// \brief Private data pointer.
private:
  void
  video_trigger(std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  std::unique_ptr<VideoPluginPrivate> dataPtr;
  std::string file_name_;
  cv::VideoCapture cap_;
  char *data_;
  std::unique_ptr<cv::VideoWriter> videoPtr_;
  std::atomic<bool> recording_;

  // ROS2 communication
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};
} // namespace gazebo
#endif
