/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#pragma once

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>

#include "mavlink/v2.0/common/mavlink.h"

namespace gazebo
{
/**
 * @class GeotaggedImagesPlugin
 * Gazebo plugin that saves geotagged camera images to disk.
 */
class GAZEBO_VISIBLE GeotaggedImagesPlugin : public SensorPlugin
{
  public: GeotaggedImagesPlugin();

  public: virtual ~GeotaggedImagesPlugin();

  public: virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);
  public: void OnUpdate(const common::UpdateInfo& /*_info*/);
  void send_mavlink_message(const mavlink_message_t *message, const int destination_port=0);
  void handle_message(mavlink_message_t *msg);
  void pollForMAVLinkMessages(double _dt, uint32_t _timeoutMs);

  public: void OnNewFrame(const unsigned char *image);
  public: void OnNewGpsPosition(ConstVector3dPtr& v);
  public: void TakePicture();

  protected: float storeIntervalSec_;
  private: int imageCounter_;
  common::Time lastImageTime_{};
  common::Time last_time_{};

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  protected: sensors::CameraSensorPtr parentSensor_;
  protected: rendering::CameraPtr camera_;
  protected: rendering::ScenePtr scene_;
  private: event::ConnectionPtr newFrameConnection_;
  private: std::string storageDir_;
  private: msgs::Vector3d lastGpsPosition_;

  private: transport::NodePtr node_handle_;
  private: std::string namespace_;
  private: transport::SubscriberPtr gpsSub_;

  protected: unsigned int width_, height_, depth_;
  protected: unsigned int destWidth_, destHeight_; ///< output size
  protected: std::string format_;
  protected: bool capture_;

  private: int _fd;
  private: struct sockaddr_in _myaddr;    ///< The locally bound address
  private: struct sockaddr_in _srcaddr;   ///< SITL instance
  private: socklen_t _addrlen;
  private: unsigned char _buf[65535];
  private: struct pollfd fds[1];
  private: in_addr_t mavlink_addr_;
  private: int mavlink_udp_port_ = 14558;
};

} /* namespace gazebo */
