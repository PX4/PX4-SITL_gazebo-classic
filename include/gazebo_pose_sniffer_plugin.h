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
/* Desc: A controller taking the position of a point and sending it into a
 * socket Author: Frederic Taillandier
 */

#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <vector>
#include <sys/socket.h>

#include <gazebo/common/common.hh>
#include <gazebo/util/system.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

struct Pose {
  uint32_t elementId;
  double x;
  double y;
  double z;
  double pitch;
  double yaw;
  double roll;
};

struct Vehicle {
  uint32_t vehicleId;
  std::string vehicleName;
  std::vector<Pose> poses;
};

class GAZEBO_VISIBLE PoseSnifferPlugin : public ModelPlugin {
  constexpr static uint32_t BUFFERSIZE = 256; 
 private:
  int _fd = -1;
  struct sockaddr_in _sockaddr;
  uint8_t _buffer[BUFFERSIZE];
  std::string _pose_receiver_ip;
  int _pose_receiver_port;
  std::string _vehicle_name;
  event::ConnectionPtr _update_connection;
  
  std::vector<gazebo::physics::LinkPtr> _links;
  std::vector<Pose> _poses;
  void InitializeUdpEndpoint(sdf::ElementPtr const &sdf);

  /* 
  * @brief serialize the vehicle into a byte array ready to be sent over udp
  * return true if the serialization succeeded
  */
  unsigned int SerializeVehicle(Vehicle const &vehicle, uint8_t  (&buffer)[BUFFERSIZE]);
  bool DeserializeVehicle(Vehicle &vehicle, uint8_t const (&buffer)[BUFFERSIZE]);

 public:
  PoseSnifferPlugin();
  ~PoseSnifferPlugin();
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo &);
};
}  // namespace gazebo