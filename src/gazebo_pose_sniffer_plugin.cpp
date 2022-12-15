/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo_pose_sniffer_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PoseSnifferPlugin)

PoseSnifferPlugin::PoseSnifferPlugin() {}

PoseSnifferPlugin::~PoseSnifferPlugin() {}

void PoseSnifferPlugin::InitializeUdpEndpoint(sdf::ElementPtr const &sdf) {
  memset(&_sockaddr, 0, sizeof(_sockaddr));

  _sockaddr.sin_family = AF_INET;  // IPv4

  if (sdf->HasElement("pose_receiver_ip")) {
    std::string receiver_ip =
        sdf->GetElement("pose_receiver_ip")->Get<std::string>();

    _sockaddr.sin_addr.s_addr = inet_addr(receiver_ip.c_str());
    gzmsg << "[gazebo_pose_sniffer_plugin] Sniffing position data to ip: "
          << receiver_ip;
  } else {
    _sockaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    gzmsg
        << "[gazebo_pose_sniffer_plugin] Sniffing position data to localhost ";
  }
  if (sdf->HasElement("pose_receiver_port")) {
    int receiver_port = sdf->GetElement("pose_receiver_port")->Get<int>();
    _sockaddr.sin_port = htons(receiver_port);
    gzmsg << "and port " << receiver_port << std::endl;
  } else {
    _sockaddr.sin_port = htons(7000);
    gzmsg << "and port 7000" << std::endl;
  }

  // Creating socket file descriptor
  if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("socket creation failed");
    exit(EXIT_FAILURE);
  }
}


void PoseSnifferPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
   InitializeUdpEndpoint(sdf);

   _update_connection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&PoseSnifferPlugin::OnUpdate, this, _1));

  auto tracked_link_element = sdf->GetElement("tracked_link");
  while (tracked_link_element) {
    auto link =
        model->GetLink(tracked_link_element->Get<std::string>().c_str());
    if (link != nullptr) {
      _links.push_back(link);
    } else {
      gzerr << "[gazebo_pose_sniffer_plugin] Link : \""
            << tracked_link_element->Get<std::string>().c_str()
            << " \" not found in the sdf file, are you sure of its name or "
               "that it's not part of an included sdf file ?"
            << std::endl;
    }
    tracked_link_element = tracked_link_element->GetNextElement("tracked_link");
  }
  _poses.resize(_links.size());
  int i = 0;
  for (auto &pose : _poses) {
    pose.systemId = 1;
    pose.elementId = i;
    ++i;
  }
}

void PoseSnifferPlugin::OnUpdate(common::UpdateInfo const &updateInfo) {
  unsigned int i = 0;
  for (auto link : _links) {
    auto pose = link->WorldPose();
    auto position = pose.Pos();
    auto rotation = pose.Rot();

    auto &pose_to_update = _poses[i];

    pose_to_update.x = position.X();
    pose_to_update.y = position.Y();
    pose_to_update.z = position.Z();
    pose_to_update.pitch = rotation.Pitch();
    pose_to_update.yaw = rotation.Yaw();
    pose_to_update.roll = rotation.Roll();
    ++i;
  }
  if (!_poses.empty()) {
    sendto(_fd, (const char *)&_poses.front(),
           sizeof(_poses[0]) * _poses.size(), 0,
           (const struct sockaddr *)&_sockaddr, sizeof(_sockaddr));
  }
}