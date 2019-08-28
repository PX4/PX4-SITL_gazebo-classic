/*
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright 2019 Swift Engineering, Inc. <nuno.marques@dronesolutions.io>
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
 */

#pragma once

#include <random>
#include <thread>

#include <Eigen/Core>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_plugins/PubQueue.h>

#include <mav_msgs/default_topics.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/subscribe_options.h>

#include "Pressure.pb.h"
#include "Int32.pb.h"
#include "Imu.pb.h"
#include "IRLock.pb.h"
#include "Range.pb.h"
#include "SITLGps.pb.h"
#include "Wind.pb.h"

// custom ROS messages
#include <mavlink_sitl_gazebo/IRLock.h>

// std ROS messages
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/Range.h>

#include <common.h>

namespace gazebo {

typedef const boost::shared_ptr<const sensor_msgs::msgs::Pressure> GzBaroMsgPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> GzImuMsgPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> GzSITLGpsMsgPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::IRLock> GzIRLockMsgPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> GzRangeMsgPtr;

/// \brief    ROS interface plugin for Gazebo.
/// \details  This routes messages to/from Gazebo and ROS. This is used so that individual plugins are not ROS dependent.
///           This is a WorldPlugin, only one of these is designed to be enabled per Gazebo world.
class GAZEBO_VISIBLE GazeboRosInterface : public WorldPlugin {
 public:
  GazeboRosInterface();
  virtual ~GazeboRosInterface();

  void InitializeParams();
  void Publish();

 protected:
  /// \ brief   Loads the world plugin and inits the required configurations
  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  /// \brief  	This gets called by the world update start event.
  virtual void OnUpdate(const common::UpdateInfo&);

 private:
  /// \brief    Provides a way for GzConnectGazeboToRosTopicMsgCallback() to connect a Gazebo subscriber to ROS publishers.
  /// \details  GazeboMsgT  The type of the message that will be subscribed to the Gazebo framework.
  ///           A vector of ROS Publishers is passed as well so one callback may have multiple different publishers.
  template <typename GazeboMsgT>
  void ConnectHelper(
          void (GazeboRosInterface::*fp)(const boost::shared_ptr<GazeboMsgT const>&, std::vector<ros::Publisher>),
                  GazeboRosInterface* ptr, std::string gazeboNamespace,
                  std::string gazeboTopicName, std::vector<ros::Publisher>,
                  transport::NodePtr gz_node_handle);

  std::vector<gazebo::transport::NodePtr> nodePtrs_;
  std::vector<gazebo::transport::SubscriberPtr> subscriberPtrs_;

  /// \brief  Handle for the Gazebo node.
  transport::NodePtr gz_node_handle_;

  /// \brief  Handle for the ROS node.
  std::unique_ptr<ros::NodeHandle> ros_node_handle_;

  /// \brief  Pointer to the world
  physics::WorldPtr world_;

  /// \brief  Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  /// \brief  Callback that maps Gazebo sensor_msgs/Baro to ROS sensor_msgs/FluidPressure and sensor_msgs/Temperature
  void GzBaroMsgCallback(GzBaroMsgPtr& gz_baro_msg, std::vector<ros::Publisher> ros_publishers);
  /// \brief  Callback that maps Gazebo sensor_msgs/SITLGps to ROS sensor_msgs/NavSatFix
  void GzSITLGpsMsgCallback(GzSITLGpsMsgPtr& gz_sitl_gps_msg, std::vector<ros::Publisher> ros_publishers);
  /// \brief  Callback that maps Gazebo sensor_msgs/Imu to ROS sensor_msgs/Imu
  void GzImuMsgCallback(GzImuMsgPtr& gz_imu_msg, std::vector<ros::Publisher> ros_publishers);
  /// \brief  Callback that maps Gazebo sensor_msgs/IRLock to ROS (custom) mavlink_sitl_gazebo/IRLock
  void GzIRLockMsgCallback(GzIRLockMsgPtr& gz_irlock_msg, std::vector<ros::Publisher> ros_publishers);
  /// \brief  Callback that maps Gazebo sensor_msgs/Range (Lidar) to ROS sensor_msgs/Range
  void GzLidarMsgCallback(GzRangeMsgPtr& gz_lidar_msg, std::vector<ros::Publisher> ros_publishers);
  /// \brief  Callback that maps Gazebo sensor_msgs/Range (Sonar) to ROS sensor_msgs/Range
  void GzSonarMsgCallback(GzRangeMsgPtr& gz_sonar_msg, std::vector<ros::Publisher> ros_publishers);
}; // class GAZEBO_VISIBLE GazeboRosInterface

}  // namespace gazebo
