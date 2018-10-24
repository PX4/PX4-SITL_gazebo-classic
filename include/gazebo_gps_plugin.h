/*
 * Copyright (C) 2012-2017 Open Source Robotics Foundation
 * Copyright (C) 2017-2018 PX4 Pro Development Team
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
/**
 * @brief GPS Plugin
 *
 * This plugin publishes GPS and Groundtruth data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#ifndef _GAZEBO_GPS_PLUGIN_HH_
#define _GAZEBO_GPS_PLUGIN_HH_

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <queue>
#include <random>

#include <sdf/sdf.hh>
#include <common.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <SITLGps.pb.h>
#include <Groundtruth.pb.h>

#if BUILD_ROS_INTERFACE
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include <gazebo_plugins/PubQueue.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>
#endif

namespace gazebo
{
class GAZEBO_VISIBLE GpsPlugin : public ModelPlugin
{
public:
  GpsPlugin();
  virtual ~GpsPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo&);

private:
  /// \brief The default GPS EPH
  static constexpr auto kDefaultEPH = 1.0;
  /// \brief The default GPS EPV
  static constexpr auto kDefaultEPV = 1.0;

  std::pair<double, double> reproject(ignition::math::Vector3d& pos);

#if BUILD_ROS_INTERFACE
  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosnode_;
  /// \brief The PoseWithCovarianceStamped publisher with reprojected global data in the ENU frame
  ros::Publisher gps_pose_pub_;
  /// \brief The NavSatFix publisher with GPS data
  ros::Publisher gps_fix_pub_;
  /// \brief The default reprojected GPS data topic
  static constexpr auto kDefaultRosGPSENUPubTopic = "/gps/local_enu";
  /// \brief The default GPS fix data topic
  static constexpr auto kDefaultRosGPSFixPubTopic = "/gps/fix";
  /// \brief A mutex to lock access to fields are used in message callbacks
  boost::mutex lock_;
  /// \brief Prevents blocking
  PubMultiQueue pmq_;
  /// \brief Fix pub custom queue
  ros::ServiceServer fix_srv_;
  /// \brief Pose pub custom queue
  ros::ServiceServer pose_srv_;
  /// \brief Fix topic name
  std::string ros_gps_fix_pub_topic_;
  /// \brief Pose topic name
  std::string ros_gps_pose_pub_topic_;
  /// \brief call back when using service
  bool ServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  /// \brief A ROS callback queue that helps process GPS fix messages
  ros::CallbackQueue fix_queue_;
  /// \brief A ROS callback queue that helps process pose messages
  ros::CallbackQueue pose_queue_;
  /// \brief A ROS publisher queue for GPS fix messages
  PubQueue<sensor_msgs::NavSatFix>::Ptr fix_pub_queue_;
  /// \brief A ROS publisher queue for pose messages
  PubQueue<geometry_msgs::PoseWithCovarianceStamped>::Ptr pose_pub_queue_;
  /// \brief Put local ENU pose data to the interface
  void PoseQueueThread();
  /// \brief Put GPS fix data to the interface
  void FixQueueThread();
  /// \brief A thread the keeps running the fix_pub_queue_
  boost::thread fix_callback_queue_thread_;
  /// \brief A thread the keeps running the pose_pub_queue_
  boost::thread pose_callback_queue_thread_;
#endif

  std::string namespace_;
  std::default_random_engine random_generator_;
  std::normal_distribution<float> standard_normal_distribution_;

  bool gps_noise_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr updateConnection_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr gt_pub_;
  transport::PublisherPtr gps_pub_;

  sensor_msgs::msgs::SITLGps gps_msg;
  sensor_msgs::msgs::Groundtruth groundtruth_msg;

  common::Time last_gps_time_;
  common::Time last_time_;

  // Set global reference point
  // Zurich Irchel Park: 47.397742, 8.545594, 488m
  // Seattle downtown (15 deg declination): 47.592182, -122.316031, 86m
  // Moscow downtown: 55.753395, 37.625427, 155m

  // The home position can be specified using the environment variables:
  // PX4_HOME_LAT, PX4_HOME_LON, and PX4_HOME_ALT

  // Zurich Irchel Park
  double lat_home = 47.397742 * M_PI / 180.0;  // rad
  double lon_home = 8.545594 * M_PI / 180.0;   // rad
  double alt_home = 488.0;                     // meters
  // Seattle downtown (15 deg declination): 47.592182, -122.316031
  // static const double lat_home = 47.592182 * M_PI / 180;    // rad
  // static const double lon_home = -122.316031 * M_PI / 180;  // rad
  // static const double alt_home = 86.0;                      // meters

  static constexpr const double earth_radius = 6353000.0;      // meters

  // gps delay related
  static constexpr double gps_update_interval_ = 0.2; // 5hz
  static constexpr double gps_delay = 0.12;           // 120 ms
  static constexpr int gps_buffer_size_max = 1000;
  std::queue<sensor_msgs::msgs::SITLGps> gps_delay_buffer;

  ignition::math::Vector3d gps_bias;
  ignition::math::Vector3d noise_gps_pos;
  ignition::math::Vector3d noise_gps_vel;
  ignition::math::Vector3d random_walk_gps;
  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;

  // gps noise parameters
  double _std_x;    // meters
  double _std_y;    // meters
  double _std_z;    // meters
  double eph_;     // meters
  double epv_;     // meters
  std::default_random_engine rand_;
  std::normal_distribution<float> randn_;
  static constexpr double _gps_corellation_time = 30.0;    // s
  static constexpr double _gps_xy_random_walk = 0.025;     // (m/s) / sqrt(hz)
  static constexpr double _gps_z_random_walk = 0.05;       // (m/s) / sqrt(hz)
  static constexpr double _gps_xy_noise_density = 2e-4;    // (m) / sqrt(hz)
  static constexpr double _gps_z_noise_density = 4e-4;     // (m) / sqrt(hz)
  static constexpr double _gps_vxy_noise_density = 2e-1;   // (m/s) / sqrt(hz)
  static constexpr double _gps_vz_noise_density = 4e-1;    // (m/s) / sqrt(hz)
};     // class GAZEBO_VISIBLE GpsPlugin
}      // namespace gazebo
#endif // _GAZEBO_GPS_PLUGIN_HH_
