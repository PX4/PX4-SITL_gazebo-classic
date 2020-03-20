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
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#ifndef _GAZEBO_GPS_ORIGIN_PLUGIN_HH_
#define _GAZEBO_GPS_ORIGIN_PLUGIN_HH_

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <queue>

#include <sdf/sdf.hh>
#include <common.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>

#include <SITLGps.pb.h>

namespace gazebo
{
class GpsOriginPlugin : public WorldPlugin
{
public:
  GpsOriginPlugin();
  virtual ~GpsOriginPlugin();

protected:
  virtual void Load(physics::WorldPtr world, sdf::ElementPtr sdf);
  virtual void OnUpdate(const common::UpdateInfo&);

private:
  std::string namespace_;
  event::ConnectionPtr updateConnection_;

  physics::WorldPtr world_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr gps_origin_pub_;

  sensor_msgs::msgs::SITLGps gps_msg;

  common::Time last_time_;

  // Zurich Irchel Park
  double lat_home = 47.397742 * M_PI / 180.0;  // rad
  double lon_home = 8.545594 * M_PI / 180.0;   // rad
  double alt_home = 488.0;                     // meters

  static constexpr double interval_ = 1.0; // 5hz

};     // class GAZEBO_VISIBLE GpsOriginPlugin
}      // namespace gazebo
#endif // _GAZEBO_GPS_ORIGIN_PLUGIN_HH_
