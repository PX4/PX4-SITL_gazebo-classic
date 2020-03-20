/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <gazebo_gps_origin_plugin.h>

namespace gazebo {
GZ_REGISTER_WORLD_PLUGIN(GpsOriginPlugin)

GpsOriginPlugin::GpsOriginPlugin() : WorldPlugin()
{ }

GpsOriginPlugin::~GpsOriginPlugin()
{
    if (updateConnection_)
      updateConnection_->~Connection();
}

void GpsOriginPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
  world_ = world;
  // Use environment variables if set for home position.
  const char *env_lat = std::getenv("PX4_HOME_LAT");
  const char *env_lon = std::getenv("PX4_HOME_LON");
  const char *env_alt = std::getenv("PX4_HOME_ALT");

  if (env_lat) {
    gzmsg << "Home latitude is set to " << env_lat << ".\n";
    lat_home = std::stod(env_lat) * M_PI / 180.0;
  } else if(sdf->HasElement("homeLatitude")) {
    double latitude;
    getSdfParam<double>(sdf, "homeLatitude", latitude, 47.397742);
    lat_home = latitude * M_PI / 180.0;
  }
  if (env_lon) {
    gzmsg << "Home longitude is set to " << env_lon << ".\n";
    lon_home = std::stod(env_lon) * M_PI / 180.0;
  } else if(sdf->HasElement("homeLongitude")) {
    double longitude;
    getSdfParam<double>(sdf, "homeLongitude", longitude, 8.545594);
    lon_home = longitude * M_PI / 180.0;
  }
  if (env_alt) {
    gzmsg << "Home altitude is set to " << env_alt << ".\n";
    alt_home = std::stod(env_alt);
  } else if(sdf->HasElement("homeAltitude")) {
    getSdfParam<double>(sdf, "homeAltitude", alt_home, alt_home);
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GpsOriginPlugin::OnUpdate, this, _1));

  gps_origin_pub_ = node_handle_->Advertise<sensor_msgs::msgs::SITLGps>("~/gps_origin", 10);
}

void GpsOriginPlugin::OnUpdate(const common::UpdateInfo&){
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif

  double dt = (current_time - last_time_).Double();

  if (dt > interval_) {
    gps_msg.set_time_usec(current_time.Double() * 1e6);
    gps_msg.set_latitude_deg(lat_home * 180.0/ M_PI);
    gps_msg.set_longitude_deg(lon_home * 180.0/ M_PI);
    gps_msg.set_altitude(alt_home);

    last_time_ = current_time;
    gps_origin_pub_->Publish(gps_msg);
  }
}
} // namespace gazebo
