/*
 * Copyright (C) 2012-2017 Open Source Robotics Foundation
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
 * This plugin publishes GPS data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com>
 */

#include "gazebo_gps_plugin.h"
#include <math.h>
#include "common.h"
#include <cstdlib>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GpsPlugin)

// Set global reference point
// Zurich Irchel Park: 47.397742, 8.545594, 488m
// Seattle downtown (15 deg declination): 47.592182, -122.316031, 86m
// Moscow downtown: 55.753395, 37.625427, 155m

// The home position can be specified using the environment variables:
// PX4_HOME_LAT, PX4_HOME_LON, and PX4_HOME_ALT

// Zurich Irchel Park
static double lat_home = 47.397742 * M_PI / 180;  // rad
static double lon_home = 8.545594 * M_PI / 180;  // rad
static double alt_home = 488.0; // meters
// Seattle downtown (15 deg declination): 47.592182, -122.316031
// static const double lat_home = 47.592182 * M_PI / 180;  // rad
// static const double lon_home = -122.316031 * M_PI / 180;  // rad
// static const double alt_home = 86.0; // meters
static const double earth_radius = 6353000;  // m

GpsPlugin::GpsPlugin() : ModelPlugin()
{
    
}

GpsPlugin::~GpsPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GpsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  // Use environment variables if set for home position.
  const char *env_lat = std::getenv("PX4_HOME_LAT");
  const char *env_lon = std::getenv("PX4_HOME_LON");
  const char *env_alt = std::getenv("PX4_HOME_ALT");
  
  // Get noise param
  if(_sdf->HasElement("gpsNoise"))
  {
    getSdfParam<bool>(_sdf, "gpsNoise", gps_noise_, gps_noise_);
  }
  
  else {
    gps_noise_ = true;    
  }

  if (env_lat) {
    gzmsg << "Home latitude is set to " << env_lat << ".\n";
    lat_home = std::stod(env_lat) * M_PI / 180.0;
  }
  if (env_lon) {
    gzmsg << "Home longitude is set to " << env_lon << ".\n";
    lon_home = std::stod(env_lon) * M_PI / 180.0;
  }
  if (env_alt) {
    gzmsg << "Home altitude is set to " << env_alt << ".\n";
    alt_home = std::stod(env_alt);
  }
  
  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);
  
   // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GpsPlugin::OnUpdate, this, _1));
  
  last_gps_time_ = world_->GetSimTime();
  gps_update_interval_ = 0.2;  // in seconds for 5Hz
  gps_delay_ = 0.12; // in seconds
  gravity_W_ = world_->GetPhysicsEngine()->GetGravity();
    
  gps_pub_ = node_handle_->Advertise<gps_msgs::msgs::Gps>("~/" + model_->GetName() + "/gps", 10);
}

void GpsPlugin::OnUpdate(const common::UpdateInfo&){
    
  common::Time current_time = world_->GetSimTime();
  double dt = (current_time - last_gps_time_).Double();
    
  math::Pose T_W_I = model_->GetWorldPose(); //TODO(burrimi): Check tf.
  math::Vector3& pos_W_I = T_W_I.pos;  // Use the models' world position for GPS and pressure alt.
  
  reproject(pos_W_I, lat_rad, lon_rad);

  math::Vector3 velocity_current_W = model_->GetWorldLinearVel();  // Use the models' world position for GPS velocity.

  math::Vector3 velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.z = 0;

  dt_gps = current_time.Double() - last_gps_time_.Double();
      
  noise_gps.x = 0;
  noise_gps.y = 0;
  noise_gps.z = 0;

  random_walk_gps.x = 0;
  random_walk_gps.y = 0;
  random_walk_gps.z = 0;
    
  if(gps_noise_){
    //update noise paramters
    noise_gps.x = gps_noise_density*sqrt(dt_gps)*standard_normal_distribution_(random_generator_);
    noise_gps.y = gps_noise_density*sqrt(dt_gps)*standard_normal_distribution_(random_generator_);
    noise_gps.z = gps_noise_density*sqrt(dt_gps)*standard_normal_distribution_(random_generator_);
        
    random_walk_gps.x = gps_random_walk*standard_normal_distribution_(random_generator_);
    random_walk_gps.y = gps_random_walk*standard_normal_distribution_(random_generator_);
    random_walk_gps.z = gps_random_walk*standard_normal_distribution_(random_generator_);
  }
    
  // bias integration
  gps_bias_.x += random_walk_gps.x - gps_bias_.x/gps_corellation_time;
  gps_bias_.y += random_walk_gps.y - gps_bias_.y/gps_corellation_time;
  gps_bias_.z += random_walk_gps.z - gps_bias_.z/gps_corellation_time;

  // standard deviation of random walk
  std_xy = gps_random_walk*gps_corellation_time/sqrtf(2*gps_corellation_time-1);
  std_z = std_xy;

  gps_msg.set_time(current_time.Double());
  gps_msg.set_latitude_deg((lat_rad * 180 / M_PI + (noise_gps.x + gps_bias_.x)*1e-5) * 1e7);
  gps_msg.set_longitude_deg((lon_rad * 180 / M_PI + (noise_gps.y + gps_bias_.y)*1e-5) * 1e7);
  gps_msg.set_altitude(((pos_W_I.z + alt_home) * 1000 + noise_gps.z + gps_bias_.z) / 1000.f);
  gps_msg.set_eph(100*(std_xy + gps_noise_density*gps_noise_density));
  gps_msg.set_epv(100*(std_z  + gps_noise_density*gps_noise_density));
  gps_msg.set_velocity(velocity_current_W_xy.GetLength() * 100);
  gps_msg.set_velocity_east(velocity_current_W.x * 100);
  gps_msg.set_velocity_north(velocity_current_W.y * 100);
  gps_msg.set_velocity_down(-velocity_current_W.z * 100);
  gps_msg.set_lat_rad(lat_rad);
  gps_msg.set_lon_rad(lon_rad);
  gps_pub_->Publish(gps_msg);

  last_gps_time_ = current_time;
}

void GpsPlugin::reproject(math::Vector3& pos, double& lat_rad, double& lon_rad)
{
  // reproject local position to gps coordinates
  double x_rad = pos.y / earth_radius; // north
  double y_rad = pos.x / earth_radius; // east
  double c = sqrt(x_rad * x_rad + y_rad * y_rad);
  double sin_c = sin(c);
  double cos_c = cos(c);
  
  if (c != 0.0) {
    lat_rad = asin(cos_c * sin(lat_home) + (x_rad * sin_c * cos(lat_home)) / c);
    lon_rad = (lon_home + atan2(y_rad * sin_c, c * cos(lat_home) * cos_c - x_rad * sin(lat_home) * sin_c));
  } else {
    lat_rad = lat_home;
    lon_rad = lon_home;
  }
}

}