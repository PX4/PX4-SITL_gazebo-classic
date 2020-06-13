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
 */

#include <gazebo_gps_plugin.h>

#include <boost/algorithm/string.hpp>

using namespace std;

namespace gazebo {
GZ_REGISTER_SENSOR_PLUGIN(GpsPlugin)

GpsPlugin::GpsPlugin()
{ }

GpsPlugin::~GpsPlugin()
{
    if (updateConnection_)
      updateConnection_->~Connection();
    parentSensor_.reset();
    world_->Reset();
}

bool GpsPlugin::checkWorldHomePosition(physics::WorldPtr world) {
#if GAZEBO_MAJOR_VERSION >= 9
  common::SphericalCoordinatesPtr spherical_coords = world_->SphericalCoords();
#else
  common::SphericalCoordinatesPtr spherical_coords = world_->GetSphericalCoordinates();
#endif

  if (!spherical_coords) {
    return false;
  }
  world_latitude_ = spherical_coords->LatitudeReference().Degree() * M_PI / 180.0;
  world_longitude_ = spherical_coords->LongitudeReference().Degree() * M_PI / 180.0;
  world_altitude_ = spherical_coords->GetElevationReference();
  // This logic is required given that the spherical coordinates reference call
  // return 0 if the spherical coordnates are not defined in the world file
  return (world_latitude_ && world_longitude_ && world_altitude_) ? true : false;
}

void GpsPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  parentSensor_ = std::dynamic_pointer_cast<sensors::GpsSensor>(_parent);

  if (!parentSensor_)
    gzthrow("GpsPlugin requires a GPS Sensor as its parent");

  // Get the root model name
  const string scopedName = _parent->ParentName();
  vector<std::string> names_splitted;
  boost::split(names_splitted, scopedName, boost::is_any_of("::"));
  names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted),
                            [](const string& name)
                            { return name.size() == 0; }), end(names_splitted));
  const string rootModelName = names_splitted.front(); // The first element is the name of the root model

  // the second to the last name is the model name
  const string parentSensorModelName = names_splitted.rbegin()[1];

  // store the model name
  model_name_ = names_splitted[0];

  // get gps topic name
  if(_sdf->HasElement("topic")) {
    gps_topic_ = parentSensor_->Topic();
  } else {
    // if not set by parameter, get the topic name from the model name
    gps_topic_ = parentSensorModelName;
    gzwarn << "[gazebo_gps_plugin]: " + names_splitted.front() + "::" + names_splitted.rbegin()[1] +
      " using gps topic \"" << parentSensorModelName << "\"\n";
  }

  // Store the pointer to the world.
  world_ = physics::get_world(parentSensor_->WorldName());

  #if GAZEBO_MAJOR_VERSION >= 9
    last_time_ = world_->SimTime();
    last_gps_time_ = world_->SimTime();
  #else
    last_time_ = world_->GetSimTime();
    last_gps_time_ = world_->GetSimTime();
  #endif

  // Use environment variables if set for home position.
  const char *env_lat = std::getenv("PX4_HOME_LAT");
  const char *env_lon = std::getenv("PX4_HOME_LON");
  const char *env_alt = std::getenv("PX4_HOME_ALT");

  // Get noise param
  if (_sdf->HasElement("gpsNoise")) {
    getSdfParam<bool>(_sdf, "gpsNoise", gps_noise_, gps_noise_);
  } else {
    gps_noise_ = false;
  }

  bool world_has_origin = checkWorldHomePosition(world_);

  if (env_lat) {
    lat_home = std::stod(env_lat) * M_PI / 180.0;
    gzmsg << "Home latitude is set to " << lat_home << ".\n";
  } else if (world_has_origin) {
    lat_home = world_latitude_;
    gzmsg << "Home latitude is set to " << lat_home << ".\n";
  } else if(_sdf->HasElement("homeLatitude")) {
    double latitude;
    getSdfParam<double>(_sdf, "homeLatitude", latitude, 47.397742);
    lat_home = latitude * M_PI / 180.0;
  }

  if (env_lon) {
    lon_home = std::stod(env_lon) * M_PI / 180.0;
    gzmsg << "Home longitude is set to " << lon_home << ".\n";
  } else if (world_has_origin) {
    lon_home = world_longitude_;
    gzmsg << "Home longitude is set to " << lon_home << ".\n";
  } else if(_sdf->HasElement("homeLongitude")) {
    double longitude;
    getSdfParam<double>(_sdf, "homeLongitude", longitude, 8.545594);
    lon_home = longitude * M_PI / 180.0;
  }

  if (env_alt) {
    alt_home = std::stod(env_alt);
    gzmsg << "Home altitude is set to " << alt_home << ".\n";
  } else if (world_has_origin) {
    alt_home = world_altitude_;
    gzmsg << "Home altitude is set to " << alt_home << ".\n";
  } else if(_sdf->HasElement("homeAltitude")) {
    getSdfParam<double>(_sdf, "homeAltitude", alt_home, alt_home);
  }

  // get random walk in XY plane
  if (_sdf->HasElement("gpsXYRandomWalk")) {
    getSdfParam<double>(_sdf, "gpsXYRandomWalk", gps_xy_random_walk_, kDefaultGpsXYRandomWalk);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default random walk in XY plane: "
           << kDefaultGpsXYRandomWalk << "\n";
  }

  // get random walk in Z
  if (_sdf->HasElement("gpsZRandomWalk")) {
    getSdfParam<double>(_sdf, "gpsZRandomWalk", gps_z_random_walk_, kDefaultGpsZRandomWalk);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default random walk in Z: "
           << kDefaultGpsZRandomWalk << "\n";
  }

  // get position noise density in XY plane
  if (_sdf->HasElement("gpsXYNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsXYNoiseDensity", gps_xy_noise_density_, kDefaultGpsXYNoiseDensity);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default position noise density in XY plane: "
           << kDefaultGpsXYNoiseDensity << "\n";
  }

  // get position noise density in Z
  if (_sdf->HasElement("gpsZNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsZNoiseDensity", gps_z_noise_density_, kDefaultGpsZNoiseDensity);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default position noise density in Z: "
           << kDefaultGpsZNoiseDensity << "\n";
  }

  // get velocity noise density in XY plane
  if (_sdf->HasElement("gpsVXYNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsVXYNoiseDensity", gps_vxy_noise_density_, kDefaultGpsVXYNoiseDensity);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default velocity noise density in XY plane: "
           << kDefaultGpsVXYNoiseDensity << "\n";
  }

  // get velocity noise density in Z
  if (_sdf->HasElement("gpsVZNoiseDensity")) {
    getSdfParam<double>(_sdf, "gpsVZNoiseDensity", gps_vz_noise_density_, kDefaultGpsVZNoiseDensity);
  } else {
    gzwarn << "[gazebo_gps_plugin] Using default velocity noise density in Z: "
           << kDefaultGpsVZNoiseDensity << "\n";
  }

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Check first if the model was loaded before storing a pointer to it
  addEntityConnection_ = event::Events::ConnectAddEntity(
      std::bind(&GpsPlugin::addEntityEventCallback, this, std::placeholders::_1));

  if (addEntityConnection_->Id() < 1) {
    // Listen to the update event. This event is broadcast every simulation iteration.
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&GpsPlugin::OnUpdate, this, _1));
  }

  gravity_W_ = world_->Gravity();

  gps_pub_ = node_handle_->Advertise<sensor_msgs::msgs::SITLGps>("~/" + model_name_ + "/link/" + gps_topic_, 10);
  gt_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Groundtruth>("~/" + model_name_ + "/groundtruth", 10);
}

void GpsPlugin::addEntityEventCallback(const std::string &name) {
    // Start listening to the update event when the loaded entity matches.
    if (name == model_name_) {
      // Listen to the update event. This event is broadcast every simulation iteration.
      updateConnection_ = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GpsPlugin::OnUpdate, this, _1));
    }
}

void GpsPlugin::OnUpdate(const common::UpdateInfo&){
  // Store the pointer to the model.
  if (model_ == NULL)
    model_ = world_->ModelByName(model_name_);

#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = model_->WorldPose();    // TODO(burrimi): Check tf
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(model_->GetWorldPose());    // TODO(burrimi): Check tf
#endif
  ignition::math::Vector3d& pos_W_I = T_W_I.Pos();           // Use the models' world position for GPS and groundtruth
  ignition::math::Quaterniond& att_W_I = T_W_I.Rot();
  // reproject position without noise into geographic coordinates
  auto latlon_gt = reproject(pos_W_I);

  // Use the models' world position for GPS velocity.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d velocity_current_W = model_->WorldLinearVel();
#else
  ignition::math::Vector3d velocity_current_W = ignitionFromGazeboMath(model_->GetWorldLinearVel());
#endif

  ignition::math::Vector3d velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.Z() = 0;

  // update noise parameters if gps_noise_ is set
  if (gps_noise_) {
    noise_gps_pos.X() = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_pos.Y() = gps_xy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_pos.Z() = gps_z_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_vel.X() = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_vel.Y() = gps_vxy_noise_density_ * sqrt(dt) * randn_(rand_);
    noise_gps_vel.Z() = gps_vz_noise_density_ * sqrt(dt) * randn_(rand_);
    random_walk_gps.X() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_);
    random_walk_gps.Y() = gps_xy_random_walk_ * sqrt(dt) * randn_(rand_);
    random_walk_gps.Z() = gps_z_random_walk_ * sqrt(dt) * randn_(rand_);
  }
  else {
    noise_gps_pos.X() = 0.0;
    noise_gps_pos.Y() = 0.0;
    noise_gps_pos.Z() = 0.0;
    noise_gps_vel.X() = 0.0;
    noise_gps_vel.Y() = 0.0;
    noise_gps_vel.Z() = 0.0;
    random_walk_gps.X() = 0.0;
    random_walk_gps.Y() = 0.0;
    random_walk_gps.Z() = 0.0;
  }

  // gps bias integration
  gps_bias.X() += random_walk_gps.X() * dt - gps_bias.X() / gps_corellation_time_;
  gps_bias.Y() += random_walk_gps.Y() * dt - gps_bias.Y() / gps_corellation_time_;
  gps_bias.Z() += random_walk_gps.Z() * dt - gps_bias.Z() / gps_corellation_time_;

  // reproject position with noise into geographic coordinates
  auto pos_with_noise = pos_W_I + noise_gps_pos + gps_bias;
  auto latlon = reproject(pos_with_noise);

  // standard deviation TODO: add a way of computing this
  std_xy = 1.0;
  std_z = 1.0;

  // fill SITLGps msg
  gps_msg.set_time_usec(current_time.Double() * 1e6);
  gps_msg.set_latitude_deg(latlon.first * 180.0 / M_PI);
  gps_msg.set_longitude_deg(latlon.second * 180.0 / M_PI);
  gps_msg.set_altitude(pos_W_I.Z() + alt_home + noise_gps_pos.Z() + gps_bias.Z());
  gps_msg.set_eph(std_xy);
  gps_msg.set_epv(std_z);
  gps_msg.set_velocity(velocity_current_W_xy.Length());
  gps_msg.set_velocity_east(velocity_current_W.X() + noise_gps_vel.Y());
  gps_msg.set_velocity_north(velocity_current_W.Y() + noise_gps_vel.X());
  gps_msg.set_velocity_up(velocity_current_W.Z() + noise_gps_vel.Z());

  // add msg to buffer
  gps_delay_buffer.push(gps_msg);

  // apply GPS delay
  if ((current_time - last_gps_time_).Double() > gps_update_interval_) {
    last_gps_time_ = current_time;

    while (true) {
      gps_msg = gps_delay_buffer.front();
      double gps_current_delay = current_time.Double() - gps_delay_buffer.front().time_usec() / 1e6f;
      if (gps_delay_buffer.empty()) {
        // abort if buffer is empty already
        break;
      }
      // remove data that is too old or if buffer size is too large
      if (gps_current_delay > gps_delay) {
        gps_delay_buffer.pop();
        // remove data if buffer too large
      } else if (gps_delay_buffer.size() > gps_buffer_size_max) {
        gps_delay_buffer.pop();
      } else {
        // if we get here, we have good data, stop
        break;
      }
    }
    // publish SITLGps msg at 5hz
    gps_pub_->Publish(gps_msg);
  }

  // fill Groundtruth msg
  groundtruth_msg.set_time_usec(current_time.Double() * 1e6);
  groundtruth_msg.set_latitude_rad(latlon_gt.first);
  groundtruth_msg.set_longitude_rad(latlon_gt.second);
  groundtruth_msg.set_altitude(pos_W_I.Z() + alt_home);
  groundtruth_msg.set_velocity_east(velocity_current_W.X());
  groundtruth_msg.set_velocity_north(velocity_current_W.Y());
  groundtruth_msg.set_velocity_up(velocity_current_W.Z());
  groundtruth_msg.set_attitude_q_w(att_W_I.W());
  groundtruth_msg.set_attitude_q_x(att_W_I.X());
  groundtruth_msg.set_attitude_q_y(att_W_I.Y());
  groundtruth_msg.set_attitude_q_z(att_W_I.Z());

  // publish Groundtruth msg at full rate
  gt_pub_->Publish(groundtruth_msg);

  last_time_ = current_time;
}

std::pair<double, double> GpsPlugin::reproject(ignition::math::Vector3d& pos)
{
  // reproject local position to gps coordinates
  double x_rad = pos.Y() / earth_radius;    // north
  double y_rad = pos.X() / earth_radius;    // east
  double c = sqrt(x_rad * x_rad + y_rad * y_rad);
  double sin_c = sin(c);
  double cos_c = cos(c);
  double lat_rad, lon_rad;

  if (c != 0.0) {
    lat_rad = asin(cos_c * sin(lat_home) + (x_rad * sin_c * cos(lat_home)) / c);
    lon_rad = (lon_home + atan2(y_rad * sin_c, c * cos(lat_home) * cos_c - x_rad * sin(lat_home) * sin_c));
  } else {
    lat_rad = lat_home;
    lon_rad = lon_home;
  }

  return std::make_pair (lat_rad, lon_rad);
}
} // namespace gazebo
