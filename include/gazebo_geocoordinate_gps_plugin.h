/*
 * Copyright (C) 2012-2017 Open Source Robotics Foundation
 * Copyright (C) 2017-2020 PX4 Development Team. All rights reserved
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
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#ifndef _GAZEBO_GEOCOORDINATE_GPS_PLUGIN_HH_
#define _GAZEBO_GEOCOORDINATE_GPS_PLUGIN_HH_

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

#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/GpsSensor.hh>

#include <SITLGps.pb.h>

namespace gazebo
{
static constexpr double kDefaultUpdateRate = 5.0;               // hz
static constexpr double kDefaultGpsXYRandomWalk = 2.0;          // (m/s) / sqrt(hz)
static constexpr double kDefaultGpsZRandomWalk = 4.0;           // (m/s) / sqrt(hz)
static constexpr double kDefaultGpsXYNoiseDensity = 2.0e-4;     // (m) / sqrt(hz)
static constexpr double kDefaultGpsZNoiseDensity = 4.0e-4;      // (m) / sqrt(hz)
static constexpr double kDefaultGpsVXYNoiseDensity = 0.2;       // (m/s) / sqrt(hz)
static constexpr double kDefaultGpsVZNoiseDensity = 0.4;        // (m/s) / sqrt(hz)

class GAZEBO_VISIBLE GeocoordinateGpsPlugin : public SensorPlugin
{
public:
  GeocoordinateGpsPlugin();
  virtual ~GeocoordinateGpsPlugin();

protected:
  virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
  virtual void OnSensorUpdate();
  virtual void OnWorldUpdate(const common::UpdateInfo& /*_info*/);

  /**
   * @brief Convert WGS84 (LLA) to LV03/CH1903
   *
   * @param lat latitude (degrees) WGS84
   * @param lon lontitude (degrees) WGS84
   * @param alt Altitude WGS84
   * @param x
   * @param y
   * @param h
   */
  inline void forward(const double lat, const double lon, const double alt, double &y, double &x, double &h) {
    // 1. Convert the ellipsoidal latitudes φ and longitudes λ into arcseconds ["]
    const double lat_arc = lat * 3600.0;
    const double lon_arc = lon * 3600.0;

    // 2. Calculate the auxiliary values (differences of latitude and longitude relative to Bern in the unit [10000"]):
    //  φ' = (φ – 169028.66 ")/10000
    //  λ' = (λ – 26782.5 ")/10000
    const double lat_aux = (lat_arc - 169028.66) / 10000.0;
    const double lon_aux = (lon_arc - 26782.5) / 10000.0;

    // 3. Calculate projection coordinates in LV95 (E, N, h) or in LV03 (y, x, h)
    // E [m] = 2600072.37 + 211455.93 * λ' - 10938.51 * λ' * φ' - 0.36 * λ' * φ'2 - 44.54 * λ'3
    // y [m] = E – 2000000.00 N [m] = 1200147.07 + 308807.95 * φ' + 3745.25 * λ'2 + 76.63 * φ'2 - 194.56 * λ'2 * φ' +
    // 119.79 * φ'3 x [m] = N – 1000000.00
    // hCH [m] =hWGS – 49.55 + 2.73 * λ' + 6.94 * φ'
    const double E = 2600072.37 + 211455.93 * lon_aux - 10938.51 * lon_aux * lat_aux -
                    0.36 * lon_aux * std::pow(lat_aux, 2) - 44.54 * std::pow(lon_aux, 3);
    y = E - 2000000.00;
    const double N = 1200147.07 + 308807.95 * lat_aux + 3745.25 * std::pow(lon_aux, 2) + 76.63 * std::pow(lat_aux, 2) -
                    194.56 * std::pow(lon_aux, 2) * lat_aux + 119.79 * std::pow(lat_aux, 3);
    x = N - 1000000.00;

    h = alt - 49.55 + 2.73 * lon_aux + 6.84 * lat_aux;
  };

  /**
   * @brief  LV03/CH1903 to Convert WGS84 (LLA)
   *
   * @param x
   * @param y
   * @param h
   * @param lat latitude
   * @param lon longitude
   * @param alt altitude
   */
  static void reverse(const double y, const double x, const double h, double &lat, double &lon, double &alt) {
    // 1. Convert the projection coordinates E (easting) and N (northing) in LV95 (or y / x in LV03) into the civilian
    // system (Bern = 0 / 0) and express in the unit [1000 km]: E' = (E – 2600000 m)/1000000 = (y – 600000 m)/1000000
    // N' = (N – 1200000 m)/1000000 = (x – 200000 m)/1000000
    const double y_aux = (y - 600000.0) / 1000000.0;
    const double x_aux = (x - 200000.0) / 1000000.0;

    // 2. Calculate longitude λ and latitude φ in the unit [10000"]:
    //  λ' = 2.6779094 + 4.728982 * y' + 0.791484* y' * x' + 0.1306 * y' * x'2 - 0.0436 * y'3
    //  φ' = 16.9023892 + 3.238272 * x' - 0.270978 * y'2 - 0.002528 * x'2 - 0.0447 * y'2 * x' - 0.0140 * x'3
    // hWGS [m] = hCH + 49.55 - 12.60 * y' - 22.64 * x'
    const double lon_aux = 2.6779094 + 4.728982 * y_aux + 0.791484 * y_aux * x_aux + 0.1306 * y_aux * std::pow(x_aux, 2) -
                          0.0436 * std::pow(y_aux, 3);
    const double lat_aux = 16.9023892 + 3.238272 * x_aux - 0.270978 * std::pow(y_aux, 2) - 0.002528 * std::pow(x_aux, 2) -
                          0.0447 * std::pow(y_aux, 2) * x_aux - 0.0140 * std::pow(x_aux, 3);
    alt = h + 49.55 - 12.60 * y_aux - 22.64 * x_aux;

    lon = lon_aux * 100.0 / 36.0;
    lat = lat_aux * 100.0 / 36.0;
  };


private:
  std::string namespace_;
  std::string gps_id_;
  std::default_random_engine random_generator_;
  std::normal_distribution<float> standard_normal_distribution_;

  bool gps_noise_;

  std::string model_name_;

  sensors::GpsSensorPtr parentSensor_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr updateWorldConnection_;
  event::ConnectionPtr updateSensorConnection_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr gps_pub_;

  std::string gps_topic_;
  double update_rate_;

  common::Time last_gps_time_;
  common::Time last_time_;
  common::Time current_time_;
  common::Time start_time_;

  std::mutex data_mutex_;

  // Home defaults to Zurich Irchel Park
  // @note The home position can be specified using the environment variables:
  // PX4_HOME_LAT, PX4_HOME_LON, and PX4_HOME_ALT
  double lat_home_ = kDefaultHomeLatitude;
  double lon_home_ = kDefaultHomeLongitude;
  double alt_home_ = kDefaultHomeAltitude;
  double world_latitude_ = 0.0;
  double world_longitude_ = 0.0;
  double world_altitude_ = 0.0;

  // gps delay related
  static constexpr double gps_delay_ = 0.12;           // 120 ms
  static constexpr int gps_buffer_size_max_ = 1000;
  std::queue<sensor_msgs::msgs::SITLGps> gps_delay_buffer_;

  ignition::math::Vector3d gps_bias_;
  ignition::math::Vector3d noise_gps_pos_;
  ignition::math::Vector3d noise_gps_vel_;
  ignition::math::Vector3d random_walk_gps_;
  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;

  // gps noise parameters
  double std_xy_;    // meters
  double std_z_;     // meters
  std::default_random_engine rand_;
  std::normal_distribution<float> randn_;
  static constexpr const double gps_corellation_time_ = 60.0;    // s
  double gps_xy_random_walk_;
  double gps_z_random_walk_;
  double gps_xy_noise_density_;
  double gps_z_noise_density_;
  double gps_vxy_noise_density_;
  double gps_vz_noise_density_;
};     // class GAZEBO_VISIBLE GpsPlugin
}      // namespace gazebo
#endif // _GAZEBO_GPS_PLUGIN_HH_
