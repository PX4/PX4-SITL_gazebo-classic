/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Airspeed Plugin
 *
 * This plugin publishes Airspeed sensor data
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 * References:
 * [1] A brief summary of atmospheric modeling with citations:
 *     Cavcar, M., http://fisicaatmo.at.fcen.uba.ar/practicas/ISAweb.pdf
 */

#ifndef _GAZEBO_AIRSPEED_PLUGIN_HH_
#define _GAZEBO_AIRSPEED_PLUGIN_HH_

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <queue>
#include <random>

#include <sdf/sdf.hh>
#include <common.h>
#include <random>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/Sensor.hh>

#include <Airspeed.pb.h>
#include <Wind.pb.h>

namespace gazebo
{

typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;

static constexpr auto DEFAULT_HOME_ALT_AMSL = 488.0; // altitude AMSL at Irchel Park, Zurich, Switzerland [m]

// international standard atmosphere (troposphere model - valid up to 11km) see [1]
static constexpr auto TEMPERATURE_MSL = 288.15; // temperature at MSL [K] (15 [C])
static constexpr auto PRESSURE_MSL = 101325.0; // pressure at MSL [Pa]
static constexpr auto LAPSE_RATE = 0.0065; // reduction in temperature with altitude for troposphere [K/m]
static constexpr auto AIR_DENSITY_MSL = 1.225; // air density at MSL [kg/m^3]

class GAZEBO_VISIBLE AirspeedPlugin : public SensorPlugin
{
public:
  AirspeedPlugin();
  virtual ~AirspeedPlugin();

protected:
  virtual void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo&);
  virtual void OnSensorUpdate();

private:
  void WindVelocityCallback(WindPtr& msg);

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  sensors::SensorPtr parentSensor_;

  transport::NodePtr node_handle_;
  transport::SubscriberPtr wind_sub_;
  transport::PublisherPtr airspeed_pub_;
  event::ConnectionPtr updateConnection_;
  event::ConnectionPtr updateSensorConnection_;

  common::Time last_time_;
  std::string namespace_;
  std::string link_name_;
  std::string model_name_;
  std::string airspeed_topic_;

  ignition::math::Vector3d wind_vel_; // wind velocity in world frame [m/s]
  ignition::math::Vector3d air_vel_in_body_; // air velocity in body frame [m/s]
  ignition::math::Pose3d veh_pose_in_world_; // vehicle pose in world frame

  std::default_random_engine random_generator_;
  std::normal_distribution<float> standard_normal_distribution_;

  float diff_pressure_stddev_; // [hPa]
  float alt_home_; // home altitude AMSL [m]

};     // class GAZEBO_VISIBLE AirspeedPlugin
}      // namespace gazebo
#endif // _GAZEBO_AIRSPEED_PLUGIN_HH_
