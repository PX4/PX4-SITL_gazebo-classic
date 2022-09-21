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
 * @brief AirflowSensor Plugin
 *
 * This plugin publishes Airflow sensor data
 *
 * @author Henry Kotze <henry@flycloudline.com>
 */

#ifndef _GAZEBO_AIRFLOWSENSOR_PLUGIN_HH_
#define _GAZEBO_AIRFLOWSENSOR_PLUGIN_HH_

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

class GAZEBO_VISIBLE AirflowSensorPlugin : public SensorPlugin
{
public:
  AirflowSensorPlugin();
  virtual ~AirflowSensorPlugin();

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
  transport::PublisherPtr airflow_sensor_pub_;
  event::ConnectionPtr updateSensorConnection_;
  event::ConnectionPtr updateConnection_;

  // linear velocity
  ignition::math::Vector3d vel_a_;
  // angular velocity
  ignition::math::Vector3d ang_a_;

  common::Time last_time_;
  std::string namespace_;
  std::string link_name_;
  std::string model_name_;
  std::string airflowsensor_topic_;

  std::normal_distribution<float> gauss_dir_;
  std::normal_distribution<float> gauss_speed_;
  std::default_random_engine generator_;
  
  ignition::math::Vector3d wind_;
  ignition::math::Vector3d body_wind_;
  ignition::math::Vector3d body_vel_;
  float airflow_direction_;
  float airflow_speed_;

};     // class GAZEBO_VISIBLE AirflowSensorPlugin
}      // namespace gazebo
#endif // _GAZEBO_AIRFLOWSENSOR_PLUGIN_HH_
