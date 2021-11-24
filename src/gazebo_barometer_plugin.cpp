/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @brief Barometer Plugin
 *
 * This plugin simulates barometer data
 *
 * @author Elia Tarasov <elias.tarasov@gmail.com>
 *
 * References in header.
 */

#include <gazebo_barometer_plugin.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(BarometerPlugin)

BarometerPlugin::BarometerPlugin() : ModelPlugin(),
    alt_home_(DEFAULT_HOME_ALT_AMSL),
    baro_rnd_y2_(0.0),
    baro_rnd_use_last_(false),
    baro_drift_pa_per_sec_(0.0),
    baro_drift_pa_(0.0)
{
}

BarometerPlugin::~BarometerPlugin()
{
  update_connection_->~Connection();
}

void BarometerPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  const char *env_alt = std::getenv("PX4_HOME_ALT");
  if (env_alt) {
    alt_home_ = std::stod(env_alt);
    gzmsg << "[gazebo_barometer_plugin] Home altitude is set to " << alt_home_ << " m AMSL.\n";
  } else {
    alt_home_ = DEFAULT_HOME_ALT_AMSL;
  }

  namespace_.clear();
  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_barometer_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("pubRate")) {
    pub_rate_ = sdf->GetElement("pubRate")->Get<unsigned int>();
  } else {
    pub_rate_ = DEFAULT_PUB_RATE;
    gzwarn << "[gazebo_barometer_plugin] Using default publication rate of " << pub_rate_ << " Hz\n";
  }

  if (sdf->HasElement("baroTopic")) {
    baro_topic_ = sdf->GetElement("baroTopic")->Get<std::string>();
  } else {
    baro_topic_ = DEFAULT_BAROMETER_TOPIC;
    gzwarn << "[gazebo_barometer_plugin] Using default barometer topic " << baro_topic_ << "\n";
  }

  if (sdf->HasElement("baroDriftPaPerSec")) {
    baro_drift_pa_per_sec_ = sdf->GetElement("baroDriftPaPerSec")->Get<double>();
  } else {
    baro_drift_pa_per_sec_ = 0.0;
  }
}

void BarometerPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);

  model_ = model;
  world_ = model_->GetWorld();
#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  last_pub_time_ = world_->SimTime();
  pose_model_start_ = model_->WorldPose();
#else
  last_time_ = world_->GetSimTime();
  last_pub_time_ = world_->GetSimTime();
  pose_model_start_ = ignitionFromGazeboMath(model_->GetWorldPose());
#endif

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BarometerPlugin::OnUpdate, this, _1));

  pub_baro_ = node_handle_->Advertise<sensor_msgs::msgs::Pressure>("~/" + model_->GetName() + baro_topic_, 10);

  standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);
  gravity_in_world_ = world_->Gravity();
}

void BarometerPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  const common::Time current_time = world_->SimTime();
#else
  const common::Time current_time = world_->GetSimTime();
#endif
  const double dt = (current_time - last_pub_time_).Double();

  if (dt > 1.0 / pub_rate_) {

    // get pose of the model that the plugin is attached to
#if GAZEBO_MAJOR_VERSION >= 9
    const ignition::math::Pose3d pose_model_world = model_->WorldPose();
#else
    const ignition::math::Pose3d pose_model_world = ignitionFromGazeboMath(model_->GetWorldPose());
#endif

    // calculate temperature at current altitude
    ignition::math::Pose3d pose_model; // Z-component pose in local frame (relative to where it started)
    pose_model.Pos().Z() = pose_model_world.Pos().Z() - pose_model_start_.Pos().Z();
    const float alt_rel = pose_model.Pos().Z(); // Z-component from ENU
    const float alt_amsl = (float)alt_home_ + alt_rel;
    const float temperature_local = TEMPERATURE_MSL - LAPSE_RATE * alt_amsl;

    // calculate absolute pressure at local temperature
    const float pressure_ratio = powf(TEMPERATURE_MSL / temperature_local, 5.256f);
    const float absolute_pressure = PRESSURE_MSL / pressure_ratio;

    // generate Gaussian noise sequence using polar form of Box-Muller transformation
    double y1;
    {
      double x1, x2, w;
      if (!baro_rnd_use_last_) {
        do {
          x1 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
          x2 = 2.0 * standard_normal_distribution_(random_generator_) - 1.0;
          w = x1 * x1 + x2 * x2;
        } while ( w >= 1.0 );
        w = sqrt( (-2.0 * log( w ) ) / w );
        // calculate two values - the second value can be used next time because it is uncorrelated
        y1 = x1 * w;
        baro_rnd_y2_ = x2 * w;
        baro_rnd_use_last_ = true;
      } else {
        // no need to repeat the calculation - use the second value from last update
        y1 = baro_rnd_y2_;
        baro_rnd_use_last_ = false;
      }
    }

    // Apply noise and drift
    const float abs_pressure_noise = 1.0f * (float)y1;  // 1 Pa RMS noise
    baro_drift_pa_ += baro_drift_pa_per_sec_ * dt;
    const float absolute_pressure_noisy = absolute_pressure + abs_pressure_noise + baro_drift_pa_;

    // convert to hPa
    const float absolute_pressure_noisy_hpa = absolute_pressure_noisy * 0.01f;
    baro_msg_.set_absolute_pressure(absolute_pressure_noisy_hpa);

    // calculate air density at local temperature
    const float density_ratio = powf(TEMPERATURE_MSL / temperature_local , 4.256f);
    const float air_density = AIR_DENSITY_MSL / density_ratio;

    // calculate pressure altitude including effect of pressure noise
    baro_msg_.set_pressure_altitude(alt_amsl -
                                    (abs_pressure_noise + baro_drift_pa_) /
                                        (gravity_in_world_.Length() * air_density));

    // calculate temperature in Celsius
    baro_msg_.set_temperature(temperature_local + ABSOLUTE_ZERO_C);

    // Fill baro msg
    baro_msg_.set_time_usec(current_time.Double() * 1e6);

    last_pub_time_ = current_time;

    // Publish baro msg
    pub_baro_->Publish(baro_msg_);
  }
}
}
