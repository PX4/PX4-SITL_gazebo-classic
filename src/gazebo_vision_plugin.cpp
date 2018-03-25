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
  * @brief Vision Plugin
  *
  * This plugin simulates VIO data
  *
  * @author Christoph Tobler <christoph@px4.io>
  */

#include <gazebo_vision_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(VisionPlugin)

VisionPlugin::VisionPlugin() : ModelPlugin()
{
}

VisionPlugin::~VisionPlugin()
{
  _updateConnection->~Connection();
}

void VisionPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  _namespace.clear();
  if (sdf->HasElement("robotNamespace")) {
    _namespace = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_vision_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("pub_rate")) {
    _pub_rate = sdf->GetElement("pub_rate")->Get<int>();
  } else {
    _pub_rate = DEFAULT_PUB_RATE;
    gzerr << "[gazebo_vision_plugin] Using default publication rate of " << DEFAULT_PUB_RATE << " Hz\n";
  }

  if (sdf->HasElement("corellation_time")) {
    _corellation_time = sdf->GetElement("corellation_time")->Get<float>();
  } else {
    _corellation_time = DEFAULT_CORRELATION_TIME;
    gzerr << "[gazebo_vision_plugin] Using default correlation time of " << DEFAULT_CORRELATION_TIME << " s\n";
  }

  if (sdf->HasElement("random_walk")) {
    _random_walk = sdf->GetElement("random_walk")->Get<float>();
  } else {
    _random_walk = DEFAULT_RANDOM_WALK;
    gzerr << "[gazebo_vision_plugin] Using default random walk of " << DEFAULT_RANDOM_WALK << " (m/s) / sqrt(hz)\n";
  }

  if (sdf->HasElement("noise_density")) {
    _noise_density = sdf->GetElement("noise_density")->Get<float>();
  } else {
    _noise_density = DEFAULT_NOISE_DENSITY;
    gzerr << "[gazebo_vision_plugin] Using default noise density of " << DEFAULT_NOISE_DENSITY << " (m) / sqrt(hz)\n";
  }
}

void VisionPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);

  // Store model
  _model = model;

  _world = _model->GetWorld();
#if GAZEBO_MAJOR_VERSION >= 9
  _last_time = _world->SimTime();
  _last_pub_time = _world->SimTime();
  // remember start pose -> VIO should always start with zero
  _pose_model_start = _model->WorldPose();
#else
  _last_time = _world->GetSimTime();
  _last_pub_time = _world->GetSimTime();
  // remember start pose -> VIO should always start with zero
  _pose_model_start = ignitionFromGazeboMath(_model->GetWorldPose());
#endif


  _nh = transport::NodePtr(new transport::Node());
  _nh->Init(_namespace);

  // Listen to the update event. This event is broadcast every simulation iteration.
  _updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VisionPlugin::OnUpdate, this, _1));

  _pub_odom = _nh->Advertise<odom_msgs::msgs::odom>("~/" + _model->GetName() + "/vision_odom", 10);
}

void VisionPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = _world->SimTime();
#else
  common::Time current_time = _world->GetSimTime();
#endif
  double dt = (current_time - _last_pub_time).Double();

  if (dt > 1.0 / _pub_rate) {

    // get pose of the model that the plugin is attached to
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d pose_model_world = _model->WorldPose();
#else
    ignition::math::Pose3d pose_model_world = ignitionFromGazeboMath(_model->GetWorldPose());
#endif
    ignition::math::Pose3d pose_model; // pose in local frame (relative to where it started)
    // convert to local frame (ENU)
    pose_model.Pos().X() = cos(_pose_model_start.Rot().Yaw()) * (pose_model_world.Pos().Y() - _pose_model_start.Pos().Y()) -
                       sin(_pose_model_start.Rot().Yaw()) * (pose_model_world.Pos().X() - _pose_model_start.Pos().X());
    pose_model.Pos().Y() = cos(_pose_model_start.Rot().Yaw()) * (pose_model_world.Pos().X() - _pose_model_start.Pos().X()) +
                       sin(_pose_model_start.Rot().Yaw()) * (pose_model_world.Pos().Y() - _pose_model_start.Pos().Y());
    pose_model.Pos().Z() = pose_model_world.Pos().Z() - _pose_model_start.Pos().Z();
    pose_model.Rot().Euler(pose_model_world.Rot().Pitch(),
                           pose_model_world.Rot().Roll(),
                           pose_model_world.Rot().Yaw() - _pose_model_start.Rot().Yaw());

    // update noise parameters
    ignition::math::Vector3d noise;
    ignition::math::Vector3d random_walk;
    noise.X() = _noise_density * sqrt(dt) * _randn(_rand);
    noise.Y() = _noise_density * sqrt(dt) * _randn(_rand);
    noise.Z() = _noise_density * sqrt(dt) * _randn(_rand);
    random_walk.X() = _random_walk * sqrt(dt) * _randn(_rand);
    random_walk.Y() = _random_walk * sqrt(dt) * _randn(_rand);
    random_walk.X() = _random_walk * sqrt(dt) * _randn(_rand);

    // bias integration
    _bias.X() += random_walk.X() * dt - _bias.X() / _corellation_time;
    _bias.Y() += random_walk.Y() * dt - _bias.Y() / _corellation_time;
    _bias.Z() += random_walk.Z() * dt - _bias.Z() / _corellation_time;

    // Fill odom msg
    odom_msgs::msgs::odom odom_msg;
    odom_msg.set_usec(current_time.Double() * 1e6);
    odom_msg.set_x(pose_model.Pos().X() + noise.X() + _bias.X());
    odom_msg.set_y(pose_model.Pos().Y() + noise.Y() + _bias.Y());
    odom_msg.set_z(pose_model.Pos().Z() + noise.Z() + _bias.Y());
    odom_msg.set_roll(pose_model.Rot().Roll());
    odom_msg.set_pitch(pose_model.Rot().Pitch());
    odom_msg.set_yaw(pose_model.Rot().Yaw());

    _last_pub_time = current_time;

    // publish odom msg
    _pub_odom->Publish(odom_msg);
  }
}

} // namespace gazebo
