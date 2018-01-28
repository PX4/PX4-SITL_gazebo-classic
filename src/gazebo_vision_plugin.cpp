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
  event::Events::DisconnectWorldUpdateBegin(_updateConnection);
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
  _last_time = _world->GetSimTime();
  _last_pub_time = _world->GetSimTime();

  // remember start pose -> VIO should always start with zero
  _pose_model_start = _model->GetWorldPose();

  _nh = transport::NodePtr(new transport::Node());
  _nh->Init(_namespace);

  // Listen to the update event. This event is broadcast every simulation iteration.
  _updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VisionPlugin::OnUpdate, this, _1));

  _pub_odom = _nh->Advertise<odom_msgs::msgs::odom>("~/" + _model->GetName() + "/vision_odom", 10);
}

void VisionPlugin::OnUpdate(const common::UpdateInfo&)
{
  common::Time current_time = _world->GetSimTime();
  double dt = (current_time - _last_pub_time).Double();

  if (dt > 1.0 / _pub_rate) {

    // get pose of the model that the plugin is attached to
    math::Pose pose_model_world = _model->GetWorldPose();
    math::Pose pose_model; // pose in local frame (relative to where it started)
    // convert to local frame (ENU)
    pose_model.pos.x = cos(_pose_model_start.rot.GetYaw()) * (pose_model_world.pos.y - _pose_model_start.pos.y) -
                       sin(_pose_model_start.rot.GetYaw()) * (pose_model_world.pos.x - _pose_model_start.pos.x);
    pose_model.pos.y = cos(_pose_model_start.rot.GetYaw()) * (pose_model_world.pos.x - _pose_model_start.pos.x) +
                       sin(_pose_model_start.rot.GetYaw()) * (pose_model_world.pos.y - _pose_model_start.pos.y);
    pose_model.pos.z = pose_model_world.pos.z - _pose_model_start.pos.z;
    pose_model.rot.SetFromEuler(pose_model_world.rot.GetPitch(),
                                pose_model_world.rot.GetRoll(),
                                pose_model_world.rot.GetYaw() - _pose_model_start.rot.GetYaw());

    // update noise parameters
    math::Vector3 noise;
    math::Vector3 random_walk;
    noise.x = _noise_density * sqrt(dt) * _randn(_rand);
    noise.y = _noise_density * sqrt(dt) * _randn(_rand);
    noise.z = _noise_density * sqrt(dt) * _randn(_rand);
    random_walk.x = _random_walk * sqrt(dt) * _randn(_rand);
    random_walk.y = _random_walk * sqrt(dt) * _randn(_rand);
    random_walk.z = _random_walk * sqrt(dt) * _randn(_rand);

    // bias integration
    _bias.x += random_walk.x * dt - _bias.x / _corellation_time;
    _bias.y += random_walk.y * dt - _bias.y / _corellation_time;
    _bias.z += random_walk.z * dt - _bias.z / _corellation_time;

    // Fill odom msg
    odom_msgs::msgs::odom odom_msg;
    odom_msg.set_usec(current_time.Double() * 1e6);
    odom_msg.set_x(pose_model.pos.x + noise.x + _bias.x);
    odom_msg.set_y(pose_model.pos.y + noise.y + _bias.y);
    odom_msg.set_z(pose_model.pos.z + noise.z + _bias.z);
    odom_msg.set_roll(pose_model.rot.GetRoll());
    odom_msg.set_pitch(pose_model.rot.GetPitch());
    odom_msg.set_yaw(pose_model.rot.GetYaw());

    _last_pub_time = current_time;

    // publish odom msg
    _pub_odom->Publish(odom_msg);
  }
}

} // namespace gazebo
