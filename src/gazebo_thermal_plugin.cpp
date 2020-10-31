/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gazebo_thermal_plugin.h"
#include "common.h"

namespace gazebo {

GazeboThermalPlugin::~GazeboThermalPlugin() {
  update_connection_->~Connection();
}

void GazeboThermalPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  getSdfParam<std::string>(_sdf, "windPubTopic", wind_pub_topic_, "~/" + wind_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);

  getSdfParam<ignition::math::Vector3d>(_sdf, "thermalCenter", thermal_center_, thermal_center_);
  getSdfParam<double>(_sdf, "thermalRadius", thermal_radius_, thermal_radius_);
  getSdfParam<double>(_sdf, "thermalStrength", thermal_strength_, thermal_strength_);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboThermalPlugin::OnUpdate, this, _1));

  wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + model_->GetName() + "/wind", 10);
}

// This gets called by the world update start event.
void GazeboThermalPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d pos = (model_->WorldPose()).Pos();
#else
  ignition::math::Vector3d pos = (model_->GetWorldPose()).Pos();
#endif

  // Calculate the thermal wind velocity at position
  ignition::math::Vector3d thermal_center_vec = pos - thermal_center_;
  double thermal_updraft = thermal_strength_ * std::exp( - (thermal_center_vec).SquaredLength() / (thermal_radius_ * thermal_radius_));
  gazebo::msgs::Vector3d* wind_v = new gazebo::msgs::Vector3d();
  wind_v->set_x(0.0);
  wind_v->set_y(0.0);
  wind_v->set_z(thermal_updraft);

  wind_msg.set_frame_id(frame_id_);
  wind_msg.set_time_usec(now.Double() * 1e6);
  wind_msg.set_allocated_velocity(wind_v);

  wind_pub_->Publish(wind_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboThermalPlugin);
}
