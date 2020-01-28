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
 * This plugin publishes optical flow messages, but without rendering an image.
 * It assumes a world with only plane ground.
 *
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gazebo_opticalflow_mockup_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(OpticalFlowMockupPlugin)

OpticalFlowMockupPlugin::OpticalFlowMockupPlugin() : ModelPlugin()
{
}

OpticalFlowMockupPlugin::~OpticalFlowMockupPlugin()
{
    if (updateConnection_)
      updateConnection_->~Connection();
}

void OpticalFlowMockupPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();
#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  last_pub_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
  last_pub_time_ = world_->GetSimTime();
#endif

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
  }

if (_sdf->HasElement("pubRate")) {
    pub_rate_ = _sdf->GetElement("pubRate")->Get<int>();
  } else {
    pub_rate_ = kDefaultPubRate;
    gzwarn << "[gazebo_vision_plugin] Using default publication rate of " << pub_rate_ << " Hz\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  range_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + "/link/lidar", &OpticalFlowMockupPlugin::rangeCallback, this);
  opticalFlow_pub_ = node_handle_->Advertise<sensor_msgs::msgs::OpticalFlow>("~/" + model_->GetName() + "/px4flow/link/opticalFlow", 10);

    // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&OpticalFlowMockupPlugin::OnUpdate, this, _1));
}

void OpticalFlowMockupPlugin::OnUpdate(const common::UpdateInfo&)
{
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif


    double dt = (current_time - last_time_).Double();

  // get pose of the model that the plugin is attached to
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d pose_model_world = model_->WorldPose();
    ignition::math::Vector3d velocity_model_world = model_->WorldLinearVel();
    ignition::math::Vector3d angular_velocity_model = model_->RelativeAngularVel();
#else
    ignition::math::Pose3d pose_model_world = ignitionFromGazeboMath(model_->GetWorldPose());
    ignition::math::Vector3d velocity_model_world = ignitionFromGazeboMath(model_->GetWorldLinearVel());
    ignition::math::Vector3d angular_velocity_model = ignitionFromGazeboMath(model_->GetRelativeAngularVel());
#endif
    // Compute velocities in body FRD frame
    ignition::math::Vector3d angular_velocity = q_br.Inverse().RotateVector(angular_velocity_model);
    ignition::math::Vector3d linear_velocity = q_br.Inverse().RotateVector(
                                         pose_model_world.Rot().Inverse().RotateVector(velocity_model_world));

  // Compute flow
    float flow_x_ang = angular_velocity.X() - linear_velocity.Y() / range_measurement_;
    float flow_y_ang = angular_velocity.Y() + linear_velocity.X() / range_measurement_;


  // Integrate flow
  integrated_time += dt;
  integrated_flow_x += flow_x_ang * dt;
  integrated_flow_y += flow_y_ang * dt;

  double dt_pub = (current_time - last_pub_time_).Double();

  if (dt_pub > 1.0 / pub_rate_) {

    // Fill message
    opticalFlow_message_.set_time_usec(current_time.Double() * 1e6);
    opticalFlow_message_.set_sensor_id(2.0);
    opticalFlow_message_.set_integration_time_us(integrated_time * 1e6);
    opticalFlow_message_.set_integrated_x(integrated_flow_x);
    opticalFlow_message_.set_integrated_y(integrated_flow_y);
    opticalFlow_message_.set_integrated_xgyro(NAN);
    opticalFlow_message_.set_integrated_ygyro(NAN);
    opticalFlow_message_.set_integrated_zgyro(NAN);
    opticalFlow_message_.set_temperature(20.0f);
    opticalFlow_message_.set_quality(180.0f);
    opticalFlow_message_.set_time_delta_distance_us(0);
    opticalFlow_message_.set_distance(0.0f); //get real values in gazebo_mavlink_interface.cpp
    //send message
    opticalFlow_pub_->Publish(opticalFlow_message_);

    // Reset integrators
    integrated_flow_x = 0;
    integrated_flow_y = 0;
    integrated_time = 0;
    last_pub_time_ = current_time;
  }

  last_time_ = current_time;
}

void OpticalFlowMockupPlugin::rangeCallback(const boost::shared_ptr<const sensor_msgs::msgs::Range>& range_msg)
{
  range_measurement_ = range_msg->current_distance();  //[m]
}


} // namespace gazebo
