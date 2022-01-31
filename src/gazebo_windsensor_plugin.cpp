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
 * @brief WindSensor Plugin
 *
 * This plugin publishes Windsensor
 *
 * @author Henry Kotze <henry@flycloudline.com>
 */

#include <gazebo_windsensor_plugin.h>
#include <boost/algorithm/string.hpp>

namespace gazebo {
GZ_REGISTER_SENSOR_PLUGIN(WindSensorPlugin)

WindSensorPlugin::WindSensorPlugin() : SensorPlugin(),
  wind_direction_(0.0),
  wind_speed_(0.0f)
{ }

WindSensorPlugin::~WindSensorPlugin()
{
    if (updateConnection_)
      updateConnection_->~Connection();
}

void WindSensorPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  this->parentSensor_ = std::dynamic_pointer_cast<sensors::Sensor>(_parent);
  if (!parentSensor_)
    gzthrow("WindSensorPlugin requires a Wind Sensor as its parent");

  // Get the root model name
  const std::string scopedName = _parent->ParentName();
  link_name_ = scopedName;
  std::vector<std::string> names_splitted;
  boost::split(names_splitted, scopedName, boost::is_any_of("::"));
  names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted),
                            [](const std::string& name)
                            { return name.size() == 0; }), end(names_splitted));
  const std::string rootModelName = names_splitted.front(); // The first element is the name of the root model
  // the second to the last name is the model name
  const std::string parentSensorModelName = names_splitted.rbegin()[1];

  // store the model name
  model_name_ = names_splitted[0];

  // get windsensor topic name
  if(_sdf->HasElement("topic")) {
    windsensor_topic_ = _sdf->GetElement("topic")->Get<std::string>();
  } else {
    // if not set by parameter, get the topic name from the model name
    windsensor_topic_ = parentSensorModelName;
    gzwarn << "[gazebo_windsensor_plugin]: " + names_splitted.front() + "::" + names_splitted.rbegin()[1] +
      " using windsensor topic \"" << parentSensorModelName << "\"\n";
  }


  // Store the pointer to the model.
  world_ = physics::get_world(parentSensor_->WorldName());

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_windsensor_plugin] Please specify a robotNamespace.\n";
  }

  this->node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  this->parentSensor_->SetUpdateRate(10.0);
  this->parentSensor_->SetActive(false);
  updateSensorConnection_ = parentSensor_->ConnectUpdated(boost::bind(&WindSensorPlugin::OnSensorUpdate, this));
  this->parentSensor_->SetActive(true);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&WindSensorPlugin::OnUpdate, this, _1));

  windsensor_pub_ = node_handle_->Advertise<sensor_msgs::msgs::WindSensor>("~/" + model_name_ + "/link/" + windsensor_topic_, 10);
  wind_sub_ = node_handle_->Subscribe("~/world_wind", &WindSensorPlugin::WindVelocityCallback, this);


}

void WindSensorPlugin::OnUpdate(const common::UpdateInfo&){

  #if GAZEBO_MAJOR_VERSION >= 9
  model_ = world_->ModelByName(model_name_);
  physics::EntityPtr parentEntity = world_->EntityByName(link_name_);
#else
  model_ = world_->GetModel(model_name_);
  physics::EntityPtr parentEntity = world_->GetEntity(link_name_);
#endif
  link_ = boost::dynamic_pointer_cast<physics::Link>(parentEntity);
  if (link_ == NULL)
    gzthrow("[gazebo_airspeed_plugin] Couldn't find specified link \"" << link_name_ << "\".");

#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose();
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose());
#endif
  ignition::math::Quaterniond C_W_I = T_W_I.Rot();

#if GAZEBO_MAJOR_VERSION >= 9
  vel_a_ = link_->RelativeLinearVel() - C_W_I.RotateVectorReverse(wind_);
  ang_a_ = link_->RelativeLinearVel() - C_W_I.RotateVectorReverse(wind_);
#else
  vel_a_ = ignitionFromGazeboMath(link_->GetRelativeLinearVel()) - C_W_I.RotateVectorReverse(wind_);
  ang_a_ = ignitionFromGazeboMath(link_->GetRelativeAngularVel());
#endif

  last_time_ = current_time;

  wind_direction_ = atan2(vel_a_.X(), vel_a_.Y());
  wind_speed_ =  vel_a_.Length();
}

void WindSensorPlugin::OnSensorUpdate() {


  sensor_msgs::msgs::WindSensor windsensor_msg;
  windsensor_msg.set_time_usec(last_time_.Double() * 1e6);
  windsensor_msg.set_wind_speed(wind_speed_);
  windsensor_msg.set_wind_direction(wind_direction_);
  windsensor_pub_->Publish(windsensor_msg);

}

void WindSensorPlugin::WindVelocityCallback(WindPtr& wind) {

  // Get world wind velocity.
  ignition::math::Vector3d wind_world = ignition::math::Vector3d(wind->velocity().x(), wind->velocity().y(), wind->velocity().z());

  // Rotate to body frame
  #if GAZEBO_MAJOR_VERSION >= 9
    wind_ = link_->WorldPose().Rot().Inverse().RotateVector(wind_world);
  #else
    wind_ = ignitionFromGazeboMath(link_->GetWorldPose()).Rot().Inverse().RotateVector(wind_world);
  #endif

}
} // namespace gazebo
