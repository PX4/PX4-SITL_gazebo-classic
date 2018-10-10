/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/*
 * Desc: Contact plugin
 * Author: Nate Koenig mod by John Hsu
 */

#include "gazebo/physics/physics.hh"
#include "gazebo_sonar_plugin.h"

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <boost/algorithm/string.hpp>

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(SonarPlugin)

/////////////////////////////////////////////////
SonarPlugin::SonarPlugin()
{
}

/////////////////////////////////////////////////
SonarPlugin::~SonarPlugin()
{
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void SonarPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
//  Get then name of the parent sensor
  this->parentSensor = std::dynamic_pointer_cast<sensors::SonarSensor>(_parent);

  if (!this->parentSensor)
    gzthrow("SonarPlugin requires a Sonar Sensor as its parent");

  this->world = physics::get_world(this->parentSensor->WorldName());

  this->parentSensor->SetActive(false);
  this->newScansConnection = this->parentSensor->ConnectUpdated(boost::bind(&SonarPlugin::OnNewScans, this));
  this->parentSensor->SetActive(true);

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_sonar_plugin] Please specify a robotNamespace.\n";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  const string scopedName = _parent->ParentName();
  string topicName = "~/" + scopedName + "/sonar";
  boost::replace_all(topicName, "::", "/");

  sonar_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Range>(topicName, 10);
}

void SonarPlugin::OnNewScans()
{
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world->SimTime();
#else
  common::Time now = world->GetSimTime();
#endif

  sonar_message.set_time_usec(now.Double() * 1e6);
  sonar_message.set_min_distance(parentSensor->RangeMin());
  sonar_message.set_max_distance(parentSensor->RangeMax());
  sonar_message.set_current_distance(parentSensor->Range());

  sonar_pub_->Publish(sonar_message);
}
