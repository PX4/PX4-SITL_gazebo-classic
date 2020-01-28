/*
 * Copyright (C) 2012-2017 Open Source Robotics Foundation
 * Copyright (C) 2020 PX4 Pro Development Team
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
 * @brief Optical flow mockup Plugin
 *
 * This plugin publishes optical flow messages, but without rendering an image.
 * It assumes a world with only plane ground.
 *
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#ifndef _GAZEBO_OPTICAL_FLOW_MOCKUP_PLUGIN_HH_
#define _GAZEBO_OPTICAL_FLOW_MOCKUP_PLUGIN_HH_

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

#include "OpticalFlow.pb.h"
#include <Range.pb.h>

namespace gazebo
{
class GAZEBO_VISIBLE OpticalFlowMockupPlugin : public ModelPlugin
{
public:
  OpticalFlowMockupPlugin();
  virtual ~OpticalFlowMockupPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo&);

private:
  std::string namespace_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr updateConnection_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr opticalFlow_pub_;
  transport::SubscriberPtr range_sub_;

  sensor_msgs::msgs::OpticalFlow opticalFlow_message_;
  sensor_msgs::msgs::Range distance_measurement_;
  double range_measurement_ {0};
  double integrated_flow_x {0};
  double integrated_flow_y {0};
  double integrated_time {0};

  common::Time last_pub_time_;
  common::Time last_time_;

  double pub_rate_;

  static constexpr double kDefaultPubRate 		= 30.0;	 // [Hz]

  void rangeCallback(const boost::shared_ptr<const sensor_msgs::msgs::Range>& range_msg);



};     // class GAZEBO_VISIBLE OpticalFlowMockupPlugin
}      // namespace gazebo
#endif // _GAZEBO_OPTICAL_FLOW_MOCKUp_PLUGIN_HH_
