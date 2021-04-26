/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @brief Drop Plugin
 *
 * This plugin simulates dropping a payload
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef _GAZEBO_DROP_PLUGIN_HH_
#define _GAZEBO_DROP_PLUGIN_HH_

#include <math.h>
#include <common.h>
#include <sdf/sdf.hh>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include "CommandMotorSpeed.pb.h"


#include <Odometry.pb.h>

namespace gazebo
{

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;


class GAZEBO_VISIBLE DropPlugin : public ModelPlugin
{
public:
  DropPlugin();
  virtual ~DropPlugin();

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  void OnUpdate(const common::UpdateInfo&);
  physics::ModelPtr GetModelPtr(std::string model_name);

private:
  /// \brief Loads the package model
  void LoadPackage();

  /// \brief Callback for subscribing to motor commands
   /// \param rot_velocities Motor command velocity commanded from firmware
  void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);


  std::string namespace_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;

  bool package_spawned = false;  ///< Check if the package has already been spawned in the world
  double max_rot_velocity_ = 3500;   ///< Clip maximum motor velocity
  double ref_motor_rot_vel_ = 0;     ///< Reference motor velocity 
  double release_rot_vel_ = 300; ///< Motor velocity command to release the package
  int motor_number_;

  std::string trigger_sub_topic_ = "/gazebo/command/motor_speed"; ///< Parachute trigger signal topic
  std::string model_name_ = "parachute_package";

  transport::NodePtr node_handle_;
  transport::SubscriberPtr trigger_sub_;

};     // class GAZEBO_VISIBLE DropPlugin
}      // namespace gazebo
#endif // _GAZEBO_DROP_PLUGIN_HH_
