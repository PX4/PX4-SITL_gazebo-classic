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
 * @brief LTA Aerodynamics Plugin
 *
 * This plugin simulates the aerodynamics of an airship.
 * The equations are based on the paper:
 * Dynamic Modelling of the Airship with MATLAB using Geometric Aerodynamic Parameters
 * Authors: M.Z. Ashraf, M.A. Choudhry
 *
 * @author Anton Erasmus <anton@flycloudline.com>
 */

#include <stdio.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "common.h"

namespace gazebo {

class GazeboLTAAerodynamicsPlugin : public ModelPlugin {
 public:
  GazeboLTAAerodynamicsPlugin()
      : ModelPlugin() {}

  virtual ~GazeboLTAAerodynamicsPlugin();
  virtual void Publish();
 protected:
  virtual ignition::math::Vector3d TransformNED2XYZ(ignition::math::Vector3d vec);
  virtual ignition::math::Vector3d TransformXYZ2NED(ignition::math::Vector3d vec);
  virtual void UpdateForcesAndMoments();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);

 private:
  std::string namespace_;
  std::string link_name_;

  float air_density_;

  float length_;
  float diameter_;

  float m11_;
  float m22_;
  float m26_;
  float m33_;
  float m35_;
  float m44_;
  float m53_;
  float m55_;
  float m62_;
  float m66_;

  float cx1_;
  float cx2_;
  float cx3_;
  float cy1_;
  float cy2_;
  float cy3_;
  float cz1_;
  float cz2_;
  float cz3_;

  float cl2_;
  float cl3_;
  float cl4_;
  float cm1_;
  float cm2_;
  float cm3_;
  float cm5_;
  float cn1_;
  float cn2_;
  float cn3_;
  float cn5_;


  transport::NodePtr node_handle_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
};
}
