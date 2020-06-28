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
 * @brief LTA Dynamics Plugin
 *
 * This plugin simulates the aerodynamics of an airship.
 * The equations are based on the paper:
 * Airship Dynamics Modeling: A Literature Review
 * Authors: Y. Li, M. Nahon, I. Sharf
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

#include "Wind.pb.h"

namespace gazebo {

/// \brief Plugin class to implement the buoyancy and aerodynamics of a Lighter-Than-Air (LTA) airship.
/// This plugin accepts the following SDF parameters:
///
/// <linkName>: Name of base link for receiving pose and and applying forces.
/// <hullVolume>: The volume of the hull of the airship [m^3].
/// <airDensity>: The density of air [kg/(m^3)].
/// <m11>: The added mass matrix term in row 1 column 1.
/// <m22>: The added mass matrix term in row 2 column 2.
/// <m26>: The added mass matrix term in row 2 column 6.
/// <m33>: The added mass matrix term in row 3 column 3.
/// <m35>: The added mass matrix term in row 3 column 5.
/// <m44>: The added mass matrix term in row 4 column 4.
/// <m53>: The added mass matrix term in row 5 column 3.
/// <m55>: The added mass matrix term in row 5 column 5.
/// <m62>: The added mass matrix term in row 6 column 2.
/// <m66>: The added mass matrix term in row 6 column 6.
/// <distCOV>: The distance from the nose to the Center of Volume (CoV) [m].
/// <distPotentialFlow>: The distance from the nose where the flow ceases to be potential [m].
/// <distFinCenter>: The distance from the nose to the center of a fin [m].
/// <distQuarterChord>: The distance from the center to the quarter chord length of a fin [m].
/// <forceHullInviscidFlowCoeff>: The force coefficient of the inviscid flow contribution of the hull.
/// <forceHullViscousFlowCoeff>: The force coefficient of the viscous flow contribution of the hull.
/// <momentHullInviscidFlowCoeff>: The moment coefficient of the inviscid flow contribution of the hull.
/// <momentHullViscousFlowCoeff>: The moment coefficient of the viscous flow contribution of the hull.
/// <finNormalForceCoeff>: The fin normal force coefficient.
/// <finStallAngle>: The fin stall angle [deg].
/// <axialDragCoeff>: The axial drag coefficient.
/// <windSubTopic>: The topic name to subscribe to wind velocity data.
class GazeboLTADynamicsPlugin : public ModelPlugin {
 public:
  GazeboLTADynamicsPlugin()
      : ModelPlugin() {}

  virtual ~GazeboLTADynamicsPlugin();
  virtual void Publish();
 protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &);

  virtual double Sign(double val);
  virtual ignition::math::Vector3d LocalVelocity(ignition::math::Vector3d lin_vel, ignition::math::Vector3d ang_vel, ignition::math::Vector3d dist);
  virtual double DynamicPressure(ignition::math::Vector3d vec);
  virtual void UpdateForcesAndMoments();

 private:
  std::string namespace_;
  std::string link_name_;

  std::string wind_sub_topic_ = "/wind";
  transport::SubscriberPtr wind_sub_;
  typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;

  // The volume of the hull of the airship [m^3].
  double param_hull_volume_;
  // The density of air [kg/(m^3)].
  double param_air_density_;
  // The added mass matrix term in row 1 column 1.
  double param_m11_;
  // The added mass matrix term in row 2 column 2.
  double param_m22_;
  // The added mass matrix term in row 2 column 6.
  double param_m26_;
  // The added mass matrix term in row 3 column 3.
  double param_m33_;
  // The added mass matrix term in row 3 column 5.
  double param_m35_;
  // The added mass matrix term in row 4 column 4.
  double param_m44_;
  // The added mass matrix term in row 5 column 3.
  double param_m53_;
  // The added mass matrix term in row 5 column 5.
  double param_m55_;
  // The added mass matrix term in row 6 column 2.
  double param_m62_;
  // The added mass matrix term in row 6 column 6.
  double param_m66_;
  // The distance from the nose to the Center of Volume (CoV) [m].
  double param_cov_;
  // The distance from the nose where the flow ceases to be potential [m].
  double param_eps_v_;
  // The distance from the nose to the center of a fin [m].
  double param_dist_fin_x_;
  // The distance from the center to the quarter chord length of a fin [m].
  double param_dist_fin_quarter_chord_;
  // The force coefficient of the inviscid flow contribution of the hull.
  double param_f_hif_coeff_;
  // The force coefficient of the viscous flow contribution of the hull.
  double param_f_hvf_coeff_;
  // The moment coefficient of the inviscid flow contribution of the hull.
  double param_m_hif_coeff_;
  // The moment coefficient of the viscous flow contribution of the hull.
  double param_m_hvf_coeff_;
  // The fin normal force coefficient.
  double param_f_fnf_coeff_;
  // The fin stall angle [deg].
  double param_fin_stall_angle_;
  // The axial drag coefficient.
  double param_axial_drag_coeff_;

  ignition::math::Vector3d wind_;

  transport::NodePtr node_handle_;
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr updateConnection_;

  void WindVelocityCallback(WindPtr &wind);

  boost::thread callback_queue_thread_;
  void QueueThread();
};
}
