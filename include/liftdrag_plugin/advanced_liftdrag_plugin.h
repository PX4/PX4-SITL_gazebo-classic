/*
  @author: Karthik Srivatsan
  @version: 1.0

*/

/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_ADVANCED_LIFT_DRAG_PLUGIN_HH_
#define _GAZEBO_ADVANCED_LIFT_DRAG_PLUGIN_HH_

#include <string>
#include <vector>
#include <boost/bind.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>

#include "Wind.pb.h"

namespace gazebo
{
  /// \brief A plugin that simulates lift and drag.
  class GAZEBO_VISIBLE AdvancedLiftDragPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: AdvancedLiftDragPlugin();

    /// \brief Destructor.
    public: ~AdvancedLiftDragPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for World Update events.
    protected: virtual void OnUpdate();

    /// \brief Connection to World Update events.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    protected: physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    protected: physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    protected: physics::ModelPtr model;

    /// \brief coefficient of lift at zero angle of attack
    protected: double CL0;

    /// \brief coefficient of drag at zero lift
    protected: double CD0;

    /// \brief coefficient of pitching moment at zero angle of attack
    protected: double Cem0;

    /// \brief Coefficient of Lift / alpha slope.
    /// Lift = C_L * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    protected: double CLa;

    /// \brief Coefficient of sideforce with respect to angle of attack alpha
    protected: double CYa;

    /// \brief Coefficient of rolling moment with respect to angle of attack alpha
    protected: double Cella;

    /// \brief Coefficient of Moment / alpha slope.
    /// Moment = C_M * q * S
    /// where q (dynamic pressure) = 0.5 * rho * v^2
    protected: double Cema;

    /// \brief Coefficient of yawing moment with respect to angle of attack alpha
    // No memes, please.
    protected: double Cena;

    /// \brief Coefficient of lift with respect to sideslip angle beta.
    protected: double CLb;

    /// \brief Coefficient of sideforce with respect to sideslip angle beta
    protected: double CYb;

    /// \brief Coefficient of roll moment with respect to sideslip angle beta
    protected: double Cellb;

    /// \brief Coefficient of pitching moment with respect to sideslip angle beta.
    protected: double Cemb;

    /// \brief Coefficient of yaw moment with respect to sideslip angle beta
    protected: double Cenb;

    /// \brief Number of control surfaces.
    protected: int num_ctrl_surfaces;

    /// \brief Vectors of control surface deflections.
    protected: std::vector<double> ctrl_surface_direction;

    /// \brief Vectors of control derivatives.
    protected: std::vector<double> CD_ctrl;
    protected: std::vector<double> CY_ctrl;
    protected: std::vector<double> CL_ctrl;
    protected: std::vector<double> Cell_ctrl;
    protected: std::vector<double> Cem_ctrl;
    protected: std::vector<double> Cen_ctrl;

    /// \brief Body rate derivatives
    protected: double CDp;
    protected: double CYp;
    protected: double CLp;
    protected: double Cellp;
    protected: double Cemp;
    protected: double Cenp;

    protected: double CDq;
    protected: double CYq;
    protected: double CLq;
    protected: double Cellq;
    protected: double Cemq;
    protected: double Cenq;

    protected: double CDr;
    protected: double CYr;
    protected: double CLr;
    protected: double Cellr;
    protected: double Cemr;
    protected: double Cenr;

    /// \brief angle of attack when airfoil stalls
    protected: double alphaStall;

    /// \brief Cm-alpha rate after stall
    protected: double CemaStall;

    /// \breif Coefficient of Moment / control surface deflection angle slope
    protected: double Cem_delta;

    /// \brief: \TODO: make a stall velocity curve
    protected: double velocityStall;

    /// \brief air density
    /// at 25 deg C it's about 1.1839 kg/m^3
    /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
    protected: double rho;

    /// \brief if the shape is aerodynamically radially symmetric about
    /// the forward direction. Defaults to false for wing shapes.
    /// If set to true, the upward direction is determined by the
    /// angle of attack.
    protected: bool radialSymmetry;

    /// \brief effective planeform surface area
    protected: double area;

    /// \brief aspect ratio
    protected: double AR;

    /// \brief wing efficiency
    protected: double eff;

    /// \brief angle of attack
    protected: double alpha;

    /// \brief sideslip angle
    protected: double beta;

    /// \brief sigmoid blending parameter
    protected: double M;

    /// \brief aerodynamic reference point in link local coordinates
    // This is the point about which AVL calculates its forces and moments
    protected: ignition::math::Vector3d ref_pt;

    /// \brief Normally, this is taken as a direction parallel to the chord
    /// of the airfoil in zero angle of attack forward flight.
    protected: ignition::math::Vector3d forward;

    /// \brief A vector in the lift/drag plane, perpendicular to the forward
    /// vector.
    protected: ignition::math::Vector3d upward;

    /// \brief Pointer to link currently targeted by mud joint.
    protected: physics::LinkPtr link;

    /// \brief Pointer to joints that actuates a control surface for
    /// this lifting body
    protected: std::vector<physics::JointPtr> controlJoints;

    /// \brief SDF for this plugin;
    protected: sdf::ElementPtr sdf;

    private: void WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg);

    private: transport::NodePtr node_handle_;
    private: transport::SubscriberPtr wind_sub_;
    private: transport::PublisherPtr lift_force_pub_;
    private: common::Time last_pub_time;
    private: msgs::Factory msg_factory_;
    private: std::string namespace_;
    private: std::string wind_sub_topic_ = "world_wind";
    private: ignition::math::Vector3d wind_vel_;
  };
}
#endif
