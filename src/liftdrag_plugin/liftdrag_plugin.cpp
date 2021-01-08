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

#include <algorithm>
#include <string>

#include "common.h"
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "liftdrag_plugin/liftdrag_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(LiftDragPlugin)

/////////////////////////////////////////////////
LiftDragPlugin::LiftDragPlugin() : cla(1.0), cda(0.01), cma(0.0), rho(1.2041)
{
  this->cp = ignition::math::Vector3d(0, 0, 0);
  this->forward = ignition::math::Vector3d(1, 0, 0);
  this->upward = ignition::math::Vector3d(0, 0, 1);
  this->wind_vel_ = ignition::math::Vector3d(0.0, 0.0, 0.0);
  this->area = 1.0;
  this->alpha0 = 0.0;
  this->cd_alpha0 = 0.0;
  this->cm_alpha0 = 0.0;

  this->alpha = 0.0;
  this->velocityStall = 0.0;

  // 90 deg stall
  this->alphaStall = 0.5*M_PI;
  this->claStall = 0.0;

  this->radialSymmetry = false;

  /// \TODO: what's flat plate drag?
  this->cdaStall = 1.0;
  this->cmaStall = 0.0;

  /// how much to change CL, CD and CM per every radian of the control joint value
  this->cl_delta = 4.0;
  this->cd_delta = 0.0;
  this->cm_delta = 0.0;
}

/////////////////////////////////////////////////
LiftDragPlugin::~LiftDragPlugin()
{
}

/////////////////////////////////////////////////
void LiftDragPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "LiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "LiftDragPlugin world pointer is NULL");

#if GAZEBO_MAJOR_VERSION >= 9
  this->physics = this->world->Physics();
#else
  this->physics = this->world->GetPhysicsEngine();
#endif
  GZ_ASSERT(this->physics, "LiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "LiftDragPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("alpha0"))
    this->alpha0 = _sdf->Get<double>("alpha0");
  else if (_sdf->HasElement("a0")) {
    // a0 is deprecated, but still allowed
    this->alpha0 = _sdf->Get<double>("a0");
  }

  if (_sdf->HasElement("cd_alpha0"))
    this->cd_alpha0 = _sdf->Get<double>("cd_alpha0");

  if (_sdf->HasElement("cm_alpha0"))
    this->cm_alpha0 = _sdf->Get<double>("cm_alpha0");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->cmaStall = _sdf->Get<double>("cma_stall");

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
  this->upward.Normalize();

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    // GZ_ASSERT(this->link, "Link was NULL");

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The LiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&LiftDragPlugin::OnUpdate, this));
    }


  }

  if (_sdf->HasElement("robotNamespace"))
  {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_liftdrag_plugin] Please specify a robotNamespace.\n";
  }
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("windSubTopic")){
    this->wind_sub_topic_ = _sdf->Get<std::string>("windSubTopic");
    wind_sub_ = node_handle_->Subscribe("~/" + wind_sub_topic_, &LiftDragPlugin::WindVelocityCallback, this);
  }

  if (_sdf->HasElement("control_joint_name"))
  {
    std::string controlJointName = _sdf->Get<std::string>("control_joint_name");
    this->controlJoint = this->model->GetJoint(controlJointName);
    if (!this->controlJoint)
    {
      gzerr << "Joint with name[" << controlJointName << "] does not exist.\n";
    }
  }

  if (_sdf->HasElement("cl_delta"))
    this->cl_delta = _sdf->Get<double>("cl_delta");
  else if (_sdf->HasElement("control_joint_rad_to_cl")) {
    // control_joint_rad_to_cl is deprecated, but still allowed
    this->cl_delta = _sdf->Get<double>("control_joint_rad_to_cl");
  }

  if (_sdf->HasElement("cd_delta"))
    this->cd_delta = _sdf->Get<double>("cd_delta");

  if (_sdf->HasElement("cm_delta"))
    this->cm_delta = _sdf->Get<double>("cm_delta");
}

/////////////////////////////////////////////////
void LiftDragPlugin::OnUpdate()
{
  GZ_ASSERT(this->link, "Link was NULL");
  // get linear velocity at cp in inertial frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp) - wind_vel_;
#else
  ignition::math::Vector3d vel = ignitionFromGazeboMath(this->link->GetWorldLinearVel(this->cp)) - wind_vel_;
#endif
  ignition::math::Vector3d velI = vel;
  velI.Normalize();

  if (vel.Length() <= 0.01)
    return;

  // pose of body
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose = this->link->WorldPose();
#else
  ignition::math::Pose3d pose = ignitionFromGazeboMath(this->link->GetWorldPose());
#endif

  // rotate forward and upward vectors into inertial frame
  ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);

  if (forwardI.Dot(vel) <= 0.0){
    // Only calculate lift or drag if the wind relative velocity is in the same direction
    return;
  }

  ignition::math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    ignition::math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
  ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();


  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;

  // get direction of drag
  ignition::math::Vector3d dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  ignition::math::Vector3d liftDirection = spanwiseI.Cross(velInLDPlane);
  liftDirection.Normalize();

  // get direction of moment
  ignition::math::Vector3d momentDirection = spanwiseI;

  // compute angle between upwardI and liftDirection
  // in general, given vectors a and b:
  //   cos(alpha) = a.Dot(b)/(a.Length()*b.Lenghth())
  // given upwardI and liftDirection are both unit vectors, we can drop the denominator
  //   cos(alpha) = a.Dot(b)
  double cosAlpha = ignition::math::clamp(liftDirection.Dot(upwardI), -1.0, 1.0);

  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftDirection is in the same direction as forwardI?
  if (liftDirection.Dot(forwardI) >= 0.0)
    this->alpha = this->alpha0 + acos(cosAlpha);
  else
    this->alpha = this->alpha0 - acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;

  // compute dynamic pressure
  double speedInLDPlane = velInLDPlane.Length();
  double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  // determine controlAngle
  double controlAngle = 0;
  if (this->controlJoint)
  {
#if GAZEBO_MAJOR_VERSION >= 9
    controlAngle = this->controlJoint->Position(0);
#else
    controlAngle = this->controlJoint->GetAngle(0).Radian();
#endif
  }

  // compute cl at cp, check for stall
  double cl;
  if (this->alpha > this->alphaStall)
  {
    cl = this->cla * this->alphaStall +
         this->claStall * (this->alpha - this->alphaStall);
    // make sure cl is still great than 0
    cl = std::max(0.0, cl);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cl = -this->cla * this->alphaStall +
         this->claStall * (this->alpha + this->alphaStall);
    // make sure cl is still less than 0
    cl = std::min(0.0, cl);
  }
  else
  {
    cl = this->cla * this->alpha;
  }

  // modify cl per control joint value
  cl = cl + this->cl_delta * controlAngle;

  // compute lift force at cp
  ignition::math::Vector3d lift = cl * q * this->area * liftDirection;

  // compute cd at cp, check for stall
  double cd;
  if (fabs(this->alpha) > this->alphaStall)
  {
    cd = this->cd_alpha0 + this->cda * this->alphaStall +
         this->cdaStall * (fabs(this->alpha) - this->alphaStall);
  }
  else
  {
    cd = this->cd_alpha0 + this->cda * fabs(this->alpha);
  }

  // make sure drag is positive
  cd = fabs(cd);

  // Take into account the effect of control surface deflection angle to Cd. Contribution must be positive.
  cd += fabs(this->cd_delta * controlAngle);

  // drag at cp
  ignition::math::Vector3d drag = cd * q * this->area * dragDirection;

  // compute cm at cp, check for stall
  double cm;
  if (this->alpha > this->alphaStall)
  {
    cm = this->cm_alpha0 + this->cma * this->alphaStall +
         this->cmaStall * (this->alpha - this->alphaStall);
    // Past stall, cm_alpha tends to drop sharply, so it cannot be larger than cm_alpha0
    // See e.g. https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-01-unified-engineering-i-ii-iii-iv-fall-2005-spring-2006/fluid-mechanics/f19_fall.pdf, page 3
    cm = std::min(this->cm_alpha0, cm);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cm = this->cm_alpha0 - this->cma * this->alphaStall +
         this->cmaStall * (this->alpha + this->alphaStall);
    // make sure cm is still less than 0
    cm = std::max(this->cm_alpha0, cm);
  }
  else
  {
    cm = this->cm_alpha0 + this->cma * this->alpha;
  }

  // Take into account the effect of control surface deflection angle to Cm
  cm += this->cm_delta * controlAngle;

  // compute moment (torque) at cp
  ignition::math::Vector3d moment = cm * q * this->area * momentDirection;

  // force about cg in inertial frame
  ignition::math::Vector3d force = lift + drag;

  // debug
  //
  // if ((this->link->GetName() == "wing_1" ||
  //      this->link->GetName() == "wing_2") &&
  //     (vel.Length() > 50.0 &&
  //      vel.Length() < 50.0))
  if (0)
  {
    gzdbg << "=============================\n";
    gzdbg << "sensor: [" << this->GetHandle() << "]\n";
    gzdbg << "Link: [" << this->link->GetName()
          << "] pose: [" << pose
          << "] dynamic pressure: [" << q << "]\n";
    gzdbg << "spd: [" << vel.Length()
          << "] vel: [" << vel << "]\n";
    gzdbg << "LD plane spd: [" << velInLDPlane.Length()
          << "] vel : [" << velInLDPlane << "]\n";
    gzdbg << "forward (inertial): " << forwardI << "\n";
    gzdbg << "upward (inertial): " << upwardI << "\n";
    gzdbg << "lift dir (inertial): " << liftDirection << "\n";
    gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
    gzdbg << "alpha: " << this->alpha << "\n";
    gzdbg << "lift: " << lift << "\n";
    gzdbg << "drag: " << drag << " cd: "
          << cd << " cda: " << this->cda << "\n";
    gzdbg << "moment: " << moment << "\n";
    gzdbg << "force: " << force << "\n";
    gzdbg << "moment: " << moment << "\n";
  }

  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  moment.Correct();

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, this->cp);
  this->link->AddTorque(moment);
}

void LiftDragPlugin::WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg) {
  wind_vel_ = ignition::math::Vector3d(msg->velocity().x(),
            msg->velocity().y(),
            msg->velocity().z());
}
