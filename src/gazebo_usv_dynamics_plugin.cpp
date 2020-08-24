/*
 * Copyright (C) 2017  Brian Bingham
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

#include <cmath>
#include <functional>
#include <sstream>
#include <algorithm>

#include <ignition/math/Pose3.hh>

#include "gazebo_usv_dynamics_plugin.h"

#define GRAVITY 9.815

using namespace gazebo;

//////////////////////////////////////////////////
UsvDynamicsPlugin::UsvDynamicsPlugin()
{
}

//////////////////////////////////////////////////
double UsvDynamicsPlugin::SdfParamDouble(sdf::ElementPtr _sdfPtr,
  const std::string &_paramName, const double _defaultVal) const
{
  if (!_sdfPtr->HasElement(_paramName))
  {
    gzmsg << "Parameter <" << _paramName << "> not found: "
                    "Using default value of <" << _defaultVal << ">.";
    return _defaultVal;
  }

  double val = _sdfPtr->Get<double>(_paramName);
  gzdbg << "Parameter found - setting <" << _paramName <<
                  "> to <" << val << ">.";
  return val;
}

//////////////////////////////////////////////////
void UsvDynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzdbg << "Loading usv_gazebo_dynamics_plugin";
  this->world = _model->GetWorld();

  // Get parameters from SDF
  std::string linkName;
  if (!_sdf->HasElement("bodyName") ||
      !_sdf->GetElement("bodyName")->GetValue())
  {
    this->link = _model->GetLink();
    linkName = this->link->GetName();
    gzmsg << "Did not find SDF parameter bodyName";
  }
  else
  {
    linkName = _sdf->GetElement("bodyName")->Get<std::string>();
    this->link = _model->GetLink(linkName);

    gzdbg << "Found SDF parameter bodyName as <" << linkName<< ">";
  }
  if (!this->link)
  {
    gzerr << "usv_gazebo_dynamics_plugin error: bodyName: %s does not exist\n" << linkName.c_str();
    return;
  }
  else
  {
    gzdbg << "USV Dynamics Model Link Name = " << linkName;
  }

  this->waterLevel       = this->SdfParamDouble(_sdf, "waterLevel"  , 0.5);
  this->waterDensity    = this->SdfParamDouble(_sdf, "waterDensity", 997.7735);
  this->paramXdotU       = this->SdfParamDouble(_sdf, "xDotU"       , 5);
  this->paramYdotV       = this->SdfParamDouble(_sdf, "yDotV"       , 5);
  this->paramNdotR       = this->SdfParamDouble(_sdf, "nDotR"       , 1);
  this->paramXu          = this->SdfParamDouble(_sdf, "xU"          , 20);
  this->paramXuu         = this->SdfParamDouble(_sdf, "xUU"         , 0);
  this->paramYv          = this->SdfParamDouble(_sdf, "yV"          , 20);
  this->paramYvv         = this->SdfParamDouble(_sdf, "yVV"         , 0);
  this->paramZw          = this->SdfParamDouble(_sdf, "zW"          , 20);
  this->paramKp          = this->SdfParamDouble(_sdf, "kP"          , 20);
  this->paramMq          = this->SdfParamDouble(_sdf, "mQ"          , 20);
  this->paramNr          = this->SdfParamDouble(_sdf, "nR"          , 20);
  this->paramNrr         = this->SdfParamDouble(_sdf, "nRR"         , 0);
  this->paramHullRadius  = this->SdfParamDouble(_sdf, "hullRadius"    , 0.213);
  this->paramBoatWidth   = this->SdfParamDouble(_sdf, "boatWidth"   , 1.0);
  this->paramBoatLength  = this->SdfParamDouble(_sdf, "boatLength"  , 1.35);
  this->paramLengthN = _sdf->GetElement("length_n")->Get<int>();

  //  Wave model
  // if (_sdf->HasElement("wave_model"))
  // {
  //   this->waveModelName = _sdf->Get<std::string>("wave_model");
  // }
  // this->waveParams = nullptr;

  // Get inertia and mass of vessel
  #if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Vector3d kInertia =
      this->link->GetInertial()->PrincipalMoments();
    const double kMass = this->link->GetInertial()->Mass();
  #else
    const ignition::math::Vector3d kInertia =
      this->link->GetInertial()->GetPrincipalMoments().Ign();
    const double kMass = this->link->GetInertial()->GetMass();
  #endif

  // Report some of the pertinent parameters for verification
  gzdbg << "USV Dynamics Parameters: From URDF XACRO model definition";
  gzdbg << "Vessel Mass (rigid-body): " << kMass;
  gzdbg << "Vessel Inertia Vector (rigid-body): X:" << kInertia[0] <<
                  " Y:" << kInertia[1] << " Z:" << kInertia[2];

  // Initialize time and odometry position
  #if GAZEBO_MAJOR_VERSION >= 8
    this->prevUpdateTime = this->world->SimTime();
  #else
    this->prevUpdateTime = this->world->GetSimTime();
  #endif

  // Listen to the update event broadcastes every physics iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvDynamicsPlugin::Update, this));

  // Initialize Added Mass Matrix
  this->Ma = Eigen::MatrixXd(6, 6);
  this->Ma <<
    this->paramXdotU, 0,                0,   0,   0,   0,
    0,                this->paramYdotV, 0,   0,   0,   0,
    0,                0,                0.1, 0,   0,   0,
    0,                0,                0,   0.1, 0,   0,
    0,                0,                0,   0,   0.1, 0,
    0,                0,                0,   0,   0,   this->paramNdotR;
}

double UsvDynamicsPlugin::CircleSegment(double R, double h)
{
  return R*R*acos( (R-h)/R ) - (R-h)*sqrt(2*R*h-h*h);
}

//////////////////////////////////////////////////
void UsvDynamicsPlugin::Update()
{
  // If we haven't yet, retrieve the wave parameters from ocean model plugin.
  // if (waveParams == nullptr)
  // {
  //   gzmsg << "usv_gazebo_dynamics_plugin: waveParams is null. "
  //         << " Trying to get wave parameters from ocean model" << std::endl;
  //   this->waveParams = WavefieldModelPlugin::GetWaveParams(
  //     this->world, this->waveModelName);
  // }

  #if GAZEBO_MAJOR_VERSION >= 8
    const common::Time kTimeNow = this->world->SimTime();
  #else
    const common::Time kTimeNow = this->world->GetSimTime();
  #endif
  double dt = (kTimeNow - this->prevUpdateTime).Double();
  this->prevUpdateTime = kTimeNow;

  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  #if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Pose3d kPose = this->link->WorldPose();
  #else
    const ignition::math::Pose3d kPose = this->link->GetWorldPose().Ign();
  #endif
  const ignition::math::Vector3d kEuler = kPose.Rot().Euler();

  // Get body-centered linear and angular rates
  #if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Vector3d kVelLinearBody =
      this->link->RelativeLinearVel();
  #else
    const ignition::math::Vector3d kVelLinearBody =
      this->link->GetRelativeLinearVel().Ign();
  #endif
  // gzdbg << "Vel linear: " << kVelLinearBody;

  #if GAZEBO_MAJOR_VERSION >= 8
    const ignition::math::Vector3d kVelAngularBody =
      this->link->RelativeAngularVel();
  #else
    const ignition::math::Vector3d kVelAngularBody =
      this->link->GetRelativeAngularVel().Ign();
  #endif
  // gzdbg << "Vel angular: " << kVelAngularBody;

  // Estimate the linear and angular accelerations.
  // Note the the GetRelativeLinearAccel() and AngularAccel() functions
  // appear to be unreliable
  const ignition::math::Vector3d kAccelLinearBody =
    (kVelLinearBody - this->prevLinVel) / dt;
  this->prevLinVel = kVelLinearBody;
  // gzdbg << "Accel linear: " << kAccelLinearBody;
  const ignition::math::Vector3d kAccelAngularBody =
    (kVelAngularBody - this->prevAngVel) / dt;
  this->prevAngVel = kVelAngularBody;
  // gzdbg << "Accel angular: " << kAccelAngularBody;

  // Create state and derivative of state (accelerations)
  Eigen::VectorXd stateDot = Eigen::VectorXd(6);
  Eigen::VectorXd state    = Eigen::VectorXd(6);
  Eigen::MatrixXd Cmat     = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Dmat     = Eigen::MatrixXd::Zero(6, 6);

  stateDot << kAccelLinearBody.X(), kAccelLinearBody.Y(), kAccelLinearBody.Z(),
    kAccelAngularBody.X(), kAccelAngularBody.Y(), kAccelAngularBody.Z();

  state << kVelLinearBody.X(), kVelLinearBody.Y(), kVelLinearBody.Z(),
    kVelAngularBody.X(), kVelAngularBody.Y(), kVelAngularBody.Z();

  // Added Mass
  const Eigen::VectorXd kAmassVec = -1.0 * this->Ma * stateDot;
  // gzdbg << "stateDot: \n" << stateDot;
  // gzdbg << "amassVec :\n" << kAmassVec;

  // Coriolis - added mass components
  Cmat(0, 5) = this->paramYdotV * kVelLinearBody.Y();
  Cmat(1, 5) = this->paramXdotU * kVelLinearBody.X();
  Cmat(5, 0) = this->paramYdotV * kVelLinearBody.Y();
  Cmat(5, 1) = this->paramXdotU * kVelLinearBody.X();

  // Drag
  Dmat(0, 0) = this->paramXu + this->paramXuu * std::abs(kVelLinearBody.X());
  Dmat(1, 1) = this->paramYv + this->paramYvv * std::abs(kVelLinearBody.Y());
  Dmat(2, 2) = this->paramZw;
  Dmat(3, 3) = this->paramKp;
  Dmat(4, 4) = this->paramMq;
  Dmat(5, 5) = this->paramNr + this->paramNrr * std::abs(kVelAngularBody.Z());
  // gzdbg << "Dmat :\n" << Dmat;
  const Eigen::VectorXd kDvec = -1.0 * Dmat * state;
  // gzdbg << "Dvec :\n" << kDvec;

  // Vehicle frame transform
  ignition::math::Quaterniond vq(kEuler.X(), kEuler.Y(), kEuler.Z());

  ignition::math::Matrix4<double> xformV(vq);

  // Sum all forces - in body frame
  const Eigen::VectorXd kForceSum = kAmassVec + kDvec;

  // Forces in fixed frame
  // gzdbg << "forceSum :\n" << kForceSum;

  // Add dynamic forces/torques to link at CG
  this->link->AddRelativeForce(
    ignition::math::Vector3d(kForceSum(0), kForceSum(1), kForceSum(2)));
  this->link->AddRelativeTorque(
    ignition::math::Vector3d(kForceSum(3), kForceSum(4), kForceSum(5)));

  // Loop over boat grid points
  // Grid point location in boat frame - might be able to precalculate these?
  ignition::math::Vector3d bpnt(0, 0, 0);
  // Grid point location in world frame
  ignition::math::Vector3d bpntW(0, 0, 0);
  // For each hull
  for (int ii = 0; ii < 2; ii++)
  {
  // Grid point in boat frame
  bpnt.Y((ii*2.0-1.0)*this->paramBoatWidth/2.0);
  // For each length segment
    for (int jj = 1; jj <= this->paramLengthN; jj++)
    {
      bpnt.X(((jj - 0.5) / (static_cast<float>(this->paramLengthN)) - 0.5) *
        this->paramBoatLength);

      // Transform from vessel to water/world frame
      bpntW = xformV * bpnt;

      // Debug
      // gzdbg << "[" << ii << "," << jj <<
          // "] grid points" << bpnt.X() << "," << bpnt.Y() << "," << bpnt.Z();
      // gzdbg << "v frame euler " << kEuler;
      // gzdbg << "in water frame" << bpntW.Z() << "," <<
          // bpntW.Y() << "," << bpntW.Z();

      // Vertical location of boat grid point in world frame
      const float kDdz = kPose.Pos().Z() + bpntW.Z();
      // gzdbg << "Z, pose: " << kPose.Pos().Z() << ", bpnt: "
        // << bpntW.Z() << ", dd: " << kDdz;

      // Find vertical displacement of wave field
      // World location of grid point
      ignition::math::Vector3d X;
      X.X() = kPose.Pos().X() + bpntW.Z();
      X.Y() = kPose.Pos().Y() + bpntW.Y();

      // Compute the depth at the grid point.
      double simTime = kTimeNow.Double();
      double depth = 0.0;
      // double depth = WavefieldSampler::ComputeDepthDirectly(
      //  *waveParams, X, simTime);
      // double depth = WavefieldSampler::ComputeDepthSimply(
      //   *waveParams, X, simTime);

      // Vertical wave displacement.
      double dz = depth + X.Z();

      // Total z location of boat grid point relative to water surface
      double  deltaZ = (this->waterLevel + dz) - kDdz;
      deltaZ = std::max(deltaZ, 0.0);  // enforce only upward buoy force
      deltaZ = std::min(deltaZ, this->paramHullRadius);
      // Buoyancy force at grid point
      const float kBuoyForce = CircleSegment(this->paramHullRadius, deltaZ) *
        this->paramBoatLength/(static_cast<float>(this->paramLengthN)) *
        GRAVITY * this->waterDensity;
      // gzdbg << "buoyForce: " << kBuoyForce;

      // Apply force at grid point
      // From web, Appears that position is in the link frame
      // and force is in world frame
      this->link->AddForceAtRelativePosition(
        ignition::math::Vector3d(0, 0, kBuoyForce),
        ignition::math::Vector3d(bpnt.X(), bpnt.Y(), bpnt.Z()));
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(UsvDynamicsPlugin);
