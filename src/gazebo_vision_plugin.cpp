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
  * @brief Vision Plugin
  *
  * This plugin simulates VIO data
  *
  * @author Christoph Tobler <christoph@px4.io>
  */

#include <gazebo_vision_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(VisionPlugin)

VisionPlugin::VisionPlugin() : ModelPlugin()
{
}

VisionPlugin::~VisionPlugin()
{
  _updateConnection->~Connection();
}

void VisionPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  _namespace.clear();
  if (sdf->HasElement("robotNamespace")) {
    _namespace = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_vision_plugin] Please specify a robotNamespace.\n";
  }

  if (sdf->HasElement("pubRate")) {
    _pub_rate = sdf->GetElement("pubRate")->Get<int>();
  } else {
    _pub_rate = kDefaultPubRate;
    gzwarn << "[gazebo_vision_plugin] Using default publication rate of " << _pub_rate << " Hz\n";
  }

  if (sdf->HasElement("corellationTime")) {
    _corellation_time = sdf->GetElement("corellationTime")->Get<float>();
  } else {
    _corellation_time = kDefaultCorrelationTime;
    gzwarn << "[gazebo_vision_plugin] Using default correlation time of " << _corellation_time << " s\n";
  }

  if (sdf->HasElement("randomWalk")) {
    _random_walk = sdf->GetElement("randomWalk")->Get<float>();
  } else {
    _random_walk = kDefaultRandomWalk;
    gzwarn << "[gazebo_vision_plugin] Using default random walk of " << _random_walk << " (m/s) / sqrt(hz)\n";
  }

  if (sdf->HasElement("noiseDensity")) {
    _noise_density = sdf->GetElement("noiseDensity")->Get<float>();
  } else {
    _noise_density = kDefaultNoiseDensity;
    gzwarn << "[gazebo_vision_plugin] Using default noise density of " << _noise_density << " (m) / sqrt(hz)\n";
  }

  if(sdf->HasElement("enableRosOdom")) {
    _enable_ros_odom = sdf->GetElement("enableRosOdom")->Get<bool>();
  } else {
    _enable_ros_odom = kDefaultEnableRosOdom;
    gzwarn << "[gazebo_vision_plugin] Using default ros odometry switch " << _enable_ros_odom << "\n";
  }

#if BUILD_ROS_INTERFACE == 1
  if(sdf->HasElement("rosPoseSubTopic")) {
    _ros_pose_sub_topic = sdf->GetElement("rosPoseSubTopic")->Get<std::string>();
  } else {
    _ros_pose_sub_topic = kDefaultRosPoseTopic;
    gzwarn << "[gazebo_vision_plugin] Using default ROS PoseStamped subscription to topic " << _ros_pose_sub_topic << "\n";
  }

  if(sdf->HasElement("rosPoseCovSubTopic")) {
    _ros_posecov_sub_topic = sdf->GetElement("rosPoseCovSubTopic")->Get<std::string>();
  } else {
    _ros_posecov_sub_topic = kDefaultRosPoseCovTopic;
    gzwarn << "[gazebo_vision_plugin] Using default ROS PoseWithCovarianceStamped subscription to topic " << _ros_posecov_sub_topic << "\n";
  }

  if(sdf->HasElement("rosOdomSubTopic")) {
    _ros_odom_sub_topic = sdf->GetElement("rosOdomSubTopic")->Get<std::string>();
  } else {
    _ros_odom_sub_topic = kDefaultRosOdomTopic;
    gzwarn << "[gazebo_vision_plugin] Using default ROS Odometry subscription to topic " << _ros_odom_sub_topic << "\n";
  }
#endif
}

void VisionPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);

  // Store model
  _model = model;

  _world = _model->GetWorld();
#if GAZEBO_MAJOR_VERSION >= 9
  _last_time = _world->SimTime();
  _last_pub_time = _world->SimTime();
  // remember start pose -> VIO should always start with zero
  _pose_model_start = _model->WorldPose();
#else
  _last_time = _world->GetSimTime();
  _last_pub_time = _world->GetSimTime();
  // remember start pose -> VIO should always start with zero
  _pose_model_start = ignitionFromGazeboMath(_model->GetWorldPose());
#endif

  _nh = transport::NodePtr(new transport::Node());
  _nh->Init(_namespace);

  if(_enable_ros_odom)
  {
#if BUILD_ROS_INTERFACE == 1
  // ROS Topic subscriber
  // Initialize ROS, if it has not already bee initialized.
  if (!ros::isInitialized())  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_ros_sub", ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  this->_ros_node.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so_pose = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(_ros_pose_sub_topic, 1, boost::bind(&VisionPlugin::rosPoseCallBack, this, _1), ros::VoidPtr(), &this->_ros_queue);
  ros::SubscribeOptions so_posecov = ros::SubscribeOptions::create<geometry_msgs::PoseWithCovarianceStamped>(_ros_posecov_sub_topic, 1, boost::bind(&VisionPlugin::rosPoseCovCallBack, this, _1), ros::VoidPtr(), &this->_ros_queue);
  ros::SubscribeOptions so_odom = ros::SubscribeOptions::create<nav_msgs::Odometry>(_ros_odom_sub_topic, 1, boost::bind(&VisionPlugin::rosOdomCallBack, this, _1), ros::VoidPtr(), &this->_ros_queue);

  this->_ros_pose_sub = this->_ros_node->subscribe(so_pose);
  this->_ros_posecov_sub = this->_ros_node->subscribe(so_posecov);
  this->_ros_odom_sub = this->_ros_node->subscribe(so_odom);

  this->_ros_queue_thread = std::thread(std::bind(&VisionPlugin::queueThread, this));
#endif
  } else {
    // Listen to the update event. This event is broadcast every simulation iteration.
    _updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&VisionPlugin::OnUpdate, this, _1));
  }

  _pub_odom = _nh->Advertise<nav_msgs::msgs::Odometry>("~/" + _model->GetName() + "/vision_odom", 10);
}

double VisionPlugin::calcTimeStep() {
#if GAZEBO_MAJOR_VERSION >= 9
  _current_time = _world->SimTime();
#else
  _current_time = _world->GetSimTime();
#endif
  return (_current_time - _last_pub_time).Double();
}

#if BUILD_ROS_INTERFACE == 1
void VisionPlugin::handle_vision_data(double sec, Eigen::Vector3d &p, Eigen::Quaterniond &q, Eigen::Vector3d &lin_vel, Eigen::Vector3d &ang_vel, std::array<double, 36> &pose_cov, std::array<double, 36> &twist_cov)
{
   // Fill odom msg
    odom_msg.set_usec(sec * 1e6); // convert sec to usec

    gazebo::msgs::Vector3d* position = new gazebo::msgs::Vector3d();
    position->set_x(p.x());
    position->set_y(p.y());
    position->set_z(p.z());
    odom_msg.set_allocated_position(position);

    gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
    orientation->set_x(q.x());
    orientation->set_y(q.y());
    orientation->set_z(q.z());
    orientation->set_w(q.w());
    odom_msg.set_allocated_orientation(orientation);

    gazebo::msgs::Vector3d* linear_velocity = new gazebo::msgs::Vector3d();
    linear_velocity->set_x(lin_vel.x());
    linear_velocity->set_y(lin_vel.y());
    linear_velocity->set_z(lin_vel.z());
    odom_msg.set_allocated_linear_velocity(linear_velocity);

    gazebo::msgs::Vector3d* angular_velocity = new gazebo::msgs::Vector3d();
    angular_velocity->set_x(ang_vel.x());
    angular_velocity->set_y(ang_vel.y());
    angular_velocity->set_z(ang_vel.z());
    odom_msg.set_allocated_angular_velocity(angular_velocity);

    for (int i = 0; i < 36; i++) {
      odom_msg.add_pose_covariance(pose_cov[i]);
      odom_msg.add_twist_covariance(twist_cov[i]);
    }

    _last_pub_time = _current_time;

    // publish odom msg
    _pub_odom->Publish(odom_msg);
}

void VisionPlugin::rosPoseCovCallBack(const geometry_msgs::PoseWithCovarianceStampedConstPtr& odomMsg)
{
  double dt = calcTimeStep();

  if (dt > 1.0 / _pub_rate) {

    Eigen::Vector3d position(odomMsg->pose.pose.position.x, odomMsg->pose.pose.position.y, odomMsg->pose.pose.position.z);
    Eigen::Quaterniond orientation(odomMsg->pose.pose.orientation.w, odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y, odomMsg->pose.pose.orientation.z);
    Eigen::Vector3d lin_vel(0, 0, 0); // linear velocity is not provided by sensor
    Eigen::Vector3d ang_vel(0, 0, 0); // angular velocity is not provided by sensor

    std::array<double, 36> pose_covariance{};
    std::array<double, 36> twist_covariance{}; // all values are zero
    std::copy(odomMsg->pose.covariance.begin(), odomMsg->pose.covariance.end(), pose_covariance.begin());

    handle_vision_data(odomMsg->header.stamp.toSec(),
                       position,
                       orientation,
                       lin_vel,
                       ang_vel,
                       pose_covariance,
                       twist_covariance);
  }
}

void VisionPlugin::rosPoseCallBack(const geometry_msgs::PoseStampedConstPtr& odomMsg)
{
  double dt = calcTimeStep();

  if (dt > 1.0 / _pub_rate) {

    Eigen::Vector3d position(odomMsg->pose.position.x, odomMsg->pose.position.y, odomMsg->pose.position.z);
    Eigen::Quaterniond orientation(odomMsg->pose.orientation.w, odomMsg->pose.orientation.x, odomMsg->pose.orientation.y, odomMsg->pose.orientation.z);
    Eigen::Vector3d lin_vel(0, 0, 0); // linear velocity is not provided by sensor
    Eigen::Vector3d ang_vel(0, 0, 0); // angular velocity is not provided by sensor

    std::array<double, 36> pose_covariance{}; // all values are zero
    std::array<double, 36> twist_covariance{}; // all values are zero

    handle_vision_data(odomMsg->header.stamp.toSec(),
                       position,
                       orientation,
                       lin_vel,
                       ang_vel,
                       pose_covariance,
                       twist_covariance);
  }
}

void VisionPlugin::rosOdomCallBack(const nav_msgs::OdometryConstPtr& odomMsg)
{
  double dt = calcTimeStep();

  if (dt > 1.0 / _pub_rate) {

    Eigen::Vector3d position(odomMsg->pose.pose.position.x, odomMsg->pose.pose.position.y, odomMsg->pose.pose.position.z);
    Eigen::Quaterniond orientation(odomMsg->pose.pose.orientation.w, odomMsg->pose.pose.orientation.x, odomMsg->pose.pose.orientation.y, odomMsg->pose.pose.orientation.z);
    Eigen::Vector3d lin_vel(0, 0, 0); // linear velocity is not provided by sensor
    Eigen::Vector3d ang_vel(0, 0, 0); // angular velocity is not provided by sensor

    std::array<double, 36> pose_covariance{};
    std::array<double, 36> twist_covariance{};
    std::copy(odomMsg->pose.covariance.begin(), odomMsg->pose.covariance.end(), pose_covariance.begin());
    std::copy(odomMsg->twist.covariance.begin(), odomMsg->twist.covariance.end(), twist_covariance.begin());

    handle_vision_data(odomMsg->header.stamp.toSec(),
                       position,
                       orientation,
                       lin_vel,
                       ang_vel,
                       pose_covariance,
                       twist_covariance);
  }
}

void VisionPlugin::queueThread()
{
  static const double timeout = 0.01;
  while (this->_ros_node->ok())
    this->_ros_queue.callAvailable(ros::WallDuration(timeout));
}
#endif

void VisionPlugin::OnUpdate(const common::UpdateInfo&)
{
  double dt = calcTimeStep();

  if (dt > 1.0 / _pub_rate) {

    // get pose of the model that the plugin is attached to
#if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d pose_model_world = _model->WorldPose();
    ignition::math::Vector3d velocity_model_world = _model->WorldLinearVel();
    ignition::math::Vector3d angular_velocity_model = _model->RelativeAngularVel();
#else
    ignition::math::Pose3d pose_model_world = ignitionFromGazeboMath(_model->GetWorldPose());
    ignition::math::Vector3d velocity_model_world = ignitionFromGazeboMath(_model->GetWorldLinearVel());
    ignition::math::Vector3d angular_velocity_model = ignitionFromGazeboMath(_model->GetRelativeAngularVel());
#endif
    ignition::math::Pose3d pose_model; // pose in local frame (relative to where it started)
    pose_model.Pos().X() = pose_model_world.Pos().X() - _pose_model_start.Pos().X();
    pose_model.Pos().Y() = pose_model_world.Pos().Y() - _pose_model_start.Pos().Y();
    pose_model.Pos().Z() = pose_model_world.Pos().Z() - _pose_model_start.Pos().Z();
    pose_model.Rot().Euler(pose_model_world.Rot().Roll(),
                           pose_model_world.Rot().Pitch(),
                           pose_model_world.Rot().Yaw() - _pose_model_start.Rot().Yaw());

    // update noise parameters
    ignition::math::Vector3d noise_pos;
    ignition::math::Vector3d noise_linvel;
    ignition::math::Vector3d noise_angvel;

    // position noise model
    noise_pos.X() = _noise_density * sqrt(dt) * _randn(_rand);
    noise_pos.Y() = _noise_density * sqrt(dt) * _randn(_rand);
    noise_pos.Z() = _noise_density * sqrt(dt) * _randn(_rand);

    // velocity noise model
    noise_linvel.X() = _noise_density * sqrt(dt) * _randn(_rand);
    noise_linvel.Y() = _noise_density * sqrt(dt) * _randn(_rand);
    noise_linvel.Z() = _noise_density * sqrt(dt) * _randn(_rand);

    // angular rates noise model
    double tau_g = _corellation_time;
    double sigma_g_d = 1 / sqrt(dt) * _noise_density;
    double sigma_b_g = _random_walk;
    double sigma_b_g_d = sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 * (exp(-2.0 * dt / tau_g) - 1.0));
    double phi_g_d = exp(-1.0 / tau_g * dt);

    noise_angvel.X() = phi_g_d * noise_angvel.X() + sigma_b_g_d * sqrt(dt) * _randn(_rand);
    noise_angvel.Y() = phi_g_d * noise_angvel.Y() + sigma_b_g_d * sqrt(dt) * _randn(_rand);
    noise_angvel.Z() = phi_g_d * noise_angvel.Z() + sigma_b_g_d * sqrt(dt) * _randn(_rand);

    // random walk generation
    ignition::math::Vector3d random_walk;
    random_walk.X() = _random_walk * sqrt(dt) * _randn(_rand);
    random_walk.Y() = _random_walk * sqrt(dt) * _randn(_rand);
    random_walk.Z() = _random_walk * sqrt(dt) * _randn(_rand);

    // bias integration
    _bias.X() += random_walk.X() * dt - _bias.X() / _corellation_time;
    _bias.Y() += random_walk.Y() * dt - _bias.Y() / _corellation_time;
    _bias.Z() += random_walk.Z() * dt - _bias.Z() / _corellation_time;

    // Fill odom msg
    odom_msg.set_usec(_current_time.Double() * 1e6);

    gazebo::msgs::Vector3d* position = new gazebo::msgs::Vector3d();
    position->set_x(pose_model.Pos().X() + noise_pos.X() + _bias.X());
    position->set_y(pose_model.Pos().Y() + noise_pos.Y() + _bias.Y());
    position->set_z(pose_model.Pos().Z() + noise_pos.Z() + _bias.Z());
    odom_msg.set_allocated_position(position);

    ignition::math::Quaterniond pose_model_quaternion = pose_model.Rot();
    gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
    orientation->set_x(pose_model_quaternion.X());
    orientation->set_y(pose_model_quaternion.Y());
    orientation->set_z(pose_model_quaternion.Z());
    orientation->set_w(pose_model_quaternion.W());
    odom_msg.set_allocated_orientation(orientation);

    gazebo::msgs::Vector3d* linear_velocity = new gazebo::msgs::Vector3d();
    linear_velocity->set_x(velocity_model_world.X() + noise_linvel.X() + _bias.X());
    linear_velocity->set_y(velocity_model_world.Y() + noise_linvel.Y() + _bias.Y());
    linear_velocity->set_z(velocity_model_world.Z() + noise_linvel.Z() + _bias.Z());
    odom_msg.set_allocated_linear_velocity(linear_velocity);

    gazebo::msgs::Vector3d* angular_velocity = new gazebo::msgs::Vector3d();
    angular_velocity->set_x(angular_velocity_model.X() + noise_angvel.X());
    angular_velocity->set_y(angular_velocity_model.Y() + noise_angvel.Y());
    angular_velocity->set_z(angular_velocity_model.Z() + noise_angvel.Z());
    odom_msg.set_allocated_angular_velocity(angular_velocity);

    for (int i = 0; i < 36; i++){
      switch (i){
        // principal diagonal = the variance of the random variables
        // = noise_density²
        case 0: case 7: case 14: case 21: case 28: case 35:
          odom_msg.add_pose_covariance(_noise_density * _noise_density);
          odom_msg.add_twist_covariance(_noise_density * _noise_density);
          break;
        default:
          odom_msg.add_pose_covariance(0.0);
          odom_msg.add_twist_covariance(0.0);
      }
    }

    _last_pub_time = _current_time;

    // publish odom msg
    _pub_odom->Publish(odom_msg);
  }
}
} // namespace gazebo
