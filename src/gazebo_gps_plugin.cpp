/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 * Copyright (C) 2017-2018 PX4 Pro Development Team
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
 * @brief GPS Plugin
 *
 * This plugin publishes GPS and Groundtruth data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <gazebo_gps_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(GpsPlugin)

GpsPlugin::GpsPlugin() : ModelPlugin()
{ }

GpsPlugin::~GpsPlugin()
{
  this->updateConnection_.reset();
#if BUILD_ROS_INTERFACE
  this->fix_queue_.clear();
  this->pose_queue_.clear();
  this->fix_queue_.disable();
  this->pose_queue_.disable();
  this->rosnode_->shutdown();
  this->pose_callback_queue_thread_.join();
  this->fix_callback_queue_thread_.join();
  this->rosnode_.reset();
#endif
}

void GpsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();
#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  last_gps_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
  last_gps_time_ = world_->GetSimTime();
#endif

  // Use environment variables if set for home position.
  const char *env_lat = std::getenv("PX4_HOME_LAT");
  const char *env_lon = std::getenv("PX4_HOME_LON");
  const char *env_alt = std::getenv("PX4_HOME_ALT");

  // Get noise param
  if (_sdf->HasElement("gpsNoise")) {
    getSdfParam<bool>(_sdf, "gpsNoise", gps_noise_, gps_noise_);
  } else {
    gps_noise_ = false;
  }

  // Get horizontal std deviation, in the case one doesn't want to use the computed random walk std dev
  if (_sdf->HasElement("eph")) {
    getSdfParam<double>(_sdf, "eph", eph_, eph_);
  } else {
    eph_ = kDefaultEPH;
    gzwarn << "[gazebo_gps_plugin] Using default EPH of " << eph_ << "\n";
  }

  // Get vertical std deviation, in the case one doesn't want to use the computed random walk std dev
  if (_sdf->HasElement("epv")) {
    getSdfParam<double>(_sdf, "eph", eph_, eph_);
  } else {
    epv_ = kDefaultEPH;
    gzwarn << "[gazebo_gps_plugin] Using default EPV of " << epv_ << "\n";
  }

#if BUILD_ROS_INTERFACE
  if(_sdf->HasElement("rosGPSENUPubTopic")) {
    this->ros_gps_pose_pub_topic_ = _sdf->GetElement("rosGPSENUPubTopic")->Get<std::string>();
  } else {
    this->ros_gps_pose_pub_topic_ = kDefaultRosGPSENUPubTopic;
    ROS_INFO_NAMED("gazebo_gps", "Using default ROS PoseWithCovarianceStamped subscription to topic %s", this->ros_gps_pose_pub_topic_.c_str());
  }
  if(_sdf->HasElement("rosGPSFixPubTopic")) {
    this->ros_gps_fix_pub_topic_ = _sdf->GetElement("rosGPSENUPubTopic")->Get<std::string>();
  } else {
    this->ros_gps_fix_pub_topic_ = kDefaultRosGPSFixPubTopic;
    ROS_INFO_NAMED("gazebo_gps", "Using default ROS NavSatFix subscription to topic %s", this->ros_gps_fix_pub_topic_.c_str());
  }
#endif

  if (env_lat) {
    gzmsg << "Home latitude is set to " << env_lat << ".\n";
    lat_home = std::stod(env_lat) * M_PI / 180.0;
  }
  if (env_lon) {
    gzmsg << "Home longitude is set to " << env_lon << ".\n";
    lon_home = std::stod(env_lon) * M_PI / 180.0;
  }
  if (env_alt) {
    gzmsg << "Home altitude is set to " << env_alt << ".\n";
    alt_home = std::stod(env_alt);
  }

  this->namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    this->namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_gps_plugin] Please specify a robotNamespace.\n";
  }

  this->node_handle_ = transport::NodePtr(new transport::Node());
  this->node_handle_->Init(namespace_);

#if BUILD_ROS_INTERFACE
  // Initialize ros, if it has not already been initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  this->rosnode_.reset(new ros::NodeHandle("gazebo_client"));

  // Publish multi queue
  this->pmq_.startServiceThread();

  // if topic name specified as empty, do not publish
  if (this->ros_gps_fix_pub_topic_ != "")
  {
    this->fix_pub_queue_ = this->pmq_.addPub<sensor_msgs::NavSatFix>();
    this->gps_fix_pub_ = this->rosnode_->advertise<sensor_msgs::NavSatFix>(this->ros_gps_fix_pub_topic_, 1);

    // advertise services on the custom queues
    ros::AdvertiseServiceOptions nav_sat_aso =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      this->ros_gps_pose_pub_topic_, boost::bind(&GpsPlugin::ServiceCallback,
      this, _1, _2), ros::VoidPtr(), &this->fix_queue_);
    this->pose_srv_ = this->rosnode_->advertiseService(nav_sat_aso);
  }

  if (this->ros_gps_pose_pub_topic_ != "")
  {
    this->pose_pub_queue_ = this->pmq_.addPub<geometry_msgs::PoseWithCovarianceStamped>();
    this->gps_pose_pub_ = this->rosnode_->advertise<geometry_msgs::PoseWithCovarianceStamped>(this->ros_gps_pose_pub_topic_, 1);

    // advertise services on the custom queues
    ros::AdvertiseServiceOptions pose_aso =
      ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      this->ros_gps_fix_pub_topic_, boost::bind(&GpsPlugin::ServiceCallback,
      this, _1, _2), ros::VoidPtr(), &this->pose_queue_);
    this->fix_srv_ = this->rosnode_->advertiseService(pose_aso);
  }

  ROS_INFO_NAMED("gazebo_gps", "Starting ENU pose data publishing (ns = %s)", this->namespace_.c_str() );
  this->pose_callback_queue_thread_ = boost::thread(boost::bind(&GpsPlugin::PoseQueueThread, this));

  ROS_INFO_NAMED("gazebo_gps", "Starting GPS fix data publishing (ns = %s)", this->namespace_.c_str() );
  this->fix_callback_queue_thread_= boost::thread(boost::bind(&GpsPlugin::FixQueueThread, this));
#endif

  // Listen to the update event. This event is broadcast every simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GpsPlugin::OnUpdate, this, _1));

  gravity_W_ = world_->Gravity();

  gps_pub_ = node_handle_->Advertise<sensor_msgs::msgs::SITLGps>("~/" + model_->GetName() + "/gps", 10);
  gt_pub_ = node_handle_->Advertise<sensor_msgs::msgs::Groundtruth>("~/" + model_->GetName() + "/groundtruth", 10);
}

#if BUILD_ROS_INTERFACE
// returns true always
bool GpsPlugin::ServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  return true;
}
#endif

void GpsPlugin::OnUpdate(const common::UpdateInfo&){
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = model_->WorldPose();    // TODO(burrimi): Check tf
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(model_->GetWorldPose());    // TODO(burrimi): Check tf
#endif
  ignition::math::Vector3d& pos_W_I = T_W_I.Pos();           // Use the models' world position for GPS and groundtruth

  // reproject position without noise into geographic coordinates
  auto latlon_gt = reproject(pos_W_I);

  // Use the models' world position for GPS velocity.
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d velocity_current_W = model_->WorldLinearVel();
#else
  ignition::math::Vector3d velocity_current_W = ignitionFromGazeboMath(model_->GetWorldLinearVel());
#endif

  ignition::math::Vector3d velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.Z() = 0;

  // update noise parameters if gps_noise_ is set
  noise_gps_pos.X() = gps_noise_ ? _gps_xy_noise_density * sqrt(dt) * randn_(rand_) : 0.0;
  noise_gps_pos.Y() = gps_noise_ ? _gps_xy_noise_density * sqrt(dt) * randn_(rand_) : 0.0;
  noise_gps_pos.Z() = gps_noise_ ? _gps_z_noise_density * sqrt(dt) * randn_(rand_) : 0.0;
  noise_gps_vel.X() = gps_noise_ ? _gps_vxy_noise_density * sqrt(dt) * randn_(rand_) : 0.0;
  noise_gps_vel.Y() = gps_noise_ ? _gps_vxy_noise_density * sqrt(dt) * randn_(rand_) : 0.0;
  noise_gps_vel.Z() = gps_noise_ ? _gps_vz_noise_density * sqrt(dt) * randn_(rand_) : 0.0;
  random_walk_gps.X() = gps_noise_ ? _gps_xy_random_walk * sqrt(dt) * randn_(rand_) : 0.0;
  random_walk_gps.Y() = gps_noise_ ? _gps_xy_random_walk * sqrt(dt) * randn_(rand_) : 0.0;
  random_walk_gps.Z() = gps_noise_ ? _gps_z_random_walk * sqrt(dt) * randn_(rand_) : 0.0;

  // gps bias integration
  gps_bias.X() += random_walk_gps.X() * dt - gps_bias.X() / _gps_corellation_time;
  gps_bias.Y() += random_walk_gps.Y() * dt - gps_bias.Y() / _gps_corellation_time;
  gps_bias.Z() += random_walk_gps.Z() * dt - gps_bias.Z() / _gps_corellation_time;

  // reproject position with noise into geographic coordinates
  auto pos_with_noise = pos_W_I + noise_gps_pos + gps_bias;
  auto latlon = reproject(pos_with_noise);

  // standard deviation of random walk
  _std_x = random_walk_gps.X() * _gps_corellation_time / sqrtf(2 * _gps_corellation_time - 1);
  _std_y = random_walk_gps.Y() * _gps_corellation_time / sqrtf(2 * _gps_corellation_time - 1);
  _std_z = random_walk_gps.Z() * _gps_corellation_time / sqrtf(2 * _gps_corellation_time - 1);

  // fill SITLGps msg
  gps_msg.set_time_usec(current_time.Double() * 1e6);
  gps_msg.set_latitude_deg(latlon.first * 180.0 / M_PI);
  gps_msg.set_longitude_deg(latlon.second * 180.0 / M_PI);
  gps_msg.set_altitude(pos_W_I.Z() + alt_home + noise_gps_pos.Z() + gps_bias.Z());
  if (gps_noise_) {
    gps_msg.set_eph(std::min(_std_x + _gps_xy_noise_density * _gps_xy_noise_density, _std_y + _gps_xy_noise_density * _gps_xy_noise_density));
    gps_msg.set_epv(_std_z + _gps_z_noise_density);
  }
  else {
    gps_msg.set_eph(eph_);
    gps_msg.set_epv(epv_);
  }
  gps_msg.set_velocity(velocity_current_W_xy.Length());
  gps_msg.set_velocity_east(velocity_current_W.X() + noise_gps_vel.Y());
  gps_msg.set_velocity_north(velocity_current_W.Y() + noise_gps_vel.X());
  gps_msg.set_velocity_up(velocity_current_W.Z() + noise_gps_vel.Z());

  // add msg to buffer
  gps_delay_buffer.push(gps_msg);

#if BUILD_ROS_INTERFACE
  // fill ROS NavSatFix msg
  auto fix_msg = boost::make_shared<sensor_msgs::NavSatFix>();

  fix_msg->header.frame_id = "map";
  fix_msg->header.stamp.sec = current_time.sec;
  fix_msg->header.stamp.nsec = current_time.nsec;

  fix_msg->latitude = latlon.first * 180.0 / M_PI;
  fix_msg->longitude = latlon.second * 180.0 / M_PI;
  fix_msg->altitude = pos_W_I.Z() + alt_home + noise_gps_pos.Z() + gps_bias.Z();

  fix_msg->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  fix_msg->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

  if (gps_noise_) {
    fix_msg->position_covariance[0] = std::pow(_std_x + _gps_xy_noise_density, 2);
    fix_msg->position_covariance[4] = std::pow(_std_y + _gps_xy_noise_density, 2);
    fix_msg->position_covariance[8] = std::pow(_std_z + _gps_z_noise_density, 2);
  }
  else {
    fix_msg->position_covariance[0] = fix_msg->position_covariance[4] = eph_ * eph_;
    fix_msg->position_covariance[8] = epv_ * epv_;
  }

  fix_msg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

  // fill ROS PoseWithCovarianceStamped msg
  auto pose_msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>();

  pose_msg->header.frame_id = "map";
  pose_msg->header.stamp.sec = current_time.sec;
  pose_msg->header.stamp.nsec = current_time.nsec;

  pose_msg->pose.pose.position.x = pos_with_noise.X();
  pose_msg->pose.pose.position.y = pos_with_noise.Y();
  pose_msg->pose.pose.position.z = pos_with_noise.Z();

  ignition::math::Quaterniond& rot_W_I = T_W_I.Rot();   // model orientation WRT world
  pose_msg->pose.pose.orientation.x = rot_W_I.X();
  pose_msg->pose.pose.orientation.y = rot_W_I.Y();
  pose_msg->pose.pose.orientation.z = rot_W_I.Z();
  pose_msg->pose.pose.orientation.w = rot_W_I.W();

  for (int i = 0; i < 36; i++){
    switch (i){
      // principal diagonal = the variance of the random variables
      case 0:
        pose_msg->pose.covariance[i] = std::pow(_std_x + _gps_xy_noise_density, 2);
        break;
      case 7:
        pose_msg->pose.covariance[i] = std::pow(_std_y + _gps_xy_noise_density, 2);
        break;
      case 14:
        pose_msg->pose.covariance[i] = std::pow(_std_z * _gps_xy_noise_density, 2);
        break;
      case 21: case 28: case 35:
        pose_msg->pose.covariance[i] = 0.01;
        break;
      default:
        pose_msg->pose.covariance[i] = 0.0;
    }
  }
#endif

  // apply GPS delay
  if ((current_time - last_gps_time_).Double() > gps_update_interval_) {
    last_gps_time_ = current_time;

    while (true) {
      gps_msg = gps_delay_buffer.front();
      double gps_current_delay = current_time.Double() - gps_delay_buffer.front().time_usec() / 1e6f;
      if (gps_delay_buffer.empty()) {
        // abort if buffer is empty already
        break;
      }
      // remove data that is too old or if buffer size is too large
      if (gps_current_delay > gps_delay) {
        gps_delay_buffer.pop();
        // remove data if buffer too large
      } else if (gps_delay_buffer.size() > gps_buffer_size_max) {
        gps_delay_buffer.pop();
      } else {
        // if we get here, we have good data, stop
        break;
      }
    }
    // publish SITLGps msg at 5hz
    gps_pub_->Publish(gps_msg);

#if BUILD_ROS_INTERFACE
    {
      boost::mutex::scoped_lock lock(this->lock_);
      // publish local ENU data as a PoseWithCovarianceStamped ROS topic
      if (this->gps_pose_pub_.getNumSubscribers() > 0 && this->ros_gps_pose_pub_topic_ != "")
      {
        this->pose_pub_queue_->push(*pose_msg, this->gps_pose_pub_);
      }
      // publish projected GPS fix data as a NavSatFix ROS topic
      if (this->gps_fix_pub_.getNumSubscribers() > 0 && this->ros_gps_fix_pub_topic_ != "")
      {
        this->fix_pub_queue_->push(*fix_msg, this->gps_fix_pub_);
      }
    }
#endif
  }

  // fill Groundtruth msg
  groundtruth_msg.set_time_usec(current_time.Double() * 1e6);
  groundtruth_msg.set_latitude_rad(latlon_gt.first);
  groundtruth_msg.set_longitude_rad(latlon_gt.second);
  groundtruth_msg.set_altitude(-pos_W_I.Z() + alt_home);
  groundtruth_msg.set_velocity_east(velocity_current_W.X());
  groundtruth_msg.set_velocity_north(velocity_current_W.Y());
  groundtruth_msg.set_velocity_up(velocity_current_W.Z());

  // publish Groundtruth msg at full rate
  gt_pub_->Publish(groundtruth_msg);

  last_time_ = current_time;
}

std::pair<double, double> GpsPlugin::reproject(ignition::math::Vector3d& pos)
{
  // reproject local position to gps coordinates
  double x_rad = pos.Y() / earth_radius;    // north
  double y_rad = pos.X() / earth_radius;    // east
  double c = sqrt(x_rad * x_rad + y_rad * y_rad);
  double sin_c = sin(c);
  double cos_c = cos(c);
  double lat_rad, lon_rad;

  if (c != 0.0) {
    lat_rad = asin(cos_c * sin(lat_home) + (x_rad * sin_c * cos(lat_home)) / c);
    lon_rad = (lon_home + atan2(y_rad * sin_c, c * cos(lat_home) * cos_c - x_rad * sin(lat_home) * sin_c));
  } else {
    lat_rad = lat_home;
    lon_rad = lon_home;
  }

  return std::make_pair (lat_rad, lon_rad);
}

#if BUILD_ROS_INTERFACE
void GpsPlugin::PoseQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->pose_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GpsPlugin::FixQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->fix_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif
} // namespace gazebo
