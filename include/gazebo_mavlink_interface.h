/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Pro Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <vector>
#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>

#include <iostream>
#include <random>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <Eigen/Eigen>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include <common.h>
#include <Airspeed.pb.h>
#include <CommandMotorSpeed.pb.h>
#include <MotorSpeed.pb.h>
#include <Imu.pb.h>
#include <OpticalFlow.pb.h>
#include <Range.pb.h>
#include <SITLGps.pb.h>
#include <IRLock.pb.h>
#include <TargetRelative.pb.h>
#include <Groundtruth.pb.h>
#include <Odometry.pb.h>
#include <MagneticField.pb.h>
#include <Pressure.pb.h>
#include <Wind.pb.h>

#include "mavlink_interface.h"
#include "msgbuffer.h"

//! Default distance sensor model joint naming
static const std::regex kDefaultLidarModelNaming(".*(lidar|sf10a)(.*)");
static const std::regex kDefaultSonarModelNaming(".*(sonar|mb1240-xl-ez4)(.*)");
static const std::regex kDefaultGPSModelNaming(".*(gps|ublox-neo-7M)(.*)");
static const std::regex kDefaultAirspeedModelJointNaming(".*(airspeed)(.*_joint)");

namespace gazebo {

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const nav_msgs::msgs::Odometry> OdomPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Airspeed> AirspeedPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::IRLock> IRLockPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::TargetRelative> TargetRelativePtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::OpticalFlow> OpticalFlowPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> SonarPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> LidarPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> GpsPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::MagneticField> MagnetometerPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Pressure> BarometerPtr;
typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;

typedef std::pair<const int, const ignition::math::Quaterniond> SensorIdRot_P;
typedef std::map<transport::SubscriberPtr, SensorIdRot_P > Sensor_M;

// Default values
static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed";

static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultOpticalFlowTopic = "/px4flow/link/opticalFlow";
static const std::string kDefaultIRLockTopic = "/camera/link/irlock";
static const std::string kDefaultTargetGpsTopic = "/land_pad/link/gps_target";
static const std::string kDefaultArucoMarkerTopic = "/aruco_cam/link/arucoMarker";
static const std::string kDefaultVisionTopic = "/vision_odom";
static const std::string kDefaultMagTopic = "/mag";
static const std::string kDefaultBarometerTopic = "/baro";
static const std::string kDefaultWindTopic = "/world_wind";
static const std::string kDefaultGroundtruthTopic = "/groundtruth";

//! OR operation for the enumeration and unsigned types that returns the bitmask
template<typename A, typename B>
static inline uint32_t operator |(A lhs, B rhs) {
  // make it type safe
  static_assert((std::is_same<A, uint32_t>::value || std::is_same<A, SensorSource>::value),
		"first argument is not uint32_t or SensorSource enum type");
  static_assert((std::is_same<B, uint32_t>::value || std::is_same<B, SensorSource>::value),
		"second argument is not uint32_t or SensorSource enum type");

  return static_cast<uint32_t> (
    static_cast<std::underlying_type<SensorSource>::type>(lhs) |
    static_cast<std::underlying_type<SensorSource>::type>(rhs)
  );
}

class GazeboMavlinkInterface : public ModelPlugin {
public:
  GazeboMavlinkInterface();
  ~GazeboMavlinkInterface();

  void Publish();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&  /*_info*/);

private:
  bool received_first_actuator_{false};
  Eigen::VectorXd input_reference_;

  float protocol_version_{2.0};

  std::unique_ptr<MavlinkInterface> mavlink_interface_;

  std::string namespace_{kDefaultNamespace};
  std::string motor_velocity_reference_pub_topic_{kDefaultMotorVelocityReferencePubTopic};
  std::string mavlink_control_sub_topic_;
  std::string link_name_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_reference_pub_;
  transport::SubscriberPtr mav_control_sub_;

  physics::ModelPtr model_{};
  physics::WorldPtr world_{nullptr};

  bool send_vision_estimation_{false};
  bool send_odometry_{false};

  std::vector<physics::JointPtr> joints_;
  std::vector<common::PID> pids_;
  std::vector<double> joint_max_errors_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;
  event::ConnectionPtr sigIntConnection_;

  void ImuCallback(ImuPtr& imu_msg);
  void GpsCallback(GpsPtr& gps_msg, const int& id);
  void TargetGpsCallback(GpsPtr& gps_msg);
  void GroundtruthCallback(GtPtr& groundtruth_msg);
  void LidarCallback(LidarPtr& lidar_msg, const int& id);
  void SonarCallback(SonarPtr& sonar_msg, const int& id);
  void AirspeedCallback(AirspeedPtr& airspeed_msg, const int& id);
  void OpticalFlowCallback(OpticalFlowPtr& opticalFlow_msg);
  void IRLockCallback(IRLockPtr& irlock_msg);
  void targetReleativeCallback(TargetRelativePtr& targetRelative_msg);
  void VisionCallback(OdomPtr& odom_msg);
  void MagnetometerCallback(MagnetometerPtr& mag_msg);
  void BarometerCallback(BarometerPtr& baro_msg);
  void WindVelocityCallback(WindPtr& msg);
  void SendSensorMessages();
  void SendGroundTruth();
  void handle_actuator_controls();
  void handle_control(double _dt);
  bool IsRunning();
  void onSigInt();

  /**
   * @brief Set the MAV_SENSOR_ORIENTATION enum value based on the sensor orientation
   *
   * @param[in] rootModel		The root model where the sensor is attached
   * @param[in] u_Xs				Unit vector of X-axis sensor in `base_link` frame
   * @param[in] sensor_msg	The Mavlink DISTANCE_SENSOR message struct
   */
  template <class T>
  void setMavlinkSensorOrientation(const ignition::math::Vector3d& u_Xs, T& sensor_msg);

  /**
   * @brief A helper class that allows the creation of multiple subscriptions to sensors.
   *	    It gets the sensor link/joint and creates the subscriptions based on those.
   *	    It also allows to set the initial rotation of the sensor, to allow computing
   *	    the sensor orientation quaternion.
   * @details GazeboMsgT  The type of the message that will be subscribed to the Gazebo framework.
   */
  template <typename GazeboMsgT>
  void CreateSensorSubscription(
      void (GazeboMavlinkInterface::*fp)(const boost::shared_ptr<GazeboMsgT const>&, const int&),
      GazeboMavlinkInterface* ptr, const physics::Joint_V& joints, physics::ModelPtr& nested_model, const std::regex& model);

  static const unsigned n_out_max = 16;

  double input_offset_[n_out_max]{};
  double input_scaling_[n_out_max]{};
  std::string joint_control_type_[n_out_max];
  std::string gztopic_[n_out_max];
  double zero_position_disarmed_[n_out_max]{};
  double zero_position_armed_[n_out_max]{};
  int input_index_[n_out_max]{};
  transport::PublisherPtr joint_control_pub_[n_out_max];

  transport::SubscriberPtr imu_sub_{nullptr};
  transport::SubscriberPtr opticalFlow_sub_{nullptr};
  transport::SubscriberPtr irlock_sub_{nullptr};
  transport::SubscriberPtr target_gps_sub_{nullptr};
  transport::SubscriberPtr arucoMarker_sub_{nullptr};
  transport::SubscriberPtr groundtruth_sub_{nullptr};
  transport::SubscriberPtr vision_sub_{nullptr};
  transport::SubscriberPtr mag_sub_{nullptr};
  transport::SubscriberPtr baro_sub_{nullptr};
  transport::SubscriberPtr wind_sub_{nullptr};

  Sensor_M sensor_map_{}; // Map of sensor SubscriberPtr, IDs and orientations

  std::string imu_sub_topic_{kDefaultImuTopic};
  std::string opticalFlow_sub_topic_{kDefaultOpticalFlowTopic};
  std::string irlock_sub_topic_{kDefaultIRLockTopic};
  std::string target_gps_sub_topic_{kDefaultTargetGpsTopic};
  std::string arucoMarker_sub_topic_{kDefaultArucoMarkerTopic};
  std::string groundtruth_sub_topic_{kDefaultGroundtruthTopic};
  std::string vision_sub_topic_{kDefaultVisionTopic};
  std::string mag_sub_topic_{kDefaultMagTopic};
  std::string baro_sub_topic_{kDefaultBarometerTopic};
  std::string wind_sub_topic_{kDefaultWindTopic};

  std::mutex imu_received_mutex_ {};
  std::condition_variable imu_received_cond_ {};
  bool imu_received_ {false};
  bool imu_received_once_ {false};
  int64_t last_imu_message_seq_{0};
  common::Time last_time_;
  common::Time last_imu_time_;
  common::Time last_actuator_time_;
  common::Time last_heartbeat_sent_time_{};

  double groundtruth_lat_rad_{0.0};
  double groundtruth_lon_rad_{0.0};
  double groundtruth_altitude_{0.0};

  double imu_update_interval_{0.004}; ///< Used for non-lockstep

  ignition::math::Vector3d velocity_prev_W_;
  ignition::math::Vector3d wind_vel_;

  bool close_conn_{false};

  double optflow_distance_{0.0};
  double sonar_distance;

  bool enable_lockstep_{false};
  double speed_factor_{1.0};
  unsigned update_skip_factor_{1};
  uint64_t update_counter_{0u};

  bool hil_mode_{false};
  bool hil_state_level_{false};

};
}
