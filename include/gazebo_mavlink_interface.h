/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "CommandMotorSpeed.pb.h"
#include "MotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <stdio.h>

#include "common.h"

#include "SensorImu.pb.h"
#include "HilControl.pb.h"
#include "HilSensor.pb.h"
#include "HilGps.pb.h"
#include <boost/bind.hpp>

#include <iostream>
#include <math.h>
#include <deque>
#include <random>
#include <sdf/sdf.hh>

#include "mavlink/v1.0/common/mavlink.h"

#include "gazebo/math/Vector3.hh"
#include <sys/socket.h>
#include <netinet/in.h>

static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;


namespace gazebo {

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const mavlink::msgs::HilControl>   HilControlPtr;
typedef const boost::shared_ptr<const mavlink::msgs::HilSensor>   HilSensorPtr;
typedef const boost::shared_ptr<const mavlink::msgs::HilGps>   HilGpsPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;

// Default values
static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic = "gazebo/command/motor_speed";
static const std::string kDefaultMavlinkControlSubTopic = "HilControl";

static const std::string kDefaultImuTopic = "imu";
static const std::string kDefaultMavlinkHilSensorPubTopic = "HilSensor";
static const std::string kDefaultMavlinkHilGpsPubTopic = "HilGps";

static bool use_mavlink_udp = true;

class GazeboMavlinkInterface : public ModelPlugin {
 public:
  GazeboMavlinkInterface()
      : ModelPlugin(),
        received_first_referenc_(false),
        namespace_(kDefaultNamespace),
        motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
        hil_sensor_mavlink_pub_topic_(kDefaultMavlinkHilSensorPubTopic),
        hil_gps_mavlink_pub_topic_(kDefaultMavlinkHilGpsPubTopic),
        imu_sub_topic_(kDefaultImuTopic),
        mavlink_control_sub_topic_(kDefaultMavlinkControlSubTopic) {}
  ~GazeboMavlinkInterface();

  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:

  bool received_first_referenc_;
  Eigen::VectorXd input_reference_;

  std::string namespace_;
  std::string motor_velocity_reference_pub_topic_;
  std::string mavlink_control_sub_topic_;
  std::string link_name_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_reference_pub_;
  transport::SubscriberPtr mav_control_sub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr left_elevon_joint_;
  physics::JointPtr right_elevon_joint_;
  physics::JointPtr elevator_joint_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void HilControlCallback(HilControlPtr &rmsg);
  void ImuCallback(ImuPtr& imu_msg);
  void send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID);
  void handle_message(mavlink_message_t *msg);
  void pollForMAVLinkMessages();

  unsigned _rotor_count;
  struct {
    float control[8];
  } inputs; 

  transport::SubscriberPtr imu_sub_;
  transport::PublisherPtr hil_sensor_pub_;
  transport::PublisherPtr hil_gps_pub_;

  std::string hil_sensor_mavlink_pub_topic_;
  std::string hil_gps_mavlink_pub_topic_;
  std::string imu_sub_topic_;
  std::string left_elevon_joint_name_;
  std::string right_elevon_joint_name_;
  std::string elevator_joint_name_;
  
  common::Time last_time_;
  common::Time last_gps_time_;
  double gps_update_interval_;

  math::Vector3 gravity_W_;
  math::Vector3 velocity_prev_W_;
  math::Vector3 mag_W_;

  std::default_random_engine random_generator_;
  std::normal_distribution<float> standard_normal_distribution_;

  mavlink::msgs::HilSensor hil_sensor_msg_;
  mavlink::msgs::HilGps hil_gps_msg_;

  int _fd;
  struct sockaddr_in _myaddr;
  struct sockaddr_in _srcaddr;
  socklen_t _addrlen;
  unsigned char _buf[5000];
  struct pollfd fds[1];
  };
}
