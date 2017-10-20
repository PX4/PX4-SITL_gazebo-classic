/*
 * Copyright 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
 * Copyright 2017 Siddharth Patel, NTU Singapore
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

#include <stdio.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "common.h"
//#include "int.pb.h"

// ROS Topic subscriber
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <std_msgs/Int32.h>

namespace gazebo {

// Default values
static const std::string kDefaultROSMotorNumSubTopic = "/motor_failure/motor_number";
static const std::string kDefaultMotorFailureNumPubTopic = "/gazebo/motor_failure_num";

static constexpr int kDefaultMotorFailureNum = 0;

class GazeboMotorFailure : public ModelPlugin {
 public:

  void motorFailNumCallBack(const std_msgs::Int32ConstPtr& _msg) {
    this->motor_Failure_Number_ = _msg->data;
    //std::cout << "[gazebo_motor_failure_plugin]: Subscribe to /motor_failure/motor_number " << std::endl;
    //std::cout << "[gazebo_motor_failure_plugin] Current motor num : " << motor_Failure_Number_ << std::endl;
  }

  GazeboMotorFailure()
      : ModelPlugin(),
        motor_Failure_Number_(kDefaultMotorFailureNum),
	ROS_motor_num_sub_topic_(kDefaultROSMotorNumSubTopic),
	motor_failure_num_pub_topic_(kDefaultMotorFailureNumPubTopic) {
  }

  virtual ~GazeboMotorFailure();
  virtual void Publish();

  //void MainTask();
 
 protected:

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:

  int motor_Failure_Number_;
  int tmp_motor_num;

  physics::ModelPtr model_;

  std::string ROS_motor_num_sub_topic_;
  std::string motor_failure_num_pub_topic_;
  std::string namespace_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_failure_pub_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread()
  {
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  msgs::Int motor_failure_msg_;

  // ROS communication
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Subscriber rosSub;
  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
};
}
