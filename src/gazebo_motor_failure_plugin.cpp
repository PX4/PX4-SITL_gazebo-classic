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

#include "gazebo_motor_failure_plugin.h"

namespace gazebo {

GazeboMotorFailure::~GazeboMotorFailure() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}


void GazeboMotorFailure::Publish_num() {
  motor_failure_msg_.set_data(motor_Failure_Number_);
  motor_failure_pub_->Publish(motor_failure_msg_);
}

void GazeboMotorFailure::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  namespace_.clear();

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  motor_failure_pub_ = node_handle_->Advertise<msgs::Int>(motor_failure_num_pub_topic_, 1);

  getSdfParam<std::string>(_sdf, "ROSMotorNumSubTopic", ROS_motor_num_sub_topic_, ROS_motor_num_sub_topic_);
  getSdfParam<std::string>(_sdf, "MotorFailureNumPubTopic", motor_failure_num_pub_topic_, motor_failure_num_pub_topic_);


  // ROS Topic subscriber
  // Initialize ROS, if it has not already bee initialized.
  if (!ros::isInitialized())  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_ros_sub", ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Int32>(ROS_motor_num_sub_topic_, 1, boost::bind(&GazeboMotorFailure::motorFailNumCallBack, this, _1), ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);
  
  this->rosQueueThread = std::thread(std::bind(&GazeboMotorFailure::QueueThread, this));

  std::cout << "[gazebo_motor_failure_plugin]: Subscribe to ROS topic: "<< ROS_motor_num_sub_topic_ << std::endl;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMotorFailure::OnUpdate, this, _1));
}

void GazeboMotorFailure::OnUpdate(const common::UpdateInfo& _info) {
  Publish_num();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMotorFailure);
}

