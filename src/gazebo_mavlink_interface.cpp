/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2021 PX4 Development Team
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

#include <gazebo_mavlink_interface.h>
namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkInterface);

GazeboMavlinkInterface::GazeboMavlinkInterface() : ModelPlugin(){
      mavlink_interface_ = std::make_unique<MavlinkInterface>();

}

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  mavlink_interface_->close();
  sigIntConnection_->~Connection();
  updateConnection_->~Connection();
}

template <class T>
T our_any_cast(const boost::any &val) {
#if GAZEBO_MAJOR_VERSION >= 11
  return gazebo::physics::PhysicsEngine::any_cast<T>(val);
#else
  return boost::any_cast<T>(val);
#endif
}


/// \brief      A helper class that provides storage for additional parameters that are inserted into the callback.
/// \details    GazeboMsgT  The type of the message that will be subscribed to the Gazebo framework.
template <typename GazeboMsgT>
struct SensorHelperStorage {
  /// \brief    Pointer to the ROS interface plugin class.
  GazeboMavlinkInterface* ptr;

  /// \brief    Function pointer to the subscriber callback with additional parameters.
  void (GazeboMavlinkInterface::*fp)(const boost::shared_ptr<GazeboMsgT const>&, const int&);

  /// \brief    The sensor ID.
  int sensor_id;

  /// \brief    This is what gets passed into the Gazebo Subscribe method as a callback,
  ///           and hence can onlyhave one parameter (note boost::bind() does not work with the
  ///           current Gazebo Subscribe() definitions).
  void callback(const boost::shared_ptr<GazeboMsgT const>& msg_ptr) {
    (ptr->*fp)(msg_ptr, sensor_id);
  }
};

template <typename GazeboMsgT>
void GazeboMavlinkInterface::CreateSensorSubscription(
    void (GazeboMavlinkInterface::*fp)(const boost::shared_ptr<GazeboMsgT const>&, const int&),
    GazeboMavlinkInterface* ptr, const physics::Joint_V& joints, physics::ModelPtr& nested_model, const std::regex& model) {

  // Get nested sensors on included models
  std::string nested_sensor_name;
  if (nested_model != nullptr && std::regex_match(nested_model->GetName(), model)) {
    const std::string model_name = model_->GetName();

    if (nested_model->GetName().find("::") != std::string::npos) {
      nested_sensor_name = nested_model->GetName().substr(nested_model->GetName().find("::") + 2);
    } else {
      nested_sensor_name = nested_model->GetName();
    }

    // Get sensor ID sensor name
    int sensor_id = 0;
    try {
      // get the sensor id by getting the (last) numbers on the sensor name (ex. lidar10, gets id 10)
      sensor_id = std::stoi(nested_sensor_name.substr(nested_sensor_name.find_last_not_of("0123456789") + 1));
    } catch(...) {
      gzwarn << "No identifier on joint. Using 0 as default sensor ID" << std::endl;
    }

    // Get the sensor link orientation with respect to the base_link
#if GAZEBO_MAJOR_VERSION >= 9
    const auto sensor_orientation = nested_model->RelativePose().Rot();
#else
    const auto sensor_orientation = nested_model->GetChild()->GetRelativePose()).Rot();
#endif

    // One map will be created for each Gazebo message type
    static std::map<std::string, SensorHelperStorage<GazeboMsgT> > callback_map;

    // Store the callback entries
    auto callback_entry = callback_map.emplace(
        "~/" + model_name + "/link/" + nested_sensor_name,
        SensorHelperStorage<GazeboMsgT>{ptr, fp, sensor_id});

    // Check if element was already present
    if (!callback_entry.second)
      gzerr << "Tried to add element to map but the gazebo topic name was already present in map."
            << std::endl;

    // Create the subscriber for the sensors
    auto subscriberPtr = node_handle_->Subscribe("~/" + model_name + "/link/" + nested_sensor_name,
                                                 &SensorHelperStorage<GazeboMsgT>::callback,
                                                 &callback_entry.first->second);

    // Store the SubscriberPtr, sensor ID and sensor orientation
    sensor_map_.insert(std::pair<transport::SubscriberPtr, SensorIdRot_P>(subscriberPtr,
                                                                          SensorIdRot_P(sensor_id, sensor_orientation))
                                                                         );
  }

  // Verify if the sensor joint exists
  for (physics::Joint_V::const_iterator it = joints.begin(); it != joints.end(); ++it) {
    // std::cout << (*it)->GetName() << std::endl;
    if (std::regex_match((*it)->GetName(), model)) {
      // Get sensor joint name (without the ''::joint' suffix)
      const std::string joint_name = (*it)->GetName().substr(0, (*it)->GetName().size() - 6);

      // If the model is nested, use the sensor name (child model) as the sensor topic
      const std::string model_name = model_->GetName();
      const std::string::size_type pos = joint_name.find("::");

      std::string sensor_name = joint_name;
      if (pos != std::string::npos) {
        sensor_name = joint_name.substr(pos + 2);
      }

      // If a nested sensor was already registered with this name
      // ignore it
      if (nested_sensor_name == sensor_name) {
        break;
      }

      // Get sensor ID from joint name
      int sensor_id = 0;
      try {
        // get the sensor id by getting the (last) numbers on the joint name (ex. lidar10_joint, gets id 10)
        sensor_id = std::stoi(sensor_name.substr(sensor_name.find_last_not_of("0123456789") + 1));
      } catch(...) {
        gzwarn << "No identifier on joint. Using 0 as default sensor ID" << std::endl;
      }

      // Get the sensor link orientation with respect to the base_link
  #if GAZEBO_MAJOR_VERSION >= 9
      const auto sensor_orientation = (*it)->GetChild()->RelativePose().Rot();
  #else
      const auto sensor_orientation = ignitionFromGazeboMath((*it)->GetChild()->GetRelativePose()).Rot();
  #endif

      // One map will be created for each Gazebo message type
      static std::map<std::string, SensorHelperStorage<GazeboMsgT> > callback_map;

      // Store the callback entries
      auto callback_entry = callback_map.emplace(
          "~/" + model_name + "/link/" + sensor_name,
          SensorHelperStorage<GazeboMsgT>{ptr, fp, sensor_id});

      // Check if element was already present
      if (!callback_entry.second)
        gzerr << "Tried to add element to map but the gazebo topic name was already present in map."
              << std::endl;

      // Create the subscriber for the sensors
      auto subscriberPtr = node_handle_->Subscribe("~/" + model_name + "/link/" + sensor_name,
                                                   &SensorHelperStorage<GazeboMsgT>::callback,
                                                   &callback_entry.first->second);
      // Store the SubscriberPtr, sensor ID and sensor orientation
      sensor_map_.insert(std::pair<transport::SubscriberPtr, SensorIdRot_P>(subscriberPtr,
                                                                            SensorIdRot_P(sensor_id, sensor_orientation))
                                                                           );
    }
  }
}

void GazeboMavlinkInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  model_ = _model;
  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  }

  if (_sdf->HasElement("protocol_version")) {
    protocol_version_ = _sdf->GetElement("protocol_version")->Get<float>();
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
      motor_velocity_reference_pub_topic_);
  getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  getSdfParam<std::string>(_sdf, "visionSubTopic", vision_sub_topic_, vision_sub_topic_);
  getSdfParam<std::string>(_sdf, "opticalFlowSubTopic",
      opticalFlow_sub_topic_, opticalFlow_sub_topic_);
  getSdfParam<std::string>(_sdf, "irlockSubTopic", irlock_sub_topic_, irlock_sub_topic_);
  getSdfParam<std::string>(_sdf, "magSubTopic", mag_sub_topic_, mag_sub_topic_);
  getSdfParam<std::string>(_sdf, "baroSubTopic", baro_sub_topic_, baro_sub_topic_);
  getSdfParam<std::string>(_sdf, "groundtruthSubTopic", groundtruth_sub_topic_, groundtruth_sub_topic_);

  // set input_reference_ from inputs.control
  input_reference_.resize(n_out_max);
  joints_.resize(n_out_max);
  pids_.resize(n_out_max);
  joint_max_errors_.resize(n_out_max);
  for (int i = 0; i < n_out_max; ++i)
  {
    pids_[i].Init(0, 0, 0, 0, 0, 0, 0);
    input_reference_[i] = 0;
  }

  if (_sdf->HasElement("control_channels")) {
    sdf::ElementPtr control_channels = _sdf->GetElement("control_channels");
    sdf::ElementPtr channel = control_channels->GetElement("channel");
    while (channel)
    {
      if (channel->HasElement("input_index"))
      {
        int index = channel->Get<int>("input_index");
        if (index < n_out_max)
        {
          input_offset_[index] = channel->Get<double>("input_offset");
          input_scaling_[index] = channel->Get<double>("input_scaling");
          zero_position_disarmed_[index] = channel->Get<double>("zero_position_disarmed");
          zero_position_armed_[index] = channel->Get<double>("zero_position_armed");
          if (channel->HasElement("joint_control_type"))
          {
            joint_control_type_[index] = channel->Get<std::string>("joint_control_type");
          }
          else
          {
            gzwarn << "joint_control_type[" << index << "] not specified, using velocity.\n";
            joint_control_type_[index] = "velocity";
          }

          // start gz transport node handle
          if (joint_control_type_[index] == "position_gztopic")
          {
            // setup publisher handle to topic
            if (channel->HasElement("gztopic"))
              gztopic_[index] = "~/" + model_->GetName() + channel->Get<std::string>("gztopic");
            else
              gztopic_[index] = "control_position_gztopic_" + std::to_string(index);
#if GAZEBO_MAJOR_VERSION > 7 || (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION >= 4)
            /// only gazebo 7.4 and above support Any
            joint_control_pub_[index] = node_handle_->Advertise<gazebo::msgs::Any>(
                gztopic_[index]);
#else
            joint_control_pub_[index] = node_handle_->Advertise<gazebo::msgs::GzString>(
                gztopic_[index]);
#endif
          }

          if (channel->HasElement("joint_name"))
          {
            std::string joint_name = channel->Get<std::string>("joint_name");
            joints_[index] = model_->GetJoint(joint_name);
          }

          // setup joint control pid to control joint
          if (channel->HasElement("joint_control_pid"))
          {
            sdf::ElementPtr pid = channel->GetElement("joint_control_pid");
            double p = 0;
            if (pid->HasElement("p"))
              p = pid->Get<double>("p");
            double i = 0;
            if (pid->HasElement("i"))
              i = pid->Get<double>("i");
            double d = 0;
            if (pid->HasElement("d"))
              d = pid->Get<double>("d");
            double iMax = 0;
            if (pid->HasElement("iMax"))
              iMax = pid->Get<double>("iMax");
            double iMin = 0;
            if (pid->HasElement("iMin"))
              iMin = pid->Get<double>("iMin");
            double cmdMax = 0;
            if (pid->HasElement("cmdMax"))
              cmdMax = pid->Get<double>("cmdMax");
            double cmdMin = 0;
            if (pid->HasElement("cmdMin"))
              cmdMin = pid->Get<double>("cmdMin");
            if (pid->HasElement("errMax")) {
              joint_max_errors_[index] = pid->Get<double>("errMax");
            }
            pids_[index].Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
          }
        }
        else
        {
          gzerr << "input_index[" << index << "] out of range, not parsing.\n";
        }
      }
      else
      {
        gzerr << "no input_index, not parsing.\n";
        break;
      }
      channel = channel->GetNextElement("channel");
    }
  }

  if(_sdf->HasElement("hil_mode"))
  {
    hil_mode_ = _sdf->GetElement("hil_mode")->Get<bool>();
    mavlink_interface_->SetHILMode(hil_mode_);
  }

  if(_sdf->HasElement("hil_state_level"))
  {
    hil_state_level_ = _sdf->GetElement("hil_state_level")->Get<bool>();
    mavlink_interface_->SetHILStateLevel(hil_state_level_);
  }

  bool serial_enabled=false;
  if(_sdf->HasElement("serialEnabled"))
  {
    serial_enabled = _sdf->GetElement("serialEnabled")->Get<bool>();
    mavlink_interface_->SetSerialEnabled(serial_enabled);
  }

  bool use_tcp = false;
  if (!serial_enabled && _sdf->HasElement("use_tcp"))
  {
    use_tcp = _sdf->GetElement("use_tcp")->Get<bool>();
    mavlink_interface_->SetUseTcp(use_tcp);
  }
  gzmsg << "Connecting to PX4 SITL using " << (serial_enabled ? "serial" : (use_tcp ? "TCP" : "UDP")) << "\n";

  if (!hil_mode_ && _sdf->HasElement("enable_lockstep"))
  {
    enable_lockstep_ = _sdf->GetElement("enable_lockstep")->Get<bool>();
    mavlink_interface_->SetEnableLockstep(enable_lockstep_);
  }
  gzmsg << "Lockstep is " << (enable_lockstep_ ? "enabled" : "disabled") << "\n";

  // When running in lockstep, we can run the simulation slower or faster than
  // realtime. The speed can be set using the env variable PX4_SIM_SPEED_FACTOR.
  if (enable_lockstep_)
  {
    const char *speed_factor_str = std::getenv("PX4_SIM_SPEED_FACTOR");
    if (speed_factor_str)
    {
      speed_factor_ = std::atof(speed_factor_str);
      if (!std::isfinite(speed_factor_) || speed_factor_ <= 0.0)
      {
        gzerr << "Invalid speed factor '" << speed_factor_str << "', aborting\n";
        abort();
      }
    }
    gzmsg << "Speed factor set to: " << speed_factor_ << "\n";

    boost::any param;
#if GAZEBO_MAJOR_VERSION >= 8
    physics::PresetManagerPtr presetManager = world_->PresetMgr();
#else
    physics::PresetManagerPtr presetManager = world_->GetPresetManager();
#endif
    presetManager->CurrentProfile("default_physics");

    // We currently need to have the real_time_update_rate at a multiple of 250 Hz for lockstep.
    // Also, the max_step_size needs to match this (e.g. 0.004 s at 250 Hz or 0.002 s at 500 Hz).
    // Therefore we check these params and abort if they won't work.

    presetManager->GetCurrentProfileParam("real_time_update_rate", param);
    double real_time_update_rate = our_any_cast<double>(param);
    const int real_time_update_rate_int = static_cast<int>(real_time_update_rate + 0.5);

    if (real_time_update_rate_int % 250 != 0)
    {
      gzerr << "real_time_update_rate is " << real_time_update_rate_int
            << " but needs to be multiple of 250 Hz, aborting.\n";
      abort();
    }

    presetManager->GetCurrentProfileParam("max_step_size", param);
    const double max_step_size = our_any_cast<double>(param);
    if (1.0 / real_time_update_rate != max_step_size)
    {
      gzerr << "max_step_size of " << max_step_size
            << " s does not match real_time_update_rate of "
            << real_time_update_rate << ", aborting.\n";
      abort();
    }

    update_skip_factor_ = real_time_update_rate_int / 250;

    // Adapt the real_time_update_rate according to the speed
    // that we ask for in the env variable.
    real_time_update_rate *= speed_factor_;
    presetManager->SetCurrentProfileParam("real_time_update_rate", real_time_update_rate);
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  // Listen to Ctrl+C / SIGINT.
  sigIntConnection_ = event::Events::ConnectSigInt(
      boost::bind(&GazeboMavlinkInterface::onSigInt, this));

  // Subscribe to messages of other plugins.
  imu_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + imu_sub_topic_, &GazeboMavlinkInterface::ImuCallback, this);
  opticalFlow_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + opticalFlow_sub_topic_, &GazeboMavlinkInterface::OpticalFlowCallback, this);
  irlock_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + irlock_sub_topic_, &GazeboMavlinkInterface::IRLockCallback, this);
  groundtruth_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + groundtruth_sub_topic_, &GazeboMavlinkInterface::GroundtruthCallback, this);
  vision_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + vision_sub_topic_, &GazeboMavlinkInterface::VisionCallback, this);
  mag_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + mag_sub_topic_, &GazeboMavlinkInterface::MagnetometerCallback, this);
  baro_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + baro_sub_topic_, &GazeboMavlinkInterface::BarometerCallback, this);
  wind_sub_ = node_handle_->Subscribe("~/" + wind_sub_topic_, &GazeboMavlinkInterface::WindVelocityCallback, this);

  // Get the model joints
  auto joints = model_->GetJoints();

  // Get the base nested model, if nested model exist.
  // Note: this will only capture the first nested model found on the base model
  //       which is usually the GPS model of the aircraft (ex: iris_dual_gps
  //       includes iris and the nested model is iris::gps0).
  //       As an improvement, this can be made more generic by going through all
  //       the nested models present in the base model
  physics::ModelPtr nested_model;
  if (!model_->NestedModels().empty()) {
    nested_model = model_->NestedModels()[0];
  }

  // Create subscriptions to the distance sensors
  CreateSensorSubscription(&GazeboMavlinkInterface::LidarCallback, this, joints, nested_model, kDefaultLidarModelNaming);
  CreateSensorSubscription(&GazeboMavlinkInterface::SonarCallback, this, joints, nested_model, kDefaultSonarModelNaming);
  CreateSensorSubscription(&GazeboMavlinkInterface::GpsCallback, this, joints, nested_model, kDefaultGPSModelNaming);
  CreateSensorSubscription(&GazeboMavlinkInterface::AirspeedCallback, this, joints, nested_model, kDefaultAirspeedModelJointNaming);

  // Publish gazebo's motor_speed message
  motor_velocity_reference_pub_ = node_handle_->Advertise<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + motor_velocity_reference_pub_topic_, 1);

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
  last_imu_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
  last_imu_time_ = world_->GetSimTime();
#endif

  // This doesn't seem to be used anywhere but we leave it here
  // for potential compatibility
  if (_sdf->HasElement("imu_rate")) {
    imu_update_interval_ = 1 / _sdf->GetElement("imu_rate")->Get<int>();
  }

  if (_sdf->HasElement("mavlink_addr")) {
    std::string mavlink_addr_str = _sdf->GetElement("mavlink_addr")->Get<std::string>();
    if (mavlink_addr_str != "INADDR_ANY") {
      mavlink_interface_->SetMavlinkAddr(mavlink_addr_str);
    }
  }

#if GAZEBO_MAJOR_VERSION >= 9
  auto worldName = world_->Name();
#else
  auto worldName = world_->GetName();
#endif

  if (_sdf->HasElement("mavlink_udp_port")) {
    int mavlink_udp_port = _sdf->GetElement("mavlink_udp_port")->Get<int>();
    mavlink_interface_->SetMavlinkUdpPort(mavlink_udp_port);
  }

  if (_sdf->HasElement("mavlink_tcp_port")) {
    int mavlink_tcp_port = _sdf->GetElement("mavlink_tcp_port")->Get<int>();
    mavlink_interface_->SetMavlinkTcpPort(mavlink_tcp_port);
  }

  if (_sdf->HasElement("qgc_addr")) {
    std::string qgc_addr = _sdf->GetElement("qgc_addr")->Get<std::string>();
    if (qgc_addr != "INADDR_ANY") {
      mavlink_interface_->SetQgcAddr(qgc_addr);
    }
  }
  if (_sdf->HasElement("qgc_udp_port")) {
    int qgc_udp_port = _sdf->GetElement("qgc_udp_port")->Get<int>();
    mavlink_interface_->SetQgcUdpPort(qgc_udp_port);
  }

  if (_sdf->HasElement("sdk_addr")) {
    std::string sdk_addr = _sdf->GetElement("sdk_addr")->Get<std::string>();
    if (sdk_addr != "INADDR_ANY") {
      mavlink_interface_->SetSdkAddr(sdk_addr);
    }
  }
  if (_sdf->HasElement("sdk_udp_port")) {
    int sdk_udp_port = _sdf->GetElement("sdk_udp_port")->Get<int>();
    mavlink_interface_->SetSdkUdpPort(sdk_udp_port);
  }

  if (serial_enabled) {
    // Set up serial interface
    if(_sdf->HasElement("serialDevice"))
    {
      std::string device = _sdf->GetElement("serialDevice")->Get<std::string>();
      mavlink_interface_->SetDevice(device);
    }

    if (_sdf->HasElement("baudRate")) {
      int baudrate = _sdf->GetElement("baudRate")->Get<int>();
      mavlink_interface_->SetBaudrate(baudrate);
    }
  }

  if(_sdf->HasElement("send_vision_estimation"))
  {
    send_vision_estimation_ = _sdf->GetElement("send_vision_estimation")->Get<bool>();
  }

  if(_sdf->HasElement("send_odometry"))
  {
    send_odometry_ = _sdf->GetElement("send_odometry")->Get<bool>();
  }

  mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_0);

  // set the Mavlink protocol version to use on the link
  if (protocol_version_ == 2.0) {
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    gzmsg << "Using MAVLink protocol v2.0\n";
  }
  else if (protocol_version_ == 1.0) {
    chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    gzmsg << "Using MAVLink protocol v1.0\n";
  }
  else {
    gzerr << "Unkown protocol version! Using v" << protocol_version_ << "by default \n";
  }

  mavlink_interface_->Load();
}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo&  /*_info*/) {

  std::unique_lock<std::mutex> lock(last_imu_message_mutex_);

  if (previous_imu_seq_ > 0) {
    while (previous_imu_seq_ == last_imu_message_.seq() && IsRunning()) {
      last_imu_message_cond_.wait_for(lock, std::chrono::milliseconds(10));
    }
  }

  previous_imu_seq_ = last_imu_message_.seq();

  // Always run at 250 Hz. At 500 Hz, the skip factor should be 2, at 1000 Hz 4.
  if (!(previous_imu_seq_ % update_skip_factor_ == 0)) {
    return;
  }

#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();

  bool close_conn_ = false;

  if (hil_mode_) {
    mavlink_interface_->pollFromQgcAndSdk();
  } else {
    mavlink_interface_->pollForMAVLinkMessages();
  }

  // Always send Gyro and Accel data at full rate (= sim update rate)
  SendSensorMessages();

  // Send groudntruth at full rate
  SendGroundTruth();

  if (close_conn_) { // close connection if required
    mavlink_interface_->close();
  }

  handle_actuator_controls();

  handle_control(dt);

  if (received_first_actuator_) {
    mav_msgs::msgs::CommandMotorSpeed turning_velocities_msg;

    for (int i = 0; i < input_reference_.size(); i++) {
      if (last_actuator_time_ == 0 || (current_time - last_actuator_time_).Double() > 0.2) {
        turning_velocities_msg.add_motor_speed(0);
      } else {
        turning_velocities_msg.add_motor_speed(input_reference_[i]);
      }
    }
    // TODO Add timestamp and Header
    // turning_velocities_msg->header.stamp.sec = current_time.sec;
    // turning_velocities_msg->header.stamp.nsec = current_time.nsec;

    motor_velocity_reference_pub_->Publish(turning_velocities_msg);
  }

  last_time_ = current_time;
}

template <class T>
void GazeboMavlinkInterface::setMavlinkSensorOrientation(const ignition::math::Vector3d& u_Xs, T& sensor_msg) {
  const ignition::math::Vector3d u_Xb = kForwardRotation; // This is unit vector of X-axis `base_link`

  // Current rotation types are described as https://mavlink.io/en/messages/common.html#MAV_SENSOR_ORIENTATION
  if(u_Xs.Dot(kDownwardRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270;
  else if(u_Xs.Dot(kUpwardRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_90;
  else if(u_Xs.Dot(kBackwardRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_180;
  else if(u_Xs.Dot(kForwardRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_NONE;
  else if(u_Xs.Dot(kLeftRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_270;
  else if(u_Xs.Dot(kRightRotation) > 0.99)
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_YAW_90;
  else {
    sensor_msg.orientation = MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_CUSTOM;
  }

}

void GazeboMavlinkInterface::ImuCallback(ImuPtr& imu_message)
{
  std::unique_lock<std::mutex> lock(last_imu_message_mutex_);

  const int64_t diff = imu_message->seq() - last_imu_message_.seq();
  if (diff != 1 && imu_message->seq() != 0)
  {
    gzerr << "Skipped " << (diff - 1) << " IMU samples (presumably CPU usage is too high)\n";
  }

  last_imu_message_ = *imu_message;
  lock.unlock();
  last_imu_message_cond_.notify_one();
}

void GazeboMavlinkInterface::SendSensorMessages()
{
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    last_imu_message_.orientation().w(),
    last_imu_message_.orientation().x(),
    last_imu_message_.orientation().y(),
    last_imu_message_.orientation().z());

  bool should_send_imu = false;
  if (!enable_lockstep_) {
#if GAZEBO_MAJOR_VERSION >= 9
    common::Time current_time = world_->SimTime();
#else
    common::Time current_time = world_->GetSimTime();
#endif
    double dt = (current_time - last_imu_time_).Double();

    if (imu_update_interval_!=0 && dt >= imu_update_interval_) {
      should_send_imu = true;
      last_imu_time_ = current_time;
    }
  }

#if GAZEBO_MAJOR_VERSION >= 9
  uint64_t time_usec = std::llround(world_->SimTime().Double() * 1e6);
#else
  uint64_t time_usec = std::llround(world_->GetSimTime().Double() * 1e6);
#endif

  // send always accel and gyro data (not dependent of the bitmask)
  // required so to keep the timestamps on sync and the lockstep can
  // work properly
  ignition::math::Vector3d accel_b = q_FLU_to_FRD.RotateVector(ignition::math::Vector3d(
    last_imu_message_.linear_acceleration().x(),
    last_imu_message_.linear_acceleration().y(),
    last_imu_message_.linear_acceleration().z()));

  ignition::math::Vector3d gyro_b = q_FLU_to_FRD.RotateVector(ignition::math::Vector3d(
    last_imu_message_.angular_velocity().x(),
    last_imu_message_.angular_velocity().y(),
    last_imu_message_.angular_velocity().z()));

  SensorData::Imu imu_data;
  imu_data.accel_b = Eigen::Vector3d(accel_b.X(), accel_b.Y(), accel_b.Z());
  imu_data.gyro_b = Eigen::Vector3d(gyro_b.X(), gyro_b.Y(), gyro_b.Z());
  mavlink_interface_->UpdateIMU(imu_data);

  mavlink_interface_->SendSensorMessages(time_usec);
}

void GazeboMavlinkInterface::SendGroundTruth()
{
  // ground truth
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    last_imu_message_.orientation().w(),
    last_imu_message_.orientation().x(),
    last_imu_message_.orientation().y(),
    last_imu_message_.orientation().z());

  ignition::math::Quaterniond q_nb = q_ENU_to_NED * q_gr * q_FLU_to_FRD.Inverse();

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel_b = q_FLU_to_FRD.RotateVector(model_->RelativeLinearVel());
  ignition::math::Vector3d vel_n = q_ENU_to_NED.RotateVector(model_->WorldLinearVel());
  ignition::math::Vector3d omega_nb_b = q_FLU_to_FRD.RotateVector(model_->RelativeAngularVel());
#else
  ignition::math::Vector3d vel_b = q_FLU_to_FRD.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearVel()));
  ignition::math::Vector3d vel_n = q_ENU_to_NED.RotateVector(ignitionFromGazeboMath(model_->GetWorldLinearVel()));
  ignition::math::Vector3d omega_nb_b = q_FLU_to_FRD.RotateVector(ignitionFromGazeboMath(model_->GetRelativeAngularVel()));
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d accel_true_b = q_FLU_to_FRD.RotateVector(model_->RelativeLinearAccel());
#else
  ignition::math::Vector3d accel_true_b = q_FLU_to_FRD.RotateVector(ignitionFromGazeboMath(model_->GetRelativeLinearAccel()));
#endif

  // send ground truth
  mavlink_hil_state_quaternion_t hil_state_quat;
#if GAZEBO_MAJOR_VERSION >= 9
  hil_state_quat.time_usec = std::round(world_->SimTime().Double() * 1e6);
#else
  hil_state_quat.time_usec = std::round(world_->GetSimTime().Double() * 1e6);
#endif
  hil_state_quat.attitude_quaternion[0] = q_nb.W();
  hil_state_quat.attitude_quaternion[1] = q_nb.X();
  hil_state_quat.attitude_quaternion[2] = q_nb.Y();
  hil_state_quat.attitude_quaternion[3] = q_nb.Z();

  hil_state_quat.rollspeed = omega_nb_b.X();
  hil_state_quat.pitchspeed = omega_nb_b.Y();
  hil_state_quat.yawspeed = omega_nb_b.Z();

  hil_state_quat.lat = groundtruth_lat_rad_ * 180 / M_PI * 1e7;
  hil_state_quat.lon = groundtruth_lon_rad_ * 180 / M_PI * 1e7;
  hil_state_quat.alt = groundtruth_altitude_ * 1000;

  hil_state_quat.vx = vel_n.X() * 100;
  hil_state_quat.vy = vel_n.Y() * 100;
  hil_state_quat.vz = vel_n.Z() * 100;

  // assumed indicated airspeed due to flow aligned with pitot (body x)
  hil_state_quat.ind_airspeed = vel_b.X();

#if GAZEBO_MAJOR_VERSION >= 9
  hil_state_quat.true_airspeed = (model_->WorldLinearVel() -  wind_vel_).Length() * 100;
#else
  hil_state_quat.true_airspeed = (model_->GetWorldLinearVel() -  wind_vel_).GetLength() * 100;
#endif

  hil_state_quat.xacc = accel_true_b.X() * 1000;
  hil_state_quat.yacc = accel_true_b.Y() * 1000;
  hil_state_quat.zacc = accel_true_b.Z() * 1000;

  if (!hil_mode_ || (hil_mode_ && hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_state_quat);
    mavlink_interface_->send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::GpsCallback(GpsPtr& gps_msg, const int& id) {
  SensorData::Gps gps_data;
  gps_data.time_utc_usec = gps_msg->time_utc_usec();
  gps_data.fix_type = 3;
  gps_data.latitude_deg = gps_msg->latitude_deg() * 1e7;
  gps_data.longitude_deg = gps_msg->longitude_deg() * 1e7;
  gps_data.altitude = gps_msg->altitude() * 1000.0;
  gps_data.eph = gps_msg->eph() * 100.0;
  gps_data.epv = gps_msg->epv() * 100.0;
  gps_data.velocity = gps_msg->velocity() * 100.0;
  gps_data.velocity_north = gps_msg->velocity_north() * 100.0;
  gps_data.velocity_east = gps_msg->velocity_east() * 100.0;
  gps_data.velocity_down = -gps_msg->velocity_up() * 100.0;
  // MAVLINK_HIL_GPS_T CoG is [0, 360]. math::Angle::Normalize() is [-pi, pi].
  ignition::math::Angle cog(atan2(gps_msg->velocity_east(), gps_msg->velocity_north()));
  cog.Normalize();
  gps_data.cog = static_cast<uint16_t>(GetDegrees360(cog) * 100.0);
  gps_data.satellites_visible = 10;
  gps_data.id = id;

  mavlink_interface_->SendGpsMessages(gps_data);
}

void GazeboMavlinkInterface::GroundtruthCallback(GtPtr& groundtruth_msg) {
  // update groundtruth lat_rad, lon_rad and altitude
  groundtruth_lat_rad_ = groundtruth_msg->latitude_rad();
  groundtruth_lon_rad_ = groundtruth_msg->longitude_rad();
  groundtruth_altitude_ = groundtruth_msg->altitude();
  // the rest of the data is obtained directly on this interface and sent to
  // the FCU
}

void GazeboMavlinkInterface::LidarCallback(LidarPtr& lidar_message, const int& id) {
  mavlink_distance_sensor_t sensor_msg;
  sensor_msg.time_boot_ms = lidar_message->time_usec() / 1e3;   // [ms]
  sensor_msg.min_distance = lidar_message->min_distance() * 100.0;  // [cm]
  sensor_msg.max_distance = lidar_message->max_distance() * 100.0;  // [cm]
  sensor_msg.current_distance = lidar_message->current_distance() * 100.0;  // [cm]
  sensor_msg.type = 0;
  sensor_msg.id = id;
  sensor_msg.covariance = 0;
  sensor_msg.horizontal_fov = lidar_message->h_fov();
  sensor_msg.vertical_fov = lidar_message->v_fov();
  sensor_msg.signal_quality = lidar_message->signal_quality();

  ignition::math::Quaterniond q_ls = ignition::math::Quaterniond(
    lidar_message->orientation().w(),
    lidar_message->orientation().x(),
    lidar_message->orientation().y(),
    lidar_message->orientation().z());

  ignition::math::Quaterniond q_bs;
  for (Sensor_M::iterator it = sensor_map_.begin(); it != sensor_map_.end(); ++it) {
    // check the ID of the sensor on the sensor map and apply the respective rotation
    if (it->second.first == id) {
      q_bs = (it->second.second * q_ls).Inverse();
    }
  }

  sensor_msg.quaternion[0] = q_bs.W();
  sensor_msg.quaternion[1] = q_bs.X();
  sensor_msg.quaternion[2] = q_bs.Y();
  sensor_msg.quaternion[3] = q_bs.Z();

  const ignition::math::Vector3d u_Xb = kForwardRotation; // This is unit vector of X-axis `base_link`
  const ignition::math::Vector3d u_Xs = q_bs.RotateVectorReverse(u_Xb); // This is unit vector of X-axis sensor in `base_link` frame

  setMavlinkSensorOrientation(u_Xs, sensor_msg);

  // distance needed for optical flow message
  if (sensor_msg.orientation == MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270) {
    optflow_distance_ = lidar_message->current_distance();  // [m]
  }

  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  mavlink_interface_->send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::OpticalFlowCallback(OpticalFlowPtr& opticalFlow_message) {
  mavlink_hil_optical_flow_t sensor_msg;
  sensor_msg.time_usec = opticalFlow_message->time_usec();
  sensor_msg.sensor_id = opticalFlow_message->sensor_id();
  sensor_msg.integration_time_us = opticalFlow_message->integration_time_us();
  sensor_msg.integrated_x = opticalFlow_message->integrated_x();
  sensor_msg.integrated_y = opticalFlow_message->integrated_y();

  bool no_gyro = (ignition::math::isnan(opticalFlow_message->integrated_xgyro())) ||
                 (ignition::math::isnan(opticalFlow_message->integrated_ygyro())) ||
                 (ignition::math::isnan(opticalFlow_message->integrated_zgyro()));
  if (no_gyro) {
    sensor_msg.integrated_xgyro = NAN;
    sensor_msg.integrated_ygyro = NAN;
    sensor_msg.integrated_zgyro = NAN;
  } else {
    sensor_msg.integrated_xgyro = opticalFlow_message->quality() ? opticalFlow_message->integrated_xgyro() : 0.0f;
    sensor_msg.integrated_ygyro = opticalFlow_message->quality() ? opticalFlow_message->integrated_ygyro() : 0.0f;
    sensor_msg.integrated_zgyro = opticalFlow_message->quality() ? opticalFlow_message->integrated_zgyro() : 0.0f;
  }
  sensor_msg.temperature = opticalFlow_message->temperature();
  sensor_msg.quality = opticalFlow_message->quality();
  sensor_msg.time_delta_distance_us = opticalFlow_message->time_delta_distance_us();
  sensor_msg.distance = optflow_distance_;

  mavlink_message_t msg;
  mavlink_msg_hil_optical_flow_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  mavlink_interface_->send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::SonarCallback(SonarPtr& sonar_message, const int& id) {
  mavlink_distance_sensor_t sensor_msg = {};
  sensor_msg.time_boot_ms = sonar_message->time_usec() / 1e3;
  sensor_msg.min_distance = sonar_message->min_distance() * 100.0;
  sensor_msg.max_distance = sonar_message->max_distance() * 100.0;
  sensor_msg.current_distance = sonar_message->current_distance() * 100.0;
  sensor_msg.signal_quality = sonar_message->signal_quality();

  ignition::math::Quaterniond q_ls;
  for (Sensor_M::iterator it = sensor_map_.begin(); it != sensor_map_.end(); ++it) {
    // check the ID of the sensor on the sensor map and apply the respective rotation
    if (it->second.first == id) {
      q_ls = (it->second.second).Inverse();
    }
  }

  const ignition::math::Vector3d u_Xb = kForwardRotation; // This is unit vector of X-axis `base_link`
  const ignition::math::Vector3d u_Xs = q_ls.RotateVectorReverse(u_Xb); // This is unit vector of X-axis sensor in `base_link` frame

  setMavlinkSensorOrientation(u_Xs, sensor_msg);

  sensor_msg.type = 1;
  sensor_msg.id = 100 + id; // to differentiate from Lidars
  sensor_msg.covariance = 0;
  sensor_msg.horizontal_fov = sonar_message->h_fov();
  sensor_msg.vertical_fov = sonar_message->v_fov();
  sensor_msg.quaternion[0] = q_ls.W();
  sensor_msg.quaternion[1] = q_ls.X();
  sensor_msg.quaternion[2] = q_ls.Y();
  sensor_msg.quaternion[3] = q_ls.Z();

  // distance needed for optical flow message
  if (sensor_msg.orientation == MAV_SENSOR_ORIENTATION::MAV_SENSOR_ROTATION_PITCH_270) {
    optflow_distance_ = sonar_message->current_distance();  // [m]
  }

  mavlink_message_t msg;
  mavlink_msg_distance_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  mavlink_interface_->send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::IRLockCallback(IRLockPtr& irlock_message) {
  mavlink_landing_target_t sensor_msg;
  sensor_msg.time_usec = irlock_message->time_usec() / 1e3;
  sensor_msg.target_num = irlock_message->signature();
  sensor_msg.angle_x = irlock_message->pos_x();
  sensor_msg.angle_y = irlock_message->pos_y();
  sensor_msg.size_x = irlock_message->size_x();
  sensor_msg.size_y = irlock_message->size_y();
  sensor_msg.position_valid = false;
  sensor_msg.type = LANDING_TARGET_TYPE_LIGHT_BEACON;

  mavlink_message_t msg;
  mavlink_msg_landing_target_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  mavlink_interface_->send_mavlink_message(&msg);
}

void GazeboMavlinkInterface::VisionCallback(OdomPtr& odom_message) {
  mavlink_message_t msg;

  // transform position from local ENU to local NED frame
  ignition::math::Vector3d position = q_ENU_to_NED.RotateVector(ignition::math::Vector3d(
    odom_message->position().x(),
    odom_message->position().y(),
    odom_message->position().z()));

  // q_gr is the quaternion that represents the orientation of the vehicle
  // the ENU earth/local
  ignition::math::Quaterniond q_gr = ignition::math::Quaterniond(
    odom_message->orientation().w(),
    odom_message->orientation().x(),
    odom_message->orientation().y(),
    odom_message->orientation().z());

  // transform the vehicle orientation from the ENU to the NED frame
  // q_nb is the quaternion that represents the orientation of the vehicle
  // the NED earth/local
  ignition::math::Quaterniond q_nb = q_ENU_to_NED * q_gr * q_FLU_to_FRD.Inverse();

  // transform linear velocity from local ENU to body FRD frame
  ignition::math::Vector3d linear_velocity = q_FLU_to_FRD.RotateVector(
    q_gr.Inverse().RotateVector(ignition::math::Vector3d(
      odom_message->linear_velocity().x(),
      odom_message->linear_velocity().y(),
      odom_message->linear_velocity().z())));

  // transform angular velocity from body FLU to body FRD frame
  ignition::math::Vector3d angular_velocity = q_FLU_to_FRD.RotateVector(ignition::math::Vector3d(
    odom_message->angular_velocity().x(),
    odom_message->angular_velocity().y(),
    odom_message->angular_velocity().z()));

  // Only sends ODOMETRY msgs if send_odometry is set and the protocol version is 2.0
  if (send_odometry_ && protocol_version_ == 2.0) {
    // send ODOMETRY Mavlink msg
    mavlink_odometry_t odom;

    odom.time_usec = odom_message->time_usec();

    odom.frame_id = MAV_FRAME_LOCAL_NED;
    odom.child_frame_id = MAV_FRAME_BODY_FRD;

    odom.estimator_type = MAV_ESTIMATOR_TYPE_VISION;

    odom.x = position.X();
    odom.y = position.Y();
    odom.z = position.Z();

    odom.q[0] = q_nb.W();
    odom.q[1] = q_nb.X();
    odom.q[2] = q_nb.Y();
    odom.q[3] = q_nb.Z();

    odom.vx = linear_velocity.X();
    odom.vy = linear_velocity.Y();
    odom.vz = linear_velocity.Z();

    odom.rollspeed= angular_velocity.X();
    odom.pitchspeed = angular_velocity.Y();
    odom.yawspeed = angular_velocity.Z();

    // Parse covariance matrices
    // The main diagonal values are always positive (variance), so a transform
    // in the covariance matrices from one frame to another would only
    // change the values of the main diagonal. Since they are all zero,
    // there's no need to apply the rotation
    size_t count = 0;
    for (size_t x = 0; x < 6; x++) {
      for (size_t y = x; y < 6; y++) {
        size_t index = 6 * x + y;

        odom.pose_covariance[count] = odom_message->pose_covariance().data()[index];
        odom.velocity_covariance[count] = odom_message->velocity_covariance().data()[index];
        count++;
      }
    }

    mavlink_msg_odometry_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &odom);
    mavlink_interface_->send_mavlink_message(&msg);
  }
  else if (send_vision_estimation_) {
    // send VISION_POSITION_ESTIMATE Mavlink msg
    mavlink_vision_position_estimate_t vision;

    vision.usec = odom_message->time_usec();

    // transform position from local ENU to local NED frame
    vision.x = position.X();
    vision.y = position.Y();
    vision.z = position.Z();

    // q_nb is the quaternion that represents a rotation from NED earth/local
    // frame to XYZ body FRD frame
    ignition::math::Vector3d euler = q_nb.Euler();

    vision.roll = euler.X();
    vision.pitch = euler.Y();
    vision.yaw = euler.Z();

    // parse covariance matrix
    // The main diagonal values are always positive (variance), so a transform
    // in the covariance matrix from one frame to another would only
    // change the values of the main diagonal. Since they are all zero,
    // there's no need to apply the rotation
    size_t count = 0;
    for (size_t x = 0; x < 6; x++) {
      for (size_t y = x; y < 6; y++) {
        size_t index = 6 * x + y;

        vision.covariance[count++] = odom_message->pose_covariance().data()[index];
      }
    }

    mavlink_msg_vision_position_estimate_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &vision);
    mavlink_interface_->send_mavlink_message(&msg);
  }
}

void GazeboMavlinkInterface::MagnetometerCallback(MagnetometerPtr& mag_msg) {
  SensorData::Magnetometer mag_data;
  mag_data.mag_b = Eigen::Vector3d(mag_msg->magnetic_field().x(),
    mag_msg->magnetic_field().y(), mag_msg->magnetic_field().z());
  mavlink_interface_->UpdateMag(mag_data);
}

void GazeboMavlinkInterface::AirspeedCallback(AirspeedPtr& airspeed_msg, const int& id) {
  SensorData::Airspeed airspeed_data;
  airspeed_data.diff_pressure = airspeed_msg->diff_pressure();
  mavlink_interface_->UpdateAirspeed(airspeed_data, id);
}

void GazeboMavlinkInterface::BarometerCallback(BarometerPtr& baro_msg) {
  SensorData::Barometer baro_data;
  baro_data.temperature = baro_msg->temperature();
  baro_data.abs_pressure = baro_msg->absolute_pressure();
  baro_data.pressure_alt = baro_msg->pressure_altitude();
  mavlink_interface_->UpdateBarometer(baro_data);
}

void GazeboMavlinkInterface::WindVelocityCallback(WindPtr& msg) {
  wind_vel_ = ignition::math::Vector3d(msg->velocity().x(),
            msg->velocity().y(),
            msg->velocity().z());
}

void GazeboMavlinkInterface::handle_actuator_controls() {
  bool armed = mavlink_interface_->GetArmedState();

  #if GAZEBO_MAJOR_VERSION >= 9
      last_actuator_time_ = world_->SimTime();
  #else
      last_actuator_time_ = world_->GetSimTime();
  #endif

  for (unsigned i = 0; i < n_out_max; i++) {
    input_index_[i] = i;
  }
  // Read Input References
  input_reference_.resize(n_out_max);

  Eigen::VectorXd actuator_controls = mavlink_interface_->GetActuatorControls();
  if (actuator_controls.size() < n_out_max) return; //TODO: Handle this properly
  for (int i = 0; i < input_reference_.size(); i++) {
    if (armed) {
      input_reference_[i] = (actuator_controls[input_index_[i]] + input_offset_[i])
          * input_scaling_[i] + zero_position_armed_[i];
      // std::cout << input_reference_ << ", ";
    } else {
      input_reference_[i] = zero_position_disarmed_[i];
      // std::cout << input_reference_ << ", ";
    }
  }
  // std::cout << "Input Reference: " << input_reference_.transpose() << std::endl;
  received_first_actuator_ = mavlink_interface_->GetReceivedFirstActuator();
}

void GazeboMavlinkInterface::handle_control(double _dt)
{
  // set joint positions
  for (int i = 0; i < input_reference_.size(); i++) {
    if (joints_[i] || joint_control_type_[i] == "position_gztopic") {
      double target = input_reference_[i];
      if (joint_control_type_[i] == "velocity")
      {
        double current = joints_[i]->GetVelocity(0);
        double err = current - target;
        double force = pids_[i].Update(err, _dt);
        joints_[i]->SetForce(0, force);
      }
      else if (joint_control_type_[i] == "position")
      {

#if GAZEBO_MAJOR_VERSION >= 9
        double current = joints_[i]->Position(0);
#else
        double current = joints_[i]->GetAngle(0).Radian();
#endif

        double err = current - target;
        if(joint_max_errors_[i]!=0.) {
          err = std::max(std::min(err, joint_max_errors_[i]), -joint_max_errors_[i]);
        }
        double force = pids_[i].Update(err, _dt);
        joints_[i]->SetForce(0, force);
      }
      else if (joint_control_type_[i] == "position_gztopic")
      {
     #if GAZEBO_MAJOR_VERSION > 7 || (GAZEBO_MAJOR_VERSION == 7 && GAZEBO_MINOR_VERSION >= 4)
        /// only gazebo 7.4 and above support Any
        gazebo::msgs::Any m;
        m.set_type(gazebo::msgs::Any_ValueType_DOUBLE);
        m.set_double_value(target);
     #else
        std::stringstream ss;
        gazebo::msgs::GzString m;
        ss << target;
        m.set_data(ss.str());
     #endif
        joint_control_pub_[i]->Publish(m);
      }
      else if (joint_control_type_[i] == "position_kinematic")
      {
        /// really not ideal if your drone is moving at all,
        /// mixing kinematic updates with dynamics calculation is
        /// non-physical.
     #if GAZEBO_MAJOR_VERSION >= 6
        joints_[i]->SetPosition(0, input_reference_[i]);
     #else
        joints_[i]->SetAngle(0, input_reference_[i]);
     #endif
      }
      else
      {
        gzerr << "joint_control_type[" << joint_control_type_[i] << "] undefined.\n";
      }
    }
  }
}

bool GazeboMavlinkInterface::IsRunning()
{
#if GAZEBO_MAJOR_VERSION >= 8
    return world_->Running();
#else
    return world_->GetRunning();
#endif
}
void GazeboMavlinkInterface::onSigInt() {
  mavlink_interface_->onSigInt();
}

}
