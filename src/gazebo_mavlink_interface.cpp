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


#include "gazebo_mavlink_interface.h"
#include "geo_mag_declination.h"

#define UDP_PORT 14560
#define UDP_PORT_2 14556

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkInterface);

GazeboMavlinkInterface::~GazeboMavlinkInterface() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboMavlinkInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  namespace_.clear();
  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_mavlink_interface] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_,
                           motor_velocity_reference_pub_topic_);

  joints_.resize(n_out_max);

  if (_sdf->HasElement("control_channels")) {
    sdf::ElementPtr cmapping = _sdf->GetElement("control_channels");

    // sdf::ElementPtr_V children = cmapping->elements;

    // for (unsigned i = 0; i < children.size(); i++) {
    //   std::string name = children.at(i).Get<std::string>();

    //   if (name == std::string("channel")) {
    //     //int input_index = _sdf->GetElement("input_index")->Get<int>();
    //     input_offset[i] = _sdf->GetElement("input_offset")->Get<double>();
    //     input_scaling[i] = _sdf->GetElement("input_scaling")->Get<double>();
    //   }
    // }
  }

  if (_sdf->HasElement("left_elevon_joint")) {
    std::string left_elevon_joint_name = _sdf->GetElement("left_elevon_joint")->Get<std::string>();
    left_elevon_joint_ = model_->GetJoint(left_elevon_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("left_elevon_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = left_elevon_joint_;
    }
  }

  if (_sdf->HasElement("left_aileron_joint")) {
    std::string left_elevon_joint_name = _sdf->GetElement("left_aileron_joint")->Get<std::string>();
    left_elevon_joint_ = model_->GetJoint(left_elevon_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("left_aileron_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = left_elevon_joint_;
    }
  }

  if (_sdf->HasElement("right_elevon_joint")) {
    std::string right_elevon_joint_name = _sdf->GetElement("right_elevon_joint")->Get<std::string>();
    right_elevon_joint_ = model_->GetJoint(right_elevon_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("right_elevon_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = right_elevon_joint_;
    }
  }

  if (_sdf->HasElement("right_aileron_joint")) {
    std::string right_elevon_joint_name = _sdf->GetElement("right_aileron_joint")->Get<std::string>();
    right_elevon_joint_ = model_->GetJoint(right_elevon_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("right_aileron_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = right_elevon_joint_;
    }
  }

  if (_sdf->HasElement("elevator_joint")) {
    std::string elevator_joint_name = _sdf->GetElement("elevator_joint")->Get<std::string>();
    elevator_joint_ = model_->GetJoint(elevator_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("elevator_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = elevator_joint_;
    }
  }

  if (_sdf->HasElement("propeller_joint")) {
    std::string propeller_joint_name = _sdf->GetElement("propeller_joint")->Get<std::string>();
    propeller_joint_ = model_->GetJoint(propeller_joint_name);
  }

  if (_sdf->HasElement("cgo3_mount_joint")) {
    std::string gimbal_yaw_joint_name = _sdf->GetElement("cgo3_mount_joint")->Get<std::string>();
    gimbal_yaw_joint_ = model_->GetJoint(gimbal_yaw_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("cgo3_mount_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = gimbal_yaw_joint_;
    }
  }

  if (_sdf->HasElement("cgo3_vertical_arm_joint")) {
    std::string gimbal_roll_joint_name = _sdf->GetElement("cgo3_vertical_arm_joint")->Get<std::string>();
    gimbal_roll_joint_ = model_->GetJoint(gimbal_roll_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("cgo3_vertical_arm_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = gimbal_roll_joint_;
    }
  }

  if (_sdf->HasElement("cgo3_horizontal_arm_joint")) {
    std::string gimbal_pitch_joint_name = _sdf->GetElement("cgo3_horizontal_arm_joint")->Get<std::string>();
    gimbal_pitch_joint_ = model_->GetJoint(gimbal_pitch_joint_name);
    int control_index;
    getSdfParam<int>(_sdf->GetElement("cgo3_horizontal_arm_joint"), "controlIndex", control_index, -1);
    if (control_index >= 0) {
      joints_.at(control_index) = gimbal_pitch_joint_;
    }
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboMavlinkInterface::OnUpdate, this, _1));

  // Subscriber to IMU sensor_msgs::Imu Message and SITL's HilControl message
  mav_control_sub_ = node_handle_->Subscribe(mavlink_control_sub_topic_, &GazeboMavlinkInterface::HilControlCallback, this);
  imu_sub_ = node_handle_->Subscribe(imu_sub_topic_, &GazeboMavlinkInterface::ImuCallback, this);
  lidar_sub_ = node_handle_->Subscribe(lidar_sub_topic_, &GazeboMavlinkInterface::LidarCallback, this);
  opticalFlow_sub_ = node_handle_->Subscribe(opticalFlow_sub_topic_, &GazeboMavlinkInterface::OpticalFlowCallback, this);
  
  // Publish HilSensor Message and gazebo's motor_speed message
  motor_velocity_reference_pub_ = node_handle_->Advertise<mav_msgs::msgs::CommandMotorSpeed>(motor_velocity_reference_pub_topic_, 1);
  hil_sensor_pub_ = node_handle_->Advertise<mavlink::msgs::HilSensor>(hil_sensor_mavlink_pub_topic_, 1);
  hil_gps_pub_ = node_handle_->Advertise<mavlink::msgs::HilGps>(hil_gps_mavlink_pub_topic_, 1);

  _rotor_count = 5;
  last_time_ = world_->GetSimTime();
  last_gps_time_ = world_->GetSimTime();
  gps_update_interval_ = 0.2;  // in seconds for 5Hz

  gravity_W_ = world_->GetPhysicsEngine()->GetGravity();

  // Magnetic field data for Zurich from WMM2015 (10^5xnanoTesla (N, E, D))
  //mag_W_ = {0.21523, 0.00771, 0.42741};
  mag_W_.x = 0.21523;
  // We set the world Y component to zero because we apply
  // the declination based on the global position,
  // and so we need to start without any offsets.
  // The real value for Zurich would be 0.00771
  mag_W_.y = 0.0;
  mag_W_.z = 0.42741;

  //Create socket
  // udp socket data
  const int _port = UDP_PORT;

  // try to setup udp socket for communcation with simulator
  if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    printf("create socket failed\n");
    return;
  }

  memset((char *)&_myaddr, 0, sizeof(_myaddr));
  _myaddr.sin_family = AF_INET;
  _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  // Let the OS pick the port
  _myaddr.sin_port = htons(0);

  if (bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
    printf("bind failed\n");
    return;
  }

  _srcaddr.sin_family = AF_INET;
  _srcaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  _srcaddr.sin_port = htons(UDP_PORT);

  _srcaddr_2.sin_family = AF_INET;
  _srcaddr_2.sin_addr.s_addr = htonl(INADDR_ANY);
  _srcaddr_2.sin_port = htons(UDP_PORT_2);

  _addrlen = sizeof(_srcaddr);

  fds[0].fd = _fd;
  fds[0].events = POLLIN;
}

// This gets called by the world update start event.
void GazeboMavlinkInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {

  pollForMAVLinkMessages();

  common::Time now = world_->GetSimTime();

  if(received_first_referenc_) {

    mav_msgs::msgs::CommandMotorSpeed turning_velocities_msg;

    for (int i = 0; i < input_reference_.size(); i++){
      if (last_actuator_time_ == 0 || (now - last_actuator_time_).Double() > 0.2) {
        turning_velocities_msg.add_motor_speed(0);
      } else {
        turning_velocities_msg.add_motor_speed(input_reference_[i]);
      }
    }
    // TODO Add timestamp and Header
    // turning_velocities_msg->header.stamp.sec = now.sec;
    // turning_velocities_msg->header.stamp.nsec = now.nsec;

    // gzerr << turning_velocities_msg.motor_speed(0) << "\n";
    motor_velocity_reference_pub_->Publish(turning_velocities_msg);
  }

  //send gps
  common::Time current_time  = now;
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

  math::Pose T_W_I = model_->GetWorldPose(); //TODO(burrimi): Check tf.
  math::Vector3 pos_W_I = T_W_I.pos;  // Use the models' world position for GPS and pressure alt.

  math::Vector3 velocity_current_W = model_->GetWorldLinearVel();  // Use the models' world position for GPS velocity.

  math::Vector3 velocity_current_W_xy = velocity_current_W;
  velocity_current_W_xy.z = 0;

  // Set global reference point
  // Zurich Irchel Park: 47.397742, 8.545594, 488m
  // Seattle downtown (15 deg declination): 47.592182, -122.316031, 86m
  // Moscow downtown: 55.753395, 37.625427, 155m

  // TODO: Remove GPS message from IMU plugin. Added gazebo GPS plugin. This is temp here.
  // Zurich Irchel Park
  const double lat_zurich = 47.397742 * M_PI / 180;  // rad
  const double lon_zurich = 8.545594 * M_PI / 180;  // rad
  const double alt_zurich = 488.0; // meters
  // Seattle downtown (15 deg declination): 47.592182, -122.316031
  // const double lat_zurich = 47.592182 * M_PI / 180;  // rad
  // const double lon_zurich = -122.316031 * M_PI / 180;  // rad
  // const double alt_zurich = 86.0; // meters
  const float earth_radius = 6353000;  // m

  // reproject local position to gps coordinates
  double x_rad = pos_W_I.x / earth_radius;
  double y_rad = -pos_W_I.y / earth_radius;
  double c = sqrt(x_rad * x_rad + y_rad * y_rad);
  double sin_c = sin(c);
  double cos_c = cos(c);
  if (c != 0.0) {
    lat_rad = asin(cos_c * sin(lat_zurich) + (x_rad * sin_c * cos(lat_zurich)) / c);
    lon_rad = (lon_zurich + atan2(y_rad * sin_c, c * cos(lat_zurich) * cos_c - x_rad * sin(lat_zurich) * sin_c));
  } else {
   lat_rad = lat_zurich;
    lon_rad = lon_zurich;
  }
  
  if(current_time.Double() - last_gps_time_.Double() > gps_update_interval_){  // 5Hz

    if(use_mavlink_udp){
      // Raw UDP mavlink
      mavlink_hil_gps_t hil_gps_msg;
      hil_gps_msg.time_usec = current_time.nsec*1000;
      hil_gps_msg.fix_type = 3;
      hil_gps_msg.lat = lat_rad * 180 / M_PI * 1e7;
      hil_gps_msg.lon = lon_rad * 180 / M_PI * 1e7;
      hil_gps_msg.alt = (pos_W_I.z + alt_zurich) * 1000;
      hil_gps_msg.eph = 100;
      hil_gps_msg.epv = 100;
      hil_gps_msg.vel = velocity_current_W_xy.GetLength() * 100;
      hil_gps_msg.vn = velocity_current_W.x * 100;
      hil_gps_msg.ve = -velocity_current_W.y * 100;
      hil_gps_msg.vd = -velocity_current_W.z * 100;
      hil_gps_msg.cog = atan2(hil_gps_msg.ve, hil_gps_msg.vn) * 180.0/3.1416 * 100.0;
      hil_gps_msg.satellites_visible = 10;

      send_mavlink_message(MAVLINK_MSG_ID_HIL_GPS, &hil_gps_msg, 200);
    } else{
      // Send via protobuf
      hil_gps_msg_.set_time_usec(current_time.nsec*1000);
      hil_gps_msg_.set_fix_type(3);
      hil_gps_msg_.set_lat(lat_rad * 180 / M_PI * 1e7);
      hil_gps_msg_.set_lon(lon_rad * 180 / M_PI * 1e7);
      hil_gps_msg_.set_alt((pos_W_I.z + alt_zurich) * 1000);
      hil_gps_msg_.set_eph(100);
      hil_gps_msg_.set_epv(100);
      hil_gps_msg_.set_vel(velocity_current_W_xy.GetLength() * 100);
      hil_gps_msg_.set_vn(velocity_current_W.x * 100);
      hil_gps_msg_.set_ve(-velocity_current_W.y * 100);
      hil_gps_msg_.set_vd(-velocity_current_W.z * 100);
      hil_gps_msg_.set_cog(atan2(-velocity_current_W.y * 100, velocity_current_W.x * 100) * 180.0/3.1416 * 100.0);
      hil_gps_msg_.set_satellites_visible(10);
             
      hil_gps_pub_->Publish(hil_gps_msg_);
    }

    last_gps_time_ = current_time;
  }
}

void GazeboMavlinkInterface::HilControlCallback(HilControlPtr &rmsg) {
  if(!use_mavlink_udp){

    inputs.control[0] =(double)rmsg->roll_ailerons();
    inputs.control[1] =(double)rmsg->pitch_elevator();
    inputs.control[2] =(double)rmsg->yaw_rudder();
    inputs.control[3] =(double)rmsg->throttle();
    inputs.control[4] =(double)rmsg->aux1();
    inputs.control[5] =(double)rmsg->aux2();
    inputs.control[6] =(double)rmsg->aux3();
    inputs.control[7] =(double)rmsg->aux4();

    // publish message
    double scaling = 150;
    double offset = 600;

    mav_msgs::msgs::CommandMotorSpeed* turning_velocities_msg = new mav_msgs::msgs::CommandMotorSpeed;

    for (int i = 0; i < _rotor_count; i++) {
      turning_velocities_msg->add_motor_speed(inputs.control[i] * scaling + offset);
    }

    input_reference_.resize(turning_velocities_msg->motor_speed_size());
    for (int i = 0; i < turning_velocities_msg->motor_speed_size(); ++i) {
      input_reference_[i] = turning_velocities_msg->motor_speed(i);
    }
    received_first_referenc_ = true;
  }
}

void GazeboMavlinkInterface::send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID) {
  component_ID = 0;
  uint8_t payload_len = mavlink_message_lengths[msgid];
  unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  /* header */
  buf[0] = MAVLINK_STX;
  buf[1] = payload_len;
  /* no idea which numbers should be here*/
  buf[2] = 100;
  buf[3] = 0;
  buf[4] = component_ID;
  buf[5] = msgid;

  /* payload */
  memcpy(&buf[MAVLINK_NUM_HEADER_BYTES],msg, payload_len);

  /* checksum */
  uint16_t checksum;
  crc_init(&checksum);
  crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
  crc_accumulate(mavlink_message_crcs[msgid], &checksum);

  buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
  buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

  ssize_t len;

  if (msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR || msgid == MAVLINK_MSG_ID_OPTICAL_FLOW_RAD)
    len = sendto(_fd, buf, packet_len, 0, (struct sockaddr *)&_srcaddr_2, sizeof(_srcaddr_2));
  else
    len = sendto(_fd, buf, packet_len, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));

  if (len <= 0) {
    printf("Failed sending mavlink message\n");
  }
}

void GazeboMavlinkInterface::ImuCallback(ImuPtr& imu_message) {

  math::Pose T_W_I = model_->GetWorldPose();
  math::Vector3 pos_W_I = T_W_I.pos;  // Use the models'world position for GPS and pressure alt.
  
  math::Quaternion C_W_I;
  C_W_I.w = imu_message->orientation().w();
  C_W_I.x = imu_message->orientation().x();
  C_W_I.y = imu_message->orientation().y();
  C_W_I.z = imu_message->orientation().z();

  // gzerr << "got imu: " << C_W_I << "\n";
  float declination = get_mag_declination(lat_rad, lon_rad);

  math::Quaternion C_D_I(0.0, 0.0, declination);

  math::Vector3 mag_decl = C_D_I.RotateVectorReverse(mag_W_);

  // TODO replace mag_W_ in the line below with mag_decl

  math::Vector3 mag_I = C_W_I.RotateVectorReverse(mag_decl); // TODO: Add noise based on bais and variance like for imu and gyro
  math::Vector3 body_vel = C_W_I.RotateVectorReverse(model_->GetWorldLinearVel());
  
  standard_normal_distribution_ = std::normal_distribution<float>(0, 0.01f);

  float mag_noise = standard_normal_distribution_(random_generator_);

  if(use_mavlink_udp){
    mavlink_hil_sensor_t sensor_msg;
    sensor_msg.time_usec = world_->GetSimTime().nsec*1000;
    sensor_msg.xacc = imu_message->linear_acceleration().x();
    sensor_msg.yacc = imu_message->linear_acceleration().y();
    sensor_msg.zacc = imu_message->linear_acceleration().z();
    sensor_msg.xgyro = imu_message->angular_velocity().x();
    sensor_msg.ygyro = imu_message->angular_velocity().y();
    sensor_msg.zgyro = imu_message->angular_velocity().z();
    sensor_msg.xmag = mag_I.x + mag_noise;
    sensor_msg.ymag = mag_I.y + mag_noise;
    sensor_msg.zmag = mag_I.z + mag_noise;
    sensor_msg.abs_pressure = 0.0;
    sensor_msg.diff_pressure = 0.5*1.2754*(body_vel.z + body_vel.x)*(body_vel.z + body_vel.x) / 100;
    sensor_msg.pressure_alt = pos_W_I.z;
    sensor_msg.temperature = 0.0;
    sensor_msg.fields_updated = 4095;

    //gyro needed for optical flow message
    optflow_xgyro = imu_message->angular_velocity().x();
    optflow_ygyro = imu_message->angular_velocity().y();
    optflow_zgyro = imu_message->angular_velocity().z();

    send_mavlink_message(MAVLINK_MSG_ID_HIL_SENSOR, &sensor_msg, 200);    
  } else{
    hil_sensor_msg_.set_time_usec(world_->GetSimTime().nsec*1000);
    hil_sensor_msg_.set_xacc(imu_message->linear_acceleration().x());
    hil_sensor_msg_.set_yacc(imu_message->linear_acceleration().y());
    hil_sensor_msg_.set_zacc(imu_message->linear_acceleration().z());
    hil_sensor_msg_.set_xgyro(imu_message->angular_velocity().x());
    hil_sensor_msg_.set_ygyro(imu_message->angular_velocity().y());
    hil_sensor_msg_.set_zgyro(imu_message->angular_velocity().z());
    hil_sensor_msg_.set_xmag(mag_I.x);
    hil_sensor_msg_.set_ymag(mag_I.y);
    hil_sensor_msg_.set_zmag(mag_I.z);
    hil_sensor_msg_.set_abs_pressure(0.0);
    hil_sensor_msg_.set_diff_pressure(0.5*1.2754*body_vel.x*body_vel.x);
    hil_sensor_msg_.set_pressure_alt(pos_W_I.z);
    hil_sensor_msg_.set_temperature(0.0);
    hil_sensor_msg_.set_fields_updated(4095);  // 0b1111111111111 (All updated since new data with new noise added always)
    
    hil_sensor_pub_->Publish(hil_sensor_msg_);
  }
}

void GazeboMavlinkInterface::LidarCallback(LidarPtr& lidar_message) {
  
  mavlink_distance_sensor_t sensor_msg;
  sensor_msg.time_boot_ms = lidar_message->time_msec();
  sensor_msg.min_distance = lidar_message->min_distance() * 100.0;
  sensor_msg.max_distance = lidar_message->max_distance() * 100.0;
  sensor_msg.current_distance = lidar_message->current_distance() * 100.0;
  sensor_msg.type = 0;
  sensor_msg.id = 0;
  sensor_msg.orientation = 0;
  sensor_msg.covariance = 0;

  //distance needed for optical flow message
  optflow_distance = lidar_message->current_distance(); //[m]

  send_mavlink_message(MAVLINK_MSG_ID_DISTANCE_SENSOR, &sensor_msg, 200);

}

void GazeboMavlinkInterface::OpticalFlowCallback(OpticalFlowPtr& opticalFlow_message) {

  mavlink_optical_flow_rad_t sensor_msg;
  sensor_msg.time_usec = opticalFlow_message->time_usec();
  sensor_msg.sensor_id = opticalFlow_message->sensor_id();
  sensor_msg.integration_time_us = opticalFlow_message->integration_time_us();
  sensor_msg.integrated_x = opticalFlow_message->integrated_x();
  sensor_msg.integrated_y = opticalFlow_message->integrated_y();
  sensor_msg.integrated_xgyro = optflow_ygyro * opticalFlow_message->integration_time_us() / 1000000.0; //xy switched
  sensor_msg.integrated_ygyro = optflow_xgyro * opticalFlow_message->integration_time_us() / 1000000.0; //xy switched
  sensor_msg.integrated_zgyro = -optflow_zgyro * opticalFlow_message->integration_time_us() / 1000000.0; //change direction
  sensor_msg.temperature = opticalFlow_message->temperature();
  sensor_msg.quality = opticalFlow_message->quality();
  sensor_msg.time_delta_distance_us = opticalFlow_message->time_delta_distance_us();
  sensor_msg.distance = optflow_distance;

  send_mavlink_message(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, &sensor_msg, 200);

}

void GazeboMavlinkInterface::pollForMAVLinkMessages()
{
  int len;
  ::poll(&fds[0], (sizeof(fds[0])/sizeof(fds[0])), 0);
  if (fds[0].revents & POLLIN) {
    len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, &_addrlen);
    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i)
      {
        if (mavlink_parse_char(MAVLINK_COMM_0, _buf[i], &msg, &status))
        {
          // have a message, handle it
          handle_message(&msg);
        }
      }
    }
  }
}

void GazeboMavlinkInterface::handle_message(mavlink_message_t *msg)
{
  switch(msg->msgid) {
  case MAVLINK_MSG_ID_HIL_CONTROLS:
    mavlink_hil_controls_t controls;
    mavlink_msg_hil_controls_decode(msg, &controls);
    bool armed = false;

    if ((controls.mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0) {
      armed = true;
    }

    const unsigned n_out = sizeof(inputs.control) / sizeof(inputs.control[0]);
    const unsigned block_size = 8;

    unsigned off = (controls.nav_mode * block_size);

    // We only support 8 outputs so far, so we
    // ignore the second, third and fourth output

    // XXX setting this to anything higher than 8 results
    // in memory smashing, likely due to a hardcoded
    // motor count somewhere
    if (off + block_size > 8) {
      break;
    }

    inputs.control[off + 0] = controls.roll_ailerons;
    inputs.control[off + 1] = controls.pitch_elevator;
    inputs.control[off + 2] = controls.yaw_rudder;
    inputs.control[off + 3] = controls.throttle;
    inputs.control[off + 4] = controls.aux1;
    inputs.control[off + 5] = controls.aux2;
    inputs.control[off + 6] = controls.aux3;
    inputs.control[off + 7] = controls.aux4;

    bool is_vtol = (right_elevon_joint_ != nullptr);

    // Set all scalings

    // Initialize all outputs as motors
    // if joints are present these will be
    // reconfigured in the next step
    for (unsigned i = off; i < off + block_size; i++) {
      input_index[i] = i;

      // scaling values
      input_offset[i] = 1.0;

      // XXX this needs re-investigation regarding
      // the correct scaling for the correct motor
      // model
      input_scaling[i] = 550.0;
      zero_position_disarmed[i] = 0.0;
      zero_position_armed[i] = (is_vtol) ? 0.0 : 100.0;
    }

    if (is_vtol) {
      // Config for standard VTOL model

      // Fift motor
      input_index[off + 4] = off + 4;
      input_offset[off + 4] = 1.0;
      input_scaling[off + 4] = 1600;
      zero_position_disarmed[off + 4] = 0.0;
      zero_position_armed[off + 4] = 0.0;

      // Servos
      for (unsigned i = off + 5; i < off + block_size; i++) {
        // scaling values
        input_index[i] = i;
        input_offset[i] = 0.0;
        input_scaling[i] = 1.0;
        zero_position_disarmed[i] = 0.0;
        zero_position_armed[i] = 0.0;
      }
    }

    last_actuator_time_ = world_->GetSimTime();

    input_reference_.resize(n_out);

    // set rotor speeds
    for (int i = 0; i < input_reference_.size(); i++) {
      if (armed) {
        input_reference_[i] = (inputs.control[input_index[i]] + input_offset[i]) * input_scaling[i] + zero_position_armed[i];
      } else {
        input_reference_[i] = zero_position_disarmed[i];
      }
    }

    // set joint positions
    for (int i = 0; i < input_reference_.size(); i++) {
      if (joints_[i]) {
#if GAZEBO_MAJOR_VERSION >= 6
        joints_[i]->SetPosition(0, input_reference_[i]);
#else
        joints_[i]->SetAngle(0, input_reference_[i]);
#endif
      }
    }

    // legacy method, can eventually be replaced
    if (right_elevon_joint_ != NULL && left_elevon_joint_!= 0 && elevator_joint_ != 0) {
#if GAZEBO_MAJOR_VERSION >= 6
      left_elevon_joint_->SetPosition(0, input_reference_[5]);
      right_elevon_joint_->SetPosition(0, input_reference_[6]);
      elevator_joint_->SetPosition(0, input_reference_[7]);
#else
      left_elevon_joint_->SetAngle(0, input_reference_[5]);
      right_elevon_joint_->SetAngle(0, input_reference_[6]);
      elevator_joint_->SetAngle(0, input_reference_[7]);
#endif
    }

    received_first_referenc_ = true;
    break;
  }
}

}
