#include "mavlink_interface.h"

MavlinkInterface::MavlinkInterface() :
    serial_dev_(io_service_){

}

MavlinkInterface::~MavlinkInterface() {
  close();
}

void MavlinkInterface::Load()
{
  mavlink_addr_ = htonl(INADDR_ANY);
  if (mavlink_addr_str_ != "INADDR_ANY") {
    mavlink_addr_ = inet_addr(mavlink_addr_str_.c_str());
    if (mavlink_addr_ == INADDR_NONE) {
      std::cerr << "Invalid mavlink_addr: " << mavlink_addr_str_ << ", aborting\n";
      abort();
    }
  }
  local_qgc_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
  if (qgc_addr_ != "INADDR_ANY") {
    local_qgc_addr_.sin_addr.s_addr = inet_addr(qgc_addr_.c_str());
    if (local_qgc_addr_.sin_addr.s_addr == INADDR_NONE) {
      std::cerr << "Invalid qgc_addr: " << qgc_addr_ << ", aborting\n";
      abort();
    }
  }
  local_sdk_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
  if (sdk_addr_ != "INADDR_ANY") {
    local_sdk_addr_.sin_addr.s_addr = inet_addr(sdk_addr_.c_str());
    if (local_sdk_addr_.sin_addr.s_addr == INADDR_NONE) {
      std::cerr << "Invalid sdk_addr: " << sdk_addr_ << ", aborting\n";
      abort();
    }
  }

  if (hil_mode_) {

    local_qgc_addr_.sin_family = AF_INET;
    local_qgc_addr_.sin_port = htons(0);
    local_qgc_addr_len_ = sizeof(local_qgc_addr_);

    remote_qgc_addr_.sin_family = AF_INET;
    remote_qgc_addr_.sin_port = htons(qgc_udp_port_);
    remote_qgc_addr_len_ = sizeof(remote_qgc_addr_);

    local_sdk_addr_.sin_family = AF_INET;
    local_sdk_addr_.sin_port = htons(0);
    local_sdk_addr_len_ = sizeof(local_sdk_addr_);

    remote_sdk_addr_.sin_family = AF_INET;
    remote_sdk_addr_.sin_port = htons(sdk_udp_port_);
    remote_sdk_addr_len_ = sizeof(remote_sdk_addr_);

    if ((qgc_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      std::cerr << "Creating QGC UDP socket failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if (bind(qgc_socket_fd_, (struct sockaddr *)&local_qgc_addr_, local_qgc_addr_len_) < 0) {
      std::cerr << "QGC UDP bind failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if ((sdk_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      std::cerr << "Creating SDK UDP socket failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

    if (bind(sdk_socket_fd_, (struct sockaddr *)&local_sdk_addr_, local_sdk_addr_len_) < 0) {
      std::cerr << "SDK UDP bind failed: " << strerror(errno) << ", aborting\n";
      abort();
    }

  }

  if (serial_enabled_) {
    // Set up serial interface
	  io_service_.post(std::bind(&MavlinkInterface::do_serial_read, this));

    // run io_service for async io
    io_thread_ = std::thread([this] () {
    	io_service_.run();
    });
    open_serial();

  } else {
    memset((char *)&remote_simulator_addr_, 0, sizeof(remote_simulator_addr_));
    remote_simulator_addr_.sin_family = AF_INET;
    remote_simulator_addr_len_ = sizeof(remote_simulator_addr_);

    memset((char *)&local_simulator_addr_, 0, sizeof(local_simulator_addr_));
    local_simulator_addr_.sin_family = AF_INET;
    local_simulator_addr_len_ = sizeof(local_simulator_addr_);

    if (use_tcp_) {

      local_simulator_addr_.sin_addr.s_addr = htonl(mavlink_addr_);
      local_simulator_addr_.sin_port = htons(mavlink_tcp_port_);

      if ((simulator_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Creating TCP socket failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      int yes = 1;
      int result = setsockopt(simulator_socket_fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      struct linger nolinger {};
      nolinger.l_onoff = 1;
      nolinger.l_linger = 0;

      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_LINGER, &nolinger, sizeof(nolinger));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // The socket reuse is necessary for reconnecting to the same address
      // if the socket does not close but gets stuck in TIME_WAIT. This can happen
      // if the server is suddenly closed, for example, if the robot is deleted in gazebo.
      int socket_reuse = 1;
      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &socket_reuse, sizeof(socket_reuse));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // Same as above but for a given port
      result = setsockopt(simulator_socket_fd_, SOL_SOCKET, SO_REUSEPORT, &socket_reuse, sizeof(socket_reuse));
      if (result != 0) {
        std::cerr << "setsockopt failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // set socket to non-blocking
      result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
      if (result == -1) {
        std::cerr << "setting socket to non-blocking failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
        std::cerr << "bind failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      errno = 0;
      if (listen(simulator_socket_fd_, 0) < 0) {
        std::cerr << "listen failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      memset(fds_, 0, sizeof(fds_));
      fds_[LISTEN_FD].fd = simulator_socket_fd_;
      fds_[LISTEN_FD].events = POLLIN; // only listens for new connections on tcp

    } else {
      if (!hil_mode_) {
        // When connecting to SITL, we specify the port where the mavlink traffic originates from.
        remote_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
        remote_simulator_addr_.sin_port = htons(mavlink_udp_port_);
        local_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        local_simulator_addr_.sin_port = htons(0);
      } else {
        // When connecting to HITL via UDP, the vehicle talks to a specific port that we need to
        // listen to.
        remote_simulator_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        remote_simulator_addr_.sin_port = htons(0);
        local_simulator_addr_.sin_addr.s_addr = mavlink_addr_;
        local_simulator_addr_.sin_port = htons(mavlink_udp_port_);
      }

      if ((simulator_socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Creating UDP socket failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      // set socket to non-blocking
      int result = fcntl(simulator_socket_fd_, F_SETFL, O_NONBLOCK);
      if (result == -1) {
        std::cerr << "setting socket to non-blocking failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      if (bind(simulator_socket_fd_, (struct sockaddr *)&local_simulator_addr_, local_simulator_addr_len_) < 0) {
        std::cerr << "bind failed: " << strerror(errno) << ", aborting\n";
        abort();
      }

      memset(fds_, 0, sizeof(fds_));
      fds_[CONNECTION_FD].fd = simulator_socket_fd_;
      fds_[CONNECTION_FD].events = POLLIN;
    }
  }
  // hil_data_.resize(1);
}

void MavlinkInterface::SendSensorMessages(uint64_t time_usec) {
  for (auto& data : hil_data_) {
    if (data.baro_updated | data.diff_press_updated | data.mag_updated | data.imu_updated) {
      SendSensorMessages(time_usec, data);
    }
  }
}

void MavlinkInterface::SendHeartbeat() {
  // In order to start the mavlink instance on Pixhawk over USB, we need to send heartbeats.
  if (hil_mode_) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack_chan(
      1, 200,
      MAVLINK_COMM_0,
      &msg,
      MAV_TYPE_GENERIC,
      MAV_AUTOPILOT_INVALID,
      0, 0, 0);
    send_mavlink_message(&msg);
  }
}

void MavlinkInterface::SendSensorMessages(uint64_t time_usec, HILData &hil_data) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);

  HILData* data = &hil_data;
  mavlink_hil_sensor_t sensor_msg;
  sensor_msg.id = data->id;
  sensor_msg.time_usec = time_usec;
  if (data->imu_updated) {
    sensor_msg.xacc = data->accel_b[0];
    sensor_msg.yacc = data->accel_b[1];
    sensor_msg.zacc = data->accel_b[2];
    sensor_msg.xgyro = data->gyro_b[0];
    sensor_msg.ygyro = data->gyro_b[1];
    sensor_msg.zgyro = data->gyro_b[2];
    // std::cout <<data->gyro_b[2] << std::endl;

    sensor_msg.fields_updated = (uint16_t)SensorSource::ACCEL | (uint16_t)SensorSource::GYRO;

    data->imu_updated = false;
  }

  // send only mag data
  if (data->mag_updated) {
    sensor_msg.xmag = data->mag_b[0];
    sensor_msg.ymag = data->mag_b[1];
    sensor_msg.zmag = data->mag_b[2];
    sensor_msg.fields_updated = sensor_msg.fields_updated | (uint16_t)SensorSource::MAG;

    data->mag_updated = false;
  }

  // send only baro data
  if (data->baro_updated) {
    sensor_msg.temperature = data->temperature;
    sensor_msg.abs_pressure = data->abs_pressure;
    sensor_msg.pressure_alt = data->pressure_alt;
    sensor_msg.fields_updated = sensor_msg.fields_updated | (uint16_t)SensorSource::BARO;

    data->baro_updated = false;
  }

  // send only diff pressure data
  if (data->diff_press_updated) {
    sensor_msg.diff_pressure = data->diff_pressure;
    sensor_msg.fields_updated = sensor_msg.fields_updated | (uint16_t)SensorSource::DIFF_PRESS;

    data->diff_press_updated = false;
  }

  if (!hil_mode_ || (hil_mode_ && !hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
    send_mavlink_message(&msg);
  }
}

void MavlinkInterface::SendGpsMessages(const SensorData::Gps &data) {
  // fill HIL GPS Mavlink msg
  mavlink_hil_gps_t hil_gps_msg;
  hil_gps_msg.time_usec = data.time_utc_usec;
  hil_gps_msg.fix_type = data.fix_type;
  hil_gps_msg.lat = data.latitude_deg;
  hil_gps_msg.lon = data.longitude_deg;
  hil_gps_msg.alt = data.altitude;
  hil_gps_msg.eph = data.eph;
  hil_gps_msg.epv = data.epv;
  hil_gps_msg.vel = data.velocity;
  hil_gps_msg.vn = data.velocity_north;
  hil_gps_msg.ve = data.velocity_east;
  hil_gps_msg.vd = data.velocity_down;
  hil_gps_msg.cog = data.cog;
  hil_gps_msg.satellites_visible = data.satellites_visible;
  hil_gps_msg.id = data.id;

  // send HIL_GPS Mavlink msg
  if (!hil_mode_ || (hil_mode_ && !hil_state_level_)) {
    mavlink_message_t msg;
    mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &hil_gps_msg);
    send_mavlink_message(&msg);
  }
}

void MavlinkInterface::UpdateBarometer(const SensorData::Barometer &data, int id) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);
  for (auto& instance : hil_data_) {
    if (instance.id == id) {
      instance.temperature = data.temperature;
      instance.abs_pressure = data.abs_pressure;
      instance.pressure_alt = data.pressure_alt;
      instance.baro_updated = true;
      return;
    }
  }
  //Register new HIL instance if we have never seen the id
  RegisterNewHILSensorInstance(id);
}

void MavlinkInterface::UpdateAirspeed(const SensorData::Airspeed &data, int id) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);
  for (auto& instance : hil_data_) {
    if (instance.id == id) {
      instance.diff_pressure = data.diff_pressure;
      instance.diff_press_updated = true;
      return;
    }
  }
  //Register new HIL instance if we have never seen the id
  RegisterNewHILSensorInstance(id);
}

void MavlinkInterface::UpdateIMU(const SensorData::Imu &data, int id) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);
  for (auto& instance : hil_data_) {
    if (instance.id == id) {
      instance.accel_b = data.accel_b;
      instance.gyro_b = data.gyro_b;
      instance.imu_updated = true;
      return;
    }
  }
  //Register new HIL instance if we have never seen the id
  RegisterNewHILSensorInstance(id);
}

void MavlinkInterface::UpdateMag(const SensorData::Magnetometer &data, int id) {
  const std::lock_guard<std::mutex> lock(sensor_msg_mutex_);
  for (auto& instance : hil_data_) {
    if (instance.id == id) {
      instance.mag_b = data.mag_b;
      instance.mag_updated = true;
      return;
    }
  }
  //Register new HIL instance if we have never seen the id
  RegisterNewHILSensorInstance(id);
}

void MavlinkInterface::RegisterNewHILSensorInstance(int id) {
  HILData new_instance;
  new_instance.id = id;
  hil_data_.push_back(new_instance);
}

void MavlinkInterface::pollForMAVLinkMessages()
{
  if (gotSigInt_) {
    return;
  }

  received_actuator_ = false;

  do {
    const bool needs_to_wait_for_actuator = received_first_actuator_ && enable_lockstep_;
    int timeout_ms = needs_to_wait_for_actuator ? 1000 : 0;
    int ret = ::poll(&fds_[0], N_FDS, timeout_ms);

    if (ret < 0) {
      std::cerr << "poll error: " << strerror(errno) << "\n";
      return;
    }

    if (ret == 0) {
      if (needs_to_wait_for_actuator) {
        std::cerr << "poll timeout\n";
      }
      return;
    }

    for (int i = 0; i < N_FDS; i++) {
      if(fds_[i].revents == 0) {
        continue;
      }

      if (!(fds_[i].revents & POLLIN)) {
        continue;
      }

      if (i == LISTEN_FD) { // if event is raised on the listening socket
        acceptConnections();
      } else { // if event is raised on connection socket
        int ret = recvfrom(fds_[i].fd, buf_, sizeof(buf_), 0, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);
        if (ret < 0) {
          // all data is read if EWOULDBLOCK is raised
          if (errno != EWOULDBLOCK) { // disconnected from client
            std::cerr << "recvfrom error: " << strerror(errno) << "\n";
          }
          continue;
        }

        // client closed the connection orderly, only makes sense on tcp
        if (use_tcp_ && ret == 0) {
          std::cerr << "Connection closed by client." << "\n";
          close_conn_ = true;
          continue;
        }

        // data received
        int len = ret;
        mavlink_message_t msg;
        mavlink_status_t status;
        for (unsigned i = 0; i < len; ++i) {
          if (mavlink_parse_char(MAVLINK_COMM_0, buf_[i], &msg, &status)) {
            if (hil_mode_ && serial_enabled_) {
              send_mavlink_message(&msg);
            }
            handle_message(&msg);
          }
        }
      }
    }
  } while (!close_conn_ && !gotSigInt_ && !received_actuator_);
}

void MavlinkInterface::pollFromQgcAndSdk()
{
  struct pollfd fds[2] = {};
  fds[0].fd = qgc_socket_fd_;
  fds[0].events = POLLIN;
  fds[1].fd = sdk_socket_fd_;
  fds[1].events = POLLIN;

  const int timeout_ms = 0;

  int ret = ::poll(&fds[0], 2, timeout_ms);

  if (ret < 0) {
    std::cerr << "poll error: " << strerror(errno) << "\n";
    return;
  }

  if (fds[0].revents & POLLIN) {
    int len = recvfrom(qgc_socket_fd_, buf_, sizeof(buf_), 0, (struct sockaddr *)&remote_qgc_addr_, &remote_qgc_addr_len_);

    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_1, buf_[i], &msg, &status)) {
          // forward message from QGC to serial
          send_mavlink_message(&msg);
        }
      }
    }
  }

  if (fds[1].revents & POLLIN) {
    int len = recvfrom(sdk_socket_fd_, buf_, sizeof(buf_), 0, (struct sockaddr *)&remote_sdk_addr_, &remote_sdk_addr_len_);

    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_2, buf_[i], &msg, &status)) {
          // forward message from SDK to serial
          send_mavlink_message(&msg);
        }
      }
    }
  }
}

void MavlinkInterface::acceptConnections()
{
  if (fds_[CONNECTION_FD].fd > 0) {
    return;
  }

  // accepting incoming connections on listen fd
  int ret =
    accept(fds_[LISTEN_FD].fd, (struct sockaddr *)&remote_simulator_addr_, &remote_simulator_addr_len_);

  if (ret < 0) {
    if (errno != EWOULDBLOCK) {
      std::cerr << "accept error: " << strerror(errno) << "\n";
    }
    return;
  }

  // assign socket to connection descriptor on success
  fds_[CONNECTION_FD].fd = ret; // socket is replaced with latest connection
  fds_[CONNECTION_FD].events = POLLIN;
}

void MavlinkInterface::handle_message(mavlink_message_t *msg)
{
  switch (msg->msgid) {
  case MAVLINK_MSG_ID_HEARTBEAT:
    handle_heartbeat(msg);
    break;
  case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
    handle_actuator_controls(msg);
    break;
  }
}

void MavlinkInterface::handle_heartbeat(mavlink_message_t *)
{
  received_heartbeats_ = true;
}

void MavlinkInterface::handle_actuator_controls(mavlink_message_t *msg)
{
  const std::lock_guard<std::mutex> lock(actuator_mutex_);

  mavlink_hil_actuator_controls_t controls;
  mavlink_msg_hil_actuator_controls_decode(msg, &controls);

  armed_ = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);

  for (unsigned i = 0; i < n_out_max; i++) {
    input_index_[i] = i;
  }

  // set rotor speeds, controller targets
  input_reference_.resize(n_out_max);
  for (int i = 0; i < input_reference_.size(); i++) {
    input_reference_[i] = controls.controls[i];
  }

  received_actuator_ = true;
  received_first_actuator_ = true;
}

void MavlinkInterface::forward_mavlink_message(const mavlink_message_t *message)
{
  if (gotSigInt_) {
    return;
  }

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);
  ssize_t len;
  if (qgc_socket_fd_ > 0) {
    len = sendto(qgc_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_qgc_addr_, remote_qgc_addr_len_);

    if (len <= 0)
    {
      std::cerr << "Failed sending mavlink message to QGC: " << strerror(errno) << "\n";
    }
  }

  if (sdk_socket_fd_ > 0) {
    len = sendto(sdk_socket_fd_, buffer, packetlen, 0, (struct sockaddr *)&remote_sdk_addr_, remote_sdk_addr_len_);
    if (len <= 0)
    {
      std::cerr << "Failed sending mavlink message to SDK: " << strerror(errno) << "\n";
    }
  }
}

void MavlinkInterface::send_mavlink_message(const mavlink_message_t *message)
{
  assert(message != nullptr);

  if (gotSigInt_ || close_conn_) {
    return;
  }

  if (serial_enabled_) {

    if (!is_serial_open()) {
      std::cerr << "Serial port closed! \n";
      return;
    }

    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);

      if (tx_q_.size() >= MAX_TXQ_SIZE) {
        std::cout << "Tx queue overflow\n";
      }
      tx_q_.emplace_back(message);
    }
    io_service_.post(std::bind(&MavlinkInterface::do_serial_write, this, true));

  } else {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);

    if (fds_[CONNECTION_FD].fd > 0) {
      ssize_t len;
      if (use_tcp_) {
        len = send(fds_[CONNECTION_FD].fd, buffer, packetlen, 0);
      } else {
        len = sendto(fds_[CONNECTION_FD].fd, buffer, packetlen, 0, (struct sockaddr *)&remote_simulator_addr_, remote_simulator_addr_len_);
      }
      if (len < 0) {
        if (received_first_actuator_) {
          std::cerr << "Failed sending mavlink message: " << strerror(errno) << "\n";
          if (errno == ECONNRESET || errno == EPIPE) {
            if (use_tcp_) { // udp socket remains alive
              std::cerr << "Closing connection." << "\n";
              close_conn_ = true;
            }
          }
        }
      }
    }
  }
}

void MavlinkInterface::open_serial() {
  try{
    serial_dev_.open(device_);
    serial_dev_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
    serial_dev_.set_option(boost::asio::serial_port_base::character_size(8));
    serial_dev_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_dev_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_dev_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    std::cout << "Opened serial device " << device_ << "\n";
  }
  catch (boost::system::system_error &err) {
    std::cerr <<"Error opening serial device: " << err.what() << "\n";
  }
}

void MavlinkInterface::close()
{
  if(serial_enabled_) {
    ::close(qgc_socket_fd_);
    ::close(sdk_socket_fd_);

    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!is_serial_open())
      return;

    io_service_.stop();
    serial_dev_.close();

    if (io_thread_.joinable())
      io_thread_.join();

  } else {

    ::close(fds_[CONNECTION_FD].fd);
    fds_[CONNECTION_FD] = { 0, 0, 0 };
    fds_[CONNECTION_FD].fd = -1;

    received_first_actuator_ = false;

  }
}

void MavlinkInterface::do_serial_write(bool check_tx_state){
  if (check_tx_state && tx_in_progress_)
    return;

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (tx_q_.empty())
    return;

  tx_in_progress_ = true;
  auto &buf_ref = tx_q_.front();

  serial_dev_.async_write_some(
    boost::asio::buffer(buf_ref.dpos(), buf_ref.nbytes()), [this, &buf_ref] (boost::system::error_code error,   size_t bytes_transferred)
    {
      assert(bytes_transferred <= buf_ref.len);
      if(error) {
        std::cerr << "Serial error: " << error.message() << "\n";
      return;
      }

    std::lock_guard<std::recursive_mutex> lock(mutex_);

    if (tx_q_.empty()) {
      tx_in_progress_ = false;
      return;
    }

    buf_ref.pos += bytes_transferred;
    if (buf_ref.nbytes() == 0) {
      tx_q_.pop_front();
    }

    if (!tx_q_.empty()) {
      do_serial_write(false);
    }
    else {
      tx_in_progress_ = false;
    }
  });
}

void MavlinkInterface::do_serial_read(void)
{
  serial_dev_.async_read_some(boost::asio::buffer(rx_buf_), boost::bind(
      &MavlinkInterface::parse_serial_buffer, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred
      )
  );
}

// Based on MAVConnInterface::parse_buffer in MAVROS
void MavlinkInterface::parse_serial_buffer(const boost::system::error_code& err, std::size_t bytes_t){
  mavlink_status_t status;
  mavlink_message_t message;
  uint8_t *buf = this->rx_buf_.data();

  assert(rx_buf_.size() >= bytes_t);

  for(; bytes_t > 0; bytes_t--)
  {
    auto c = *buf++;

    auto msg_received = static_cast<Framing>(mavlink_frame_char_buffer(&m_buffer_, &m_status_, c, &message, &status));
    if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
      _mav_parse_error(&m_status_);
      m_status_.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      m_status_.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (c == MAVLINK_STX) {
        m_status_.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        m_buffer_.len = 0;
        mavlink_start_checksum(&m_buffer_);
      }
    }

    if (msg_received != Framing::incomplete) {
      if (hil_mode_) {
        forward_mavlink_message(&message);
      }
      handle_message(&message);
    }
  }
  do_serial_read();
}

void MavlinkInterface::onSigInt() {
  gotSigInt_ = true;
  close();
}

Eigen::VectorXd MavlinkInterface::GetActuatorControls() {
  const std::lock_guard<std::mutex> lock(actuator_mutex_);
  return input_reference_;
}

bool MavlinkInterface::GetArmedState() {
  const std::lock_guard<std::mutex> lock(actuator_mutex_);
  return armed_;
}
