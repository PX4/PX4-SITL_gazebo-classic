/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2020 PX4 Pro Development Team
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

#pragma once

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
#include <cstdint>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <Eigen/Eigen>
#include<Eigen/StdVector>

#include <development/mavlink.h>
#include "msgbuffer.h"

static const uint32_t kDefaultMavlinkUdpPort = 14560;
static const uint32_t kDefaultMavlinkTcpPort = 4560;
static const uint32_t kDefaultQGCUdpPort = 14550;
static const uint32_t kDefaultSDKUdpPort = 14540;

using lock_guard = std::lock_guard<std::recursive_mutex>;
static constexpr auto kDefaultDevice = "/dev/ttyACM0";
static constexpr auto kDefaultBaudRate = 921600;

//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t MAX_TXQ_SIZE = 1000;

//! Rx packer framing status. (same as @p mavlink::mavlink_framing_t)
enum class Framing : uint8_t {
	incomplete = MAVLINK_FRAMING_INCOMPLETE,
	ok = MAVLINK_FRAMING_OK,
	bad_crc = MAVLINK_FRAMING_BAD_CRC,
	bad_signature = MAVLINK_FRAMING_BAD_SIGNATURE,
};

//! Enumeration to use on the bitmask in HIL_SENSOR
enum class SensorSource {
  ACCEL		= 0b111,
  GYRO		= 0b111000,
  MAG		= 0b111000000,
  BARO		= 0b1101000000000,
  DIFF_PRESS	= 0b10000000000,
};

namespace SensorData {
    struct Imu {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d accel_b;
        Eigen::Vector3d gyro_b;
    };

    struct Barometer {
        double temperature;
        double abs_pressure;
        double pressure_alt;
    };

    struct Magnetometer {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d mag_b;
    };

    struct Airspeed {
        double diff_pressure;
    };

    struct Gps {
        uint64_t time_utc_usec;
        int fix_type;
        double latitude_deg;
        double longitude_deg;
        double altitude;
        double eph;
        double epv;
        double velocity;
        double velocity_north;
        double velocity_east;
        double velocity_down;
        double cog;
        double satellites_visible;
        int id;
    };
}

struct HILData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int id=-1;
    bool baro_updated{false};
    bool diff_press_updated{false};
    bool mag_updated{false};
    bool imu_updated{false};
    double temperature;
    double pressure_alt;
    double abs_pressure;
    double diff_pressure;
    Eigen::Vector3d mag_b;
    Eigen::Vector3d accel_b;
    Eigen::Vector3d gyro_b;
};

class MavlinkInterface {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MavlinkInterface();
    ~MavlinkInterface();
    void pollForMAVLinkMessages();
    void pollFromQgcAndSdk();
    void send_mavlink_message(const mavlink_message_t *message);
    void forward_mavlink_message(const mavlink_message_t *message);
    void open();
    void close();
    void Load();
    void SendHeartbeat();
    void SendSensorMessages(const uint64_t time_usec);
    void SendSensorMessages(const uint64_t time_usec, HILData &hil_data);
    void SendGpsMessages(const SensorData::Gps &data);
    void UpdateBarometer(const SensorData::Barometer &data, const int id = 0);
    void UpdateAirspeed(const SensorData::Airspeed &data, const int id = 0);
    void UpdateIMU(const SensorData::Imu &data, const int id = 0);
    void UpdateMag(const SensorData::Magnetometer &data, const int id = 0);
    Eigen::VectorXd GetActuatorControls();
    bool GetArmedState();
    void onSigInt();
    bool GetReceivedFirstActuator() {return received_first_actuator_;}
    void SetBaudrate(int baudrate) {baudrate_ = baudrate;}
    void SetSerialEnabled(bool serial_enabled) {serial_enabled_ = serial_enabled;}
    void SetUseTcp(bool use_tcp) {use_tcp_ = use_tcp;}
    void SetDevice(std::string device) {device_ = device;}
    void SetEnableLockstep(bool enable_lockstep) {enable_lockstep_ = enable_lockstep;}
    void SetMavlinkAddr(std::string mavlink_addr) {mavlink_addr_str_ = mavlink_addr;}
    void SetMavlinkTcpPort(int mavlink_tcp_port) {mavlink_tcp_port_ = mavlink_tcp_port;}
    void SetMavlinkUdpPort(int mavlink_udp_port) {mavlink_udp_port_ = mavlink_udp_port;}
    void SetQgcAddr(std::string qgc_addr) {qgc_addr_ = qgc_addr;}
    void SetQgcUdpPort(int qgc_udp_port) {qgc_udp_port_ = qgc_udp_port;}
    void SetSdkAddr(std::string sdk_addr) {sdk_addr_ = sdk_addr;}
    void SetSdkUdpPort(int sdk_udp_port) {sdk_udp_port_ = sdk_udp_port;}
    void SetHILMode(bool hil_mode) {hil_mode_ = hil_mode;}
    void SetHILStateLevel(bool hil_state_level) {hil_state_level_ = hil_state_level;}

    bool SerialEnabled() const { return serial_enabled_; }
    bool ReceivedHeartbeats() const { return received_heartbeats_; }

private:
    bool received_actuator_{false};
    bool received_first_actuator_{false};
    bool armed_;
    Eigen::VectorXd input_reference_;

    void handle_message(mavlink_message_t *msg);
    void handle_heartbeat(mavlink_message_t *msg);
    void handle_actuator_controls(mavlink_message_t *msg);
    void acceptConnections();
    void RegisterNewHILSensorInstance(int id);

    // Serial interface
    void open_serial();
    void do_serial_read();
    void parse_serial_buffer(const boost::system::error_code& err, std::size_t bytes_t);
    inline bool is_serial_open(){
        return serial_dev_.is_open();
    }
    void do_serial_write(bool check_tx_state);

    static const unsigned n_out_max = 16;

    int input_index_[n_out_max];

    struct sockaddr_in local_simulator_addr_;
    socklen_t local_simulator_addr_len_;
    struct sockaddr_in remote_simulator_addr_;
    socklen_t remote_simulator_addr_len_;

    int qgc_udp_port_{kDefaultQGCUdpPort};
    struct sockaddr_in remote_qgc_addr_;
    socklen_t remote_qgc_addr_len_;
    struct sockaddr_in local_qgc_addr_;
    std::string qgc_addr_{"INADDR_ANY"};
    socklen_t local_qgc_addr_len_;

    int sdk_udp_port_{kDefaultSDKUdpPort};
    struct sockaddr_in remote_sdk_addr_;
    socklen_t remote_sdk_addr_len_;
    struct sockaddr_in local_sdk_addr_;
    socklen_t local_sdk_addr_len_;
    std::string sdk_addr_{"INADDR_ANY"};

    unsigned char buf_[65535];
    enum FD_TYPES {
        LISTEN_FD,
        CONNECTION_FD,
        N_FDS
    };
    struct pollfd fds_[N_FDS];
    bool use_tcp_{false};
    bool close_conn_{false};

    in_addr_t mavlink_addr_;
    std::string mavlink_addr_str_{"INADDR_ANY"};
    int mavlink_udp_port_{kDefaultMavlinkUdpPort}; // MAVLink refers to the PX4 simulator interface here
    int mavlink_tcp_port_{kDefaultMavlinkTcpPort}; // MAVLink refers to the PX4 simulator interface here


    int simulator_socket_fd_{0};
    int simulator_tcp_client_fd_{0};

    int qgc_socket_fd_{0};
    int sdk_socket_fd_{0};

    bool enable_lockstep_{false};

    // Serial interface
    boost::asio::io_service io_service_{};
    boost::asio::serial_port serial_dev_;
    bool serial_enabled_{false};

    mavlink_status_t m_status_{};
    mavlink_message_t m_buffer_{};
    std::thread io_thread_;
    std::string device_{kDefaultDevice};

    std::recursive_mutex mutex_;
    std::mutex actuator_mutex_;
    std::mutex sensor_msg_mutex_;

    std::array<uint8_t, MAX_SIZE> rx_buf_{};
    unsigned int baudrate_{kDefaultBaudRate};
    std::atomic<bool> tx_in_progress_;
    std::deque<gazebo::MsgBuffer> tx_q_{};

    bool hil_mode_;
    bool hil_state_level_;

    std::vector<HILData, Eigen::aligned_allocator<HILData>> hil_data_;
    std::atomic<bool> gotSigInt_ {false};

    bool received_heartbeats_ {false};
};
