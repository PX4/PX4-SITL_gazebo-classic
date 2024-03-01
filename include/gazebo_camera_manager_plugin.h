/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include <development/mavlink.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <SITLGps.pb.h>
#include <ignition/math.hh>
#include <chrono>

namespace gazebo
{

typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> GpsPtr;

/**
 * @class CameraManagerPlugin
 * Gazebo plugin that saves geotagged camera images to disk.
 */
class GAZEBO_VISIBLE CameraManagerPlugin : public SensorPlugin
{
public:
    CameraManagerPlugin();

    virtual ~CameraManagerPlugin();
    virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

    void OnNewFrame(const unsigned char *image);
    void OnNewGpsPosition(GpsPtr& gps_msg);
    void cameraThread();

private:
    void _handle_message(mavlink_message_t *msg, struct sockaddr* srcaddr);
    void _send_mavlink_message(const mavlink_message_t *message, struct sockaddr* srcaddr = NULL);
    void _handle_camera_info(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_request_camera_capture_status(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_storage_info(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_take_photo(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_stop_take_photo(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_start_video_capture(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_stop_video_capture(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_request_camera_settings(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_request_video_stream_information(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_request_video_stream_status(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_set_camera_mode(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _handle_camera_zoom(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _send_capture_status(struct sockaddr* srcaddr = NULL);
    void _send_cmd_ack(uint8_t target_sysid, uint8_t target_compid, uint16_t cmd, unsigned char result, struct sockaddr* srcaddr);
    void _send_heartbeat();
    bool _init_udp(sdf::ElementPtr sdf);

private:

    int         _imageCounter{0};
    uint8_t     _mode{CAMERA_MODE_VIDEO};
    uint32_t    _width{0};
    uint32_t    _height{0};
    uint32_t    _depth{0};
    uint32_t    _destWidth{0};     ///< output size
    uint32_t    _destHeight{0};
    float       _maxZoom{8.0};
    float       _zoom{1.0};
    int         _captureCount{0};
    double      _captureInterval{0.0};
    int         _fd{-1};
    int         _zoom_cmd{0};

    enum {
        CAPTURE_DISABLED,
        CAPTURE_SINGLE,
        CAPTURE_ELAPSED,
        CAPTURE_VIDEO
    };

    int         _captureMode{CAPTURE_DISABLED};

    common::Time                _lastImageTime{};
    common::Time                _last_time{};
    common::Time                _last_heartbeat{};
    sensors::CameraSensorPtr    _parentSensor{nullptr};
    rendering::CameraPtr        _camera{nullptr};
    rendering::ScenePtr         _scene{nullptr};
    event::ConnectionPtr        _newFrameConnection{nullptr};
    std::string                 _storageDir;
    ignition::math::Vector3d    _lastGpsPosition;
    ignition::math::Angle       _hfov{1.57};      ///< Horizontal fov
    transport::NodePtr          _node_handle{nullptr};;
    std::string                 _namespace;
    transport::SubscriberPtr    _gpsSub{nullptr};;
    std::string                 _format;
    struct sockaddr_in          _myaddr;    ///< The locally bound address
    struct sockaddr_in          _gcsaddr;   ///< GCS target
    struct pollfd               _fds[1];
    std::mutex                  _captureMutex;
    std::string                 _videoURI{"udp://127.0.0.1:5600"};
    int                         _systemID{1};
    int                         _componentID{MAV_COMP_ID_CAMERA};
    int                         _mavlinkCamPort{14530};

    std::chrono::time_point<std::chrono::high_resolution_clock> _start_video_capture_time;
};

} /* namespace gazebo */
