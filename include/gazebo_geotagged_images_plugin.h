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
#include <mavlink/v2.0/common/mavlink.h>
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

namespace gazebo
{

typedef const boost::shared_ptr<const gps_msgs::msgs::SITLGps> GpsPtr;

/**
 * @class GeotaggedImagesPlugin
 * Gazebo plugin that saves geotagged camera images to disk.
 */
class GAZEBO_VISIBLE GeotaggedImagesPlugin : public SensorPlugin
{
public:
    GeotaggedImagesPlugin();

    virtual ~GeotaggedImagesPlugin();
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
    void _handle_request_camera_settings(const mavlink_message_t *pMsg, struct sockaddr* srcaddr);
    void _send_capture_status(struct sockaddr* srcaddr = NULL);
    void _send_cmd_ack(uint8_t target_sysid, uint8_t target_compid, uint16_t cmd, unsigned char result, struct sockaddr* srcaddr);
    void _send_heartbeat();
    bool _init_udp(sdf::ElementPtr sdf);
    void _take_picture();

private:
    float       storeIntervalSec_;
    int         imageCounter_;
    uint32_t    width_;
    uint32_t    height_;
    uint32_t    depth_;
    uint32_t    destWidth_;     ///< output size
    uint32_t    destHeight_;
    bool        capture_;
    int         _fd;

    common::Time lastImageTime_{};
    common::Time last_time_{};
    common::Time last_heartbeat_{};
    sensors::CameraSensorPtr parentSensor_;
    rendering::CameraPtr camera_;
    rendering::ScenePtr scene_;
    event::ConnectionPtr newFrameConnection_;
    std::string storageDir_;
    math::Vector3 lastGpsPosition_;
    transport::NodePtr node_handle_;
    std::string namespace_;
    transport::SubscriberPtr gpsSub_;
    std::string format_;
    struct sockaddr_in _myaddr;    ///< The locally bound address
    struct sockaddr_in _gcsaddr;   ///< GCS target
    struct pollfd fds[1];
};

} /* namespace gazebo */
