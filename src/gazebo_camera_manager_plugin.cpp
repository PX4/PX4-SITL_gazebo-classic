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

#include "gazebo_camera_manager_plugin.h"

#include <math.h>
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>

#include <development/mavlink.h>

using namespace std;
using namespace gazebo;
using namespace cv;

// #define DEBUG_MESSAGE_IO

GZ_REGISTER_SENSOR_PLUGIN(CameraManagerPlugin)

static void* start_thread(void* param) {
    CameraManagerPlugin* plugin = (CameraManagerPlugin*)param;
    plugin->cameraThread();
    return nullptr;
}

CameraManagerPlugin::CameraManagerPlugin()
    : SensorPlugin()
{
}

CameraManagerPlugin::~CameraManagerPlugin()
{
    _parentSensor.reset();
    _camera.reset();
}

void CameraManagerPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
    if (!sensor)
        gzerr << "Invalid sensor pointer.\n";

    _parentSensor =
#if GAZEBO_MAJOR_VERSION >= 7
        std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
#else
        boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
#endif

    if (!_parentSensor)
    {
        gzerr << "CameraManagerPlugin requires a CameraSensor.\n";
    }

#if GAZEBO_MAJOR_VERSION >= 7
    _camera = _parentSensor->Camera();
#else
    _camera = _parentSensor->GetCamera();
#endif

    if (!_parentSensor)
    {
        gzerr << "CameraManagerPlugin not attached to a camera sensor\n";
        return;
    }
    _scene = _camera->GetScene();
#if GAZEBO_MAJOR_VERSION >= 8
    _lastImageTime = _scene->SimTime();
#else
    _lastImageTime = _scene->GetSimTime();
#endif

#if GAZEBO_MAJOR_VERSION >= 7
    _width    = _camera->ImageWidth();
    _height   = _camera->ImageHeight();
    _depth    = _camera->ImageDepth();
    _format   = _camera->ImageFormat();
#else
    _width    = _camera->GetImageWidth();
    _height   = _camera->GetImageHeight();
    _depth    = _camera->GetImageDepth();
    _format   = _camera->GetImageFormat();
#endif

    if (sdf->HasElement("robotNamespace")) {
        _namespace = sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        gzwarn << "[gazebo_geotagging_images_camera_plugin] Please specify a robotNamespace.\n";
    }

    _destWidth = _width;
    if (sdf->HasElement("width")) {
        _destWidth = sdf->GetElement("width")->Get<int>();
    }
    _destHeight = _height;
    if (sdf->HasElement("height")) {
        _destHeight = sdf->GetElement("height")->Get<int>();
    }
    if (sdf->HasElement("maximum_zoom")) {
        _maxZoom = sdf->GetElement("maximum_zoom")->Get<float>();
    }

    if (sdf->HasElement("video_uri")) {
        _videoURI = sdf->GetElement("video_uri")->Get<std::string>();
    }
    if (sdf->HasElement("system_id")) {
        _systemID = sdf->GetElement("system_id")->Get<int>();
    }
    if (sdf->HasElement("cam_component_id")) {
        _componentID = sdf->GetElement("cam_component_id")->Get<int>();
    }
    if (sdf->HasElement("mavlink_cam_udp_port")) {
        _mavlinkCamPort = sdf->GetElement("mavlink_cam_udp_port")->Get<int>();
    }

    //check if exiftool exists
    if (system("exiftool -ver &>/dev/null") != 0) {
        gzerr << "exiftool not found. geotagging_images plugin will be disabled" << endl;
        gzerr << "On Ubuntu, use 'sudo apt-get install libimage-exiftool-perl' to install" << endl;
        return;
    }

    _node_handle = transport::NodePtr(new transport::Node());
    _node_handle->Init();

    _parentSensor->SetActive(true);

    // Get the root model name
    const string scopedName = _parentSensor->ParentName();
    vector<string> names_splitted;
    boost::split(names_splitted, scopedName, boost::is_any_of("::"));
    names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted),
                                [](const string& name)
                                { return name.size() == 0; }), end(names_splitted));
    std::string rootModelName = names_splitted.front(); // The first element is the name of the root model

    // the second to the last name is the model name
    const std::string parentSensorModelName = names_splitted.rbegin()[1];

    _newFrameConnection = _camera->ConnectNewImageFrame(
                              boost::bind(&CameraManagerPlugin::OnNewFrame, this, _1));

    _gpsSub = _node_handle->Subscribe("~/" + rootModelName + "/link/gps", &CameraManagerPlugin::OnNewGpsPosition, this);

    _storageDir = "frames";
    boost::filesystem::remove_all(_storageDir); //clear existing images
    boost::filesystem::create_directory(_storageDir);

    if (_init_udp(sdf)) {
        // Start UDP thread
        pthread_t threadId;
        pthread_create(&threadId, NULL, start_thread, this);
    }
}

void CameraManagerPlugin::OnNewGpsPosition(GpsPtr& gps_msg) {
    _lastGpsPosition.X() = gps_msg->latitude_deg();
    _lastGpsPosition.Y() = gps_msg->longitude_deg();
    _lastGpsPosition.Z() = gps_msg->altitude();
    //gzdbg << "got gps pos: "<<_lastGpsPosition.X() <<", "<<lastGpsPosition.Y() <<endl;
}

void CameraManagerPlugin::OnNewFrame(const unsigned char * image)
{

    _captureMutex.lock();
    // Are we capturing at all?
    if (_captureMode == CAPTURE_DISABLED) {
        _captureMutex.unlock();
        return;
    }
    _captureMutex.unlock();

    // Check for time lapse capture
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time currentTime = _scene->SimTime();
#else
    common::Time currentTime = _scene->GetSimTime();
#endif
    double elapsed = currentTime.Double() - _lastImageTime.Double();
    if (_captureMode == CAPTURE_ELAPSED && (elapsed < _captureInterval)) {
        return;
    }

    _lastImageTime = currentTime;

#if GAZEBO_MAJOR_VERSION >= 7
    image = _camera->ImageData(0);
#else
    image = _camera->GetImageData(0);
#endif

    Mat frame    = Mat(_height, _width, CV_8UC3);
    Mat frameBGR = Mat(_height, _width, CV_8UC3);
    frame.data   = (uchar*)image; //frame has not the right color format yet -> convert
    cvtColor(frame, frameBGR, COLOR_RGB2BGR);

    char file_name[256];
    snprintf(file_name, sizeof(file_name), "%s/DSC%05i.jpg", _storageDir.c_str(), _imageCounter);

    if (_destWidth != _width || _destHeight != _height) {
        Mat frameResized;
        cv::Size size(_destWidth, _destHeight);
        cv::resize(frameBGR, frameResized, size);
        imwrite(file_name, frameResized);
    } else {
        imwrite(file_name, frameBGR);
    }

    char gps_tag_command[1024];
    double lat = _lastGpsPosition.X();
    char north_south = 'N', east_west = 'E';
    double lon = _lastGpsPosition.Y();
    if (lat < 0.) {
        lat = -lat;
        north_south = 'S';
    }
    if (lon < 0.) {
        lon = -lon;
        east_west = 'W';
    }
    snprintf(gps_tag_command, sizeof(gps_tag_command),
             "exiftool -gpslatituderef=%c -gpslongituderef=%c -gpsaltituderef=above -gpslatitude=%.9lf -gpslongitude=%.9lf"
//           " -gpsdatetime=now -gpsmapdatum=WGS-84"
             " -datetimeoriginal=now -gpsdop=0.8"
             " -gpsmeasuremode=3-d -gpssatellites=13 -gpsaltitude=%.3lf -overwrite_original %s &>/dev/null",
             north_south, east_west, lat, lon, _lastGpsPosition.Z(), file_name);

    if (system(gps_tag_command) < 0) {
        gzerr << "gps tag command failed" << endl;
        return;
    }

    gzmsg << "Took picture: " << file_name << endl;

    // Send indication to GCS
    mavlink_message_t msg;
    mavlink_msg_camera_image_captured_pack_chan(
        _systemID,
        _componentID,
        MAVLINK_COMM_1,
        &msg,
        currentTime.Double() * 1e3, // time boot ms
        currentTime.Double() * 1e6, // time UTC
        1, // camera ID
        lat * 1e7,
        lon * 1e7,
        _lastGpsPosition.Z(),
        0, // relative alt
        0, // q[4]
        _imageCounter,
        1, // result
        0 // file_url
    );

    // Send to GCS port directly
    _send_mavlink_message(&msg);
    ++_imageCounter;
    _captureMutex.lock();
    if (_captureMode == CAPTURE_SINGLE) {
        _captureMode  = CAPTURE_DISABLED;
    } else {
        // _captureCount == 0 is infinite
        if (_captureCount && --_captureCount < 1) {
            _captureCount = 0;
            _captureMode  = CAPTURE_DISABLED;
        }
    }
    if (_captureMode == CAPTURE_DISABLED) {
        gzdbg << "Done with image capture\n";
    }
    _captureMutex.unlock();
    // Send Capture Status
    _send_capture_status();
}

void CameraManagerPlugin::_handle_message(mavlink_message_t *msg, struct sockaddr* srcaddr)
{
#if defined(DEBUG_MESSAGE_IO)
    sockaddr_in* foo = (sockaddr_in*)srcaddr;
    if (msg->sysid == 255) {
        gzdbg << "Message " << std::to_string(msg->msgid).c_str() <<
              " " << std::to_string(msg->sysid).c_str() << " " << std::to_string(msg->compid).c_str() <<
              " from: " << inet_ntoa(foo->sin_addr) << ":" << std::to_string(ntohs(foo->sin_port)).c_str() << endl;
    }
#endif
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG:
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg, &cmd);
        mavlink_command_long_t digicam_ctrl_cmd;
        if (cmd.target_component == _componentID) {
            switch (cmd.command) {
            case MAV_CMD_IMAGE_START_CAPTURE:
                _handle_take_photo(msg, srcaddr);
                break;
            case MAV_CMD_IMAGE_STOP_CAPTURE:
                _handle_stop_take_photo(msg, srcaddr);
                break;
            case MAV_CMD_VIDEO_START_CAPTURE:
                _handle_start_video_capture(msg, srcaddr);
                break;
            case MAV_CMD_VIDEO_STOP_CAPTURE:
                _handle_stop_video_capture(msg, srcaddr);
                break;
            case MAV_CMD_REQUEST_CAMERA_INFORMATION:
                _handle_camera_info(msg, srcaddr);
                break;
            case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
                _handle_request_camera_capture_status(msg, srcaddr);
                break;
            case MAV_CMD_REQUEST_STORAGE_INFORMATION:
                _handle_storage_info(msg, srcaddr);
                break;
            case MAV_CMD_REQUEST_CAMERA_SETTINGS:
                _handle_request_camera_settings(msg, srcaddr);
                break;
            case MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
                _handle_request_video_stream_information(msg, srcaddr);
                break;
            case MAV_CMD_REQUEST_VIDEO_STREAM_STATUS:
                _handle_request_video_stream_status(msg, srcaddr);
                break;
            case MAV_CMD_SET_CAMERA_MODE:
                _handle_set_camera_mode(msg, srcaddr);
                break;
            case MAV_CMD_SET_CAMERA_ZOOM:
                _handle_camera_zoom(msg, srcaddr);
                break;
            case MAV_CMD_RESET_CAMERA_SETTINGS:
                // Just ACK and ignore
                _send_cmd_ack(msg->sysid, msg->compid, MAV_CMD_RESET_CAMERA_SETTINGS, MAV_RESULT_ACCEPTED, srcaddr);
                break;
            case MAV_CMD_STORAGE_FORMAT:
                // Just ACK and ignore
                _send_cmd_ack(msg->sysid, msg->compid, MAV_CMD_STORAGE_FORMAT, MAV_RESULT_ACCEPTED, srcaddr);
                break;
            case MAV_CMD_VIDEO_START_STREAMING:
                // Just ACK and ignore
                _send_cmd_ack(msg->sysid, msg->compid, MAV_CMD_VIDEO_START_STREAMING, MAV_RESULT_ACCEPTED, srcaddr);
                break;
            case MAV_CMD_VIDEO_STOP_STREAMING:
                // Just ACK and ignore
                _send_cmd_ack(msg->sysid, msg->compid, MAV_CMD_VIDEO_STOP_STREAMING, MAV_RESULT_ACCEPTED, srcaddr);
                break;
            }
        }
        break;
    }
}

void CameraManagerPlugin::_send_mavlink_message(const mavlink_message_t *message, struct sockaddr* srcaddr)
{
    struct sockaddr* target = srcaddr ? srcaddr : (struct sockaddr *)&_gcsaddr;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);
    ssize_t len = sendto(_fd, buffer, packetlen, 0, target, sizeof(_gcsaddr));
    if (len <= 0) {
        gzerr << "Failed sending mavlink message" << endl;
#if defined(DEBUG_MESSAGE_IO)
    } else {
        sockaddr_in* foo = (sockaddr_in*)target;
        gzdbg << "Message " << std::to_string(message->msgid).c_str() <<
              " " << std::to_string(message->sysid).c_str() << " " << std::to_string(message->compid).c_str() <<
              " to: " << inet_ntoa(foo->sin_addr) << ":" << std::to_string(ntohs(foo->sin_port)).c_str() << endl;
#endif
    }
}

void CameraManagerPlugin::_send_cmd_ack(uint8_t target_sysid, uint8_t target_compid, uint16_t cmd, unsigned char result, struct sockaddr* srcaddr)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack_chan(
        _systemID,
        _componentID,
        MAVLINK_COMM_1,
        &msg,
        cmd,
        result,
        100,
        0,
        target_sysid,
        target_compid);
    _send_mavlink_message(&msg, srcaddr);
}

void CameraManagerPlugin::_send_heartbeat()
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack_chan(_systemID, _componentID, MAVLINK_COMM_1, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
    // Send to GCS port directly
    _send_mavlink_message(&msg);
    // Gazebo log output is using buffered IO for some reason
    fflush(stdout);
    fflush(stderr);
}

void CameraManagerPlugin::cameraThread() {
    mavlink_status_t  status;
    mavlink_message_t msg;
    unsigned char buffer[16 * 1024];
    while (true) {
        ::poll(&_fds[0], (sizeof(_fds[0]) / sizeof(_fds[0])), 1000);
        if (_fds[0].revents & POLLIN) {
            struct sockaddr srcaddr;
            socklen_t addrlen = sizeof(srcaddr);
            int len = recvfrom(_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&srcaddr, &addrlen);
            if (len > 0) {
                for (unsigned i = 0; i < len; ++i)
                {
                    if (mavlink_parse_char(MAVLINK_COMM_1, buffer[i], &msg, &status))
                    {
                        // have a message, handle it
                        _handle_message(&msg, &srcaddr);
                        memset(&status, 0, sizeof(status));
                        memset(&msg, 0, sizeof(msg));
                    }
                }
            }
        }
        // Heartbeat
#if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = _scene->SimTime();
#else
        common::Time current_time = _scene->GetSimTime();
#endif
        double elapsed = (current_time - _last_heartbeat).Double();
        if (elapsed > 1.0) {
            _last_heartbeat = current_time;
            _send_heartbeat();
        }

        //Move camera zoom incase of continuous zoom
        // _zoom_cmd is set by MAV_CMD_SET_CAMERA_ZOOM
        if (_zoom_cmd!=0) {
            _zoom = std::max(std::min(float(_zoom + 0.05 * _zoom_cmd), _maxZoom), 1.0f);
            _camera->SetHFOV(_hfov / _zoom);
        }

    }
}

bool CameraManagerPlugin::_init_udp(sdf::ElementPtr sdf) {
    //-- Target
    uint32_t mavlink_addr = htonl(INADDR_LOOPBACK);
    /* TODO: We need to find if the system is setup for broadcast
             and add the proper socket options and broadcast address
             if that's the case. It's hardcoded to loopback for now.

    if (sdf->HasElement("mavlink_telem_addr")) {
        std::string mavlink_addr = sdf->GetElement("mavlink_telem_addr")->Get<std::string>();
        if (mavlink_addr != "INADDR_ANY") {
            mavlink_addr_ = inet_addr(mavlink_addr.c_str());
            if (mavlink_addr_ == INADDR_NONE) {
                fprintf(stderr, "invalid mavlink_addr \"%s\"\n", mavlink_addr.c_str());
                return;
            }
        }
    }

    */
    //Create socket
    if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        gzerr << "Create camera plugin UDP socket failed" << endl;
        return false;
    }
    memset((char *)&_myaddr, 0, sizeof(_myaddr));
    _myaddr.sin_family = AF_INET;
    _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    // Choose the default cam port
    _myaddr.sin_port = htons(_mavlinkCamPort);
    if (::bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
        gzerr << "Bind failed for camera UDP plugin" << endl;
        return false;
    }
    _gcsaddr.sin_family = AF_INET;
    _gcsaddr.sin_addr.s_addr = mavlink_addr;
    _gcsaddr.sin_port = htons(14550);
    _fds[0].fd = _fd;
    _fds[0].events = POLLIN;
    mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_1);
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    gzmsg << "[Camera manager plugin]: Camera on udp port " + std::to_string(_mavlinkCamPort) + "\n";
    return true;
}

void CameraManagerPlugin::_handle_take_photo(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{

    gzdbg << "Handle Start Capture" << endl;
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    std::lock_guard<std::mutex> guard(_captureMutex);
    //-- We we busy?
    if (_captureMode != CAPTURE_DISABLED) {
        _send_cmd_ack(pMsg->sysid, pMsg->compid,
                      MAV_CMD_IMAGE_START_CAPTURE, MAV_RESULT_TEMPORARILY_REJECTED, srcaddr);
        return;
    }
    //-- Single capture?
    if (cmd.param3 == 1) {
        _captureMode = CAPTURE_SINGLE;
        _send_cmd_ack(pMsg->sysid, pMsg->compid,
                      MAV_CMD_IMAGE_START_CAPTURE, MAV_RESULT_ACCEPTED, srcaddr);
        //-- Time lapse?
    } else if (cmd.param3 >= 0 && cmd.param2 > 0.0) {
        gzdbg << "Start time lapse of " << (int)cmd.param3 << " shots every " << cmd.param2 << " seconds.\n";
        _captureInterval = cmd.param2;
        _captureCount    = cmd.param3;
        _captureMode     = CAPTURE_ELAPSED;
        _send_cmd_ack(pMsg->sysid, pMsg->compid,
                      MAV_CMD_IMAGE_START_CAPTURE, MAV_RESULT_ACCEPTED, srcaddr);
    } else {
        gzerr << "Bad Start Capture argments: " << cmd.param2 << " " << cmd.param3 << endl;
        _send_cmd_ack(pMsg->sysid, pMsg->compid,
                      MAV_CMD_IMAGE_START_CAPTURE, MAV_RESULT_DENIED, srcaddr);
    }
}

void CameraManagerPlugin::_handle_stop_take_photo(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{

    gzdbg << "Handle Stop Capture" << endl;
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    std::lock_guard<std::mutex> guard(_captureMutex);
    _captureMode = CAPTURE_DISABLED;
    _send_cmd_ack(pMsg->sysid, pMsg->compid,
                  MAV_CMD_IMAGE_STOP_CAPTURE, MAV_RESULT_ACCEPTED, srcaddr);
}

void CameraManagerPlugin::_handle_start_video_capture(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    gzdbg << "Handle Start Video Capture" << endl;

    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    std::lock_guard<std::mutex> guard(_captureMutex);

    if (_captureMode != CAPTURE_DISABLED) {
        // We are already capturing
        _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_VIDEO_START_CAPTURE, MAV_RESULT_TEMPORARILY_REJECTED, srcaddr);
        return;
    }

    if (cmd.param1 != 0 || cmd.param2 != 0) {
        gzerr << "VIDEO_START_CAPTURE: param1 and param2 must be 0\n";
        _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_VIDEO_START_CAPTURE, MAV_RESULT_DENIED, srcaddr);
        return;
    }

    _captureMode = CAPTURE_VIDEO;
    _start_video_capture_time = std::chrono::high_resolution_clock::now();
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_VIDEO_START_CAPTURE, MAV_RESULT_ACCEPTED, srcaddr);
}

void CameraManagerPlugin::_handle_stop_video_capture(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    gzdbg << "Handle Stop Video Capture" << endl;

    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    std::lock_guard<std::mutex> guard(_captureMutex);

    if (cmd.param1 != 0) {
        gzerr << "VIDEO_STOP_CAPTURE: param1 must be 0\n";
        _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_VIDEO_START_CAPTURE, MAV_RESULT_DENIED, srcaddr);
        return;
    }

    if (_captureMode != CAPTURE_VIDEO) {
        // We are not capturing
        _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_VIDEO_START_CAPTURE, MAV_RESULT_TEMPORARILY_REJECTED, srcaddr);
        return;
    }

    _captureMode = CAPTURE_DISABLED;
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_VIDEO_STOP_CAPTURE, MAV_RESULT_ACCEPTED, srcaddr);
}

void CameraManagerPlugin::_handle_request_camera_capture_status(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    // Should we execute the command
    if (cmd.param1 != 1)
    {
        gzwarn << "Camera capture status request ignored" << endl;
        return;
    }
    // ACK command received and accepted
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS, MAV_RESULT_ACCEPTED, srcaddr);
    //-- Specs call for recording time in ms. get_recording_time returns seconds.
    _send_capture_status(srcaddr);
}

void CameraManagerPlugin::_handle_camera_info(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    gzdbg << "Send camera info" << endl;
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_REQUEST_CAMERA_INFORMATION, MAV_RESULT_ACCEPTED, srcaddr);

    mavlink_camera_information_t camera_information{};
    std::strncpy((char *)camera_information.vendor_name, "PX4.io", sizeof(camera_information.vendor_name));
    std::strncpy((char *)camera_information.model_name, "Gazebo", sizeof(camera_information.model_name));
    camera_information.firmware_version = 0x01;
    camera_information.focal_length = 50.f;
    camera_information.sensor_size_h = 35.f;
    camera_information.sensor_size_v = 24.f;
    camera_information.resolution_h = _width;
    camera_information.resolution_v = _height;
    camera_information.flags = CAMERA_CAP_FLAGS_CAPTURE_IMAGE
                                | CAMERA_CAP_FLAGS_CAPTURE_VIDEO
                                | CAMERA_CAP_FLAGS_HAS_MODES
                                | CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM
                                | CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM;;

    mavlink_message_t msg;
    mavlink_msg_camera_information_encode_chan(_systemID, _componentID, MAVLINK_COMM_1, &msg, &camera_information);
    _send_mavlink_message(&msg, srcaddr);
}

void CameraManagerPlugin::_handle_request_camera_settings(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    gzdbg << "Send camera settings" << endl;
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    //-- Should we execute the command
    if ((int)cmd.param1 != 1)
    {
        gzwarn << "Request ignored" << endl;
        return;
    }
    // ACK command received and accepted
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_REQUEST_CAMERA_SETTINGS, MAV_RESULT_ACCEPTED, srcaddr);
    mavlink_message_t msg;
    mavlink_msg_camera_settings_pack_chan(
        _systemID,
        _componentID,
        MAVLINK_COMM_1,
        &msg,
        0,                      // time_boot_ms
        _mode,                  // Camera Mode
        1.0E2 * (_zoom - 1.0)/ (_maxZoom - 1.0),                    // Zoom level
        NAN,                    // Focus level
	0);                     // MAVLink camera with its own component ID
    _send_mavlink_message(&msg, srcaddr);
}

void CameraManagerPlugin::_handle_request_video_stream_status(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    gzdbg << "Send videostream status" << endl;
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    int sid = static_cast<int>(cmd.param1);
    //-- Should we execute the command
    if ((int)cmd.param1 != 1)
    {
        gzwarn << "Request ignored" << endl;
        return;
    }
    // ACK command received and accepted
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_REQUEST_VIDEO_STREAM_STATUS, MAV_RESULT_ACCEPTED, srcaddr);
    mavlink_message_t msg;
    mavlink_msg_video_stream_status_pack_chan(
        _systemID,
        _componentID,                                           // Component ID
        MAVLINK_COMM_1,
        &msg,
        static_cast<uint8_t>(sid),                              // Stream ID
        VIDEO_STREAM_STATUS_FLAGS_RUNNING,                      // Flags (It's always running)
        30,                                                     // Frame rate
        _width,                                                 // Horizontal resolution
        _height,                                                // Vertical resolution
        2048,                                                   // Bit rate (made up)
        0,                                                      // Rotation (none)
        90,                                                     // FOV (made up)
	0);                                                     // MAVLink camera with its own component ID

    _send_mavlink_message(&msg, srcaddr);
}

void CameraManagerPlugin::_handle_request_video_stream_information(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    gzdbg << "Send videostream information" << endl;
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    //-- Should we execute the command
    if ((int)cmd.param1 != 1)
    {
        gzwarn << "Request ignored" << endl;
        return;
    }

    // ACK command received and accepted
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION, MAV_RESULT_ACCEPTED, srcaddr);

    mavlink_video_stream_information_t video_stream_information{};
    video_stream_information.stream_id = 1;
    video_stream_information.count = 1;
    video_stream_information.type = VIDEO_STREAM_TYPE_RTPUDP;
    video_stream_information.flags = VIDEO_STREAM_STATUS_FLAGS_RUNNING; // It's always running
    video_stream_information.framerate = 30;
    video_stream_information.resolution_h = _width;
    video_stream_information.resolution_v = _height;
    video_stream_information.bitrate = 2048;
    video_stream_information.hfov = 90; // made up
    std::strncpy(video_stream_information.name, "Visual Spectrum", sizeof(video_stream_information.name));
    std::strncpy(video_stream_information.uri, _videoURI.c_str(), sizeof(video_stream_information.uri));

    mavlink_message_t msg;
    mavlink_msg_video_stream_information_encode_chan(_systemID, _componentID, MAVLINK_COMM_1, &msg, &video_stream_information);
    _send_mavlink_message(&msg, srcaddr);
}

void CameraManagerPlugin::_handle_set_camera_mode(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    _send_cmd_ack(pMsg->sysid, pMsg->compid,
                  MAV_CMD_SET_CAMERA_MODE, MAV_RESULT_ACCEPTED, srcaddr);
    if (_mode == CAMERA_MODE_VIDEO) {
        _mode = CAMERA_MODE_IMAGE;
    } else {
        _mode = CAMERA_MODE_VIDEO;
    }

}

void CameraManagerPlugin::_handle_camera_zoom(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    _send_cmd_ack(pMsg->sysid, pMsg->compid,
                  MAV_CMD_SET_CAMERA_ZOOM, MAV_RESULT_ACCEPTED, srcaddr);

    if (cmd.param1 == ZOOM_TYPE_CONTINUOUS) {
        _zoom = std::max(std::min(float(_zoom + 0.1 * cmd.param2), _maxZoom), 1.0f);
        _zoom_cmd = cmd.param2;
    } else if (cmd.param1 == ZOOM_TYPE_RANGE) {
	auto zoomRange = std::min(std::max(cmd.param2, 0.0f), 100.0f);
	_zoom = 1.0f + (zoomRange / 100.0f) * (_maxZoom - 1.0f);
        _camera->SetHFOV(_hfov / _zoom);
    } else {
        _zoom = std::max(std::min(float(_zoom + 0.1 * cmd.param2), _maxZoom), 1.0f);
        _camera->SetHFOV(_hfov / _zoom);
    }
}

void CameraManagerPlugin::_send_capture_status(struct sockaddr* srcaddr)
{
    _captureMutex.lock();
    uint8_t image_status = 0; // idle
    uint8_t video_status = 0; // idle
    uint32_t recording_time_ms = 0;
    if (_captureMode == CAPTURE_SINGLE) {
        image_status = 1; // active
    } else if (_captureMode == CAPTURE_ELAPSED) {
        image_status = 3; // time lapse
    } else if (_captureMode == CAPTURE_VIDEO) {
        video_status = 1; // active
        auto current_time = std::chrono::high_resolution_clock::now();
        recording_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - _start_video_capture_time).count();
    }
    float interval = CAPTURE_ELAPSED ? (float)_captureInterval : 0.0f;
    _captureMutex.unlock();

    gzdbg << "Send capture status" << endl;
    float available_mib = 0.0f;
    boost::filesystem::space_info si = boost::filesystem::space(".");
    available_mib = (float)((double)si.available / (1024.0 * 1024.0));

#if GAZEBO_MAJOR_VERSION >= 9
    common::Time current_time = _scene->SimTime();
#else
    common::Time current_time = _scene->GetSimTime();
#endif

    mavlink_message_t msg;
    mavlink_msg_camera_capture_status_pack_chan(
        _systemID,
        _componentID,
        MAVLINK_COMM_1,
        &msg,
        current_time.Double() * 1e3,
        image_status,                           // image status
        video_status,                           // video status
        interval,                               // image interval
        recording_time_ms,                      // recording time in ms
        available_mib,                          // available storage capacity
        _imageCounter,                          // total number of images
	0);                                     // MAVLink camera with its own component ID
    _send_mavlink_message(&msg, srcaddr);
}

void CameraManagerPlugin::_handle_storage_info(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    gzdbg << "Send storage info" << endl;
    float total_mib     = 0.0f;
    float available_mib = 0.0f;
    boost::filesystem::space_info si = boost::filesystem::space(".");
    const std::string storage_name = "SITL Camera Storage";
    available_mib = (float)((double)si.available / (1024.0 * 1024.0));
    total_mib     = (float)((double)si.capacity  / (1024.0 * 1024.0));
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_REQUEST_STORAGE_INFORMATION, MAV_RESULT_ACCEPTED, srcaddr);
    uint8_t storage_usage = STORAGE_USAGE_FLAG_SET | STORAGE_USAGE_FLAG_PHOTO | STORAGE_USAGE_FLAG_VIDEO | STORAGE_USAGE_FLAG_LOGS;
    mavlink_message_t msg;
    mavlink_msg_storage_information_pack_chan(
        _systemID,
        _componentID,
        MAVLINK_COMM_1,
        &msg,
        0,                                  // time_boot_ms
        1,                                  // storage_id,
        1,                                  // storage_count,
        2,                                  // status: (formatted)
        total_mib,
        total_mib - available_mib,          // used_capacity,
        available_mib,
        NAN,                                // read_speed,
        NAN,                                // write_speed
        STORAGE_TYPE_OTHER,                 // storage type
        storage_name.c_str(),               // storage name
        storage_usage                       // storage usage
    );
    _send_mavlink_message(&msg, srcaddr);
}
