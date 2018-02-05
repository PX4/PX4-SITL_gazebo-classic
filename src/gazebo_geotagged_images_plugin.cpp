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

#include "gazebo_geotagged_images_plugin.h"

#include <math.h>
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>

#include <cv.h>
#include <highgui.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace gazebo;
using namespace cv;

//#define DEBUG_MESSAGE_IO

GZ_REGISTER_SENSOR_PLUGIN(GeotaggedImagesPlugin)

static void* start_thread(void* param) {
    GeotaggedImagesPlugin* plugin = (GeotaggedImagesPlugin*)param;
    plugin->cameraThread();
    return nullptr;
}

GeotaggedImagesPlugin::GeotaggedImagesPlugin()
    : SensorPlugin()
    , width_(0)
    , height_(0)
    , depth_(0)
    , imageCounter_(0)
{
}

GeotaggedImagesPlugin::~GeotaggedImagesPlugin()
{
    this->parentSensor_.reset();
    this->camera_.reset();
}

void GeotaggedImagesPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
    if (!sensor)
        gzerr << "Invalid sensor pointer.\n";

    this->parentSensor_ =
#if GAZEBO_MAJOR_VERSION >= 7
        std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
#else
        boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
#endif

    if (!this->parentSensor_)
    {
        gzerr << "GeotaggedImagesPlugin requires a CameraSensor.\n";
    }

#if GAZEBO_MAJOR_VERSION >= 7
    this->camera_ = this->parentSensor_->Camera();
#else
    this->camera_ = this->parentSensor_->GetCamera();
#endif

    if (!this->parentSensor_)
    {
        gzerr << "GeotaggedImagesPlugin not attached to a camera sensor\n";
        return;
    }
    scene_ = camera_->GetScene();
#if GAZEBO_MAJOR_VERSION >= 8
    lastImageTime_ = scene_->SimTime();
#else
    lastImageTime_ = scene_->GetSimTime();
#endif

#if GAZEBO_MAJOR_VERSION >= 7
    this->width_    = this->camera_->ImageWidth();
    this->height_   = this->camera_->ImageHeight();
    this->depth_    = this->camera_->ImageDepth();
    this->format_   = this->camera_->ImageFormat();
#else
    this->width_    = this->camera_->GetImageWidth();
    this->height_   = this->camera_->GetImageHeight();
    this->depth_    = this->camera_->GetImageDepth();
    this->format_   = this->camera_->GetImageFormat();
#endif

    if (sdf->HasElement("robotNamespace")) {
        namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        gzwarn << "[gazebo_geotagging_images_camera_plugin] Please specify a robotNamespace.\n";
    }

    this->storeIntervalSec_ = 1;
    if (sdf->HasElement("interval")) {
        this->storeIntervalSec_ = sdf->GetElement("interval")->Get<float>();
    }

    destWidth_ = width_;
    if (sdf->HasElement("width")) {
        destWidth_ = sdf->GetElement("width")->Get<int>();
    }
    destHeight_ = height_;
    if (sdf->HasElement("height")) {
        destHeight_ = sdf->GetElement("height")->Get<int>();
    }

    //check if exiftool exists
    if (system("exiftool -ver &>/dev/null") != 0) {
        gzerr << "exiftool not found. geotagging_images plugin will be disabled" << endl;
        gzerr << "On Ubuntu, use 'sudo apt-get install libimage-exiftool-perl' to install" << endl;
        return;
    }

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    this->parentSensor_->SetActive(true);

    this->newFrameConnection_ = this->camera_->ConnectNewImageFrame(
                                    boost::bind(&GeotaggedImagesPlugin::OnNewFrame, this, _1));

    gpsSub_ = node_handle_->Subscribe("~/gps_position", &GeotaggedImagesPlugin::OnNewGpsPosition, this);

    storageDir_ = "frames";
    boost::filesystem::remove_all(storageDir_); //clear existing images
    boost::filesystem::create_directory(storageDir_);

    if (_init_udp(sdf)) {
        // Start UDP thread
        pthread_t threadId;
        pthread_create(&threadId, NULL, start_thread, this);
    }
}

void GeotaggedImagesPlugin::OnNewGpsPosition(ConstVector3dPtr& v)
{
    lastGpsPosition_ = *v;
    //gzdbg << "got gps pos: "<<lastGpsPosition_.x()<<", "<<lastGpsPosition.y()<<endl;
}

void GeotaggedImagesPlugin::OnNewFrame(const unsigned char * image)
{
#if GAZEBO_MAJOR_VERSION >= 7
    image = this->camera_->ImageData(0);
#else
    image = this->camera_->GetImageData(0);
#endif

    if (!capture_ || storeIntervalSec_ == 0) {
        return;
    }

#if GAZEBO_MAJOR_VERSION >= 8
    common::Time currentTime = scene_->SimTime();
#else
    common::Time currentTime = scene_->GetSimTime();
#endif

    if (currentTime.Double() - lastImageTime_.Double() < storeIntervalSec_) {
        gzerr << "WARNING: TRIGGERED CAMERA TOO FAST!" << endl;
        return;
    }

    lastImageTime_ = currentTime;

    Mat frame = Mat(height_, width_, CV_8UC3);
    Mat frameBGR = Mat(height_, width_, CV_8UC3);
    frame.data = (uchar*)image; //frame has not the right color format yet -> convert
    cvtColor(frame, frameBGR, CV_RGB2BGR);

    char file_name[256];
    snprintf(file_name, sizeof(file_name), "%s/DSC%05i.jpg", storageDir_.c_str(), imageCounter_);

    if (destWidth_ != width_ || destHeight_ != height_) {
        Mat frameResized;
        cv::Size size(destWidth_, destHeight_);
        cv::resize(frameBGR, frameResized, size);
        imwrite(file_name, frameResized);
    } else {
        imwrite(file_name, frameBGR);
    }

    char gps_tag_command[1024];
    double lat = lastGpsPosition_.x();
    char north_south = 'N', east_west = 'E';
    double lon = lastGpsPosition_.y();
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
//    " -gpsdatetime=now -gpsmapdatum=WGS-84"
             " -datetimeoriginal=now -gpsdop=0.8"
             " -gpsmeasuremode=3-d -gpssatellites=13 -gpsaltitude=%.3lf -overwrite_original %s &>/dev/null",
             north_south, east_west, lat, lon, lastGpsPosition_.z(), file_name);

    system(gps_tag_command);

    gzerr << "Took picture:" << file_name << endl;

    // Send indication to GCS
    mavlink_message_t msg;
    mavlink_msg_camera_image_captured_pack_chan(
        1,
        MAV_COMP_ID_CAMERA,
        MAVLINK_COMM_1,
        &msg,
        currentTime.Double() * 1e3, // time boot ms
        currentTime.Double() * 1e6, // time UTC
        1, // camera ID
        lat * 1e7,
        lon * 1e7,
        lastGpsPosition_.z(),
        0, // relative alt
        0, // q[4]
        imageCounter_,
        1, // result
        0 // file_url
    );

    // Send to GCS port directly
    send_mavlink_message(&msg);

    ++imageCounter_;
    capture_ = false;
}

void GeotaggedImagesPlugin::TakePicture()
{
    capture_ = true;
}

void GeotaggedImagesPlugin::handle_message(mavlink_message_t *msg, struct sockaddr* srcaddr)
{
#if defined(DEBUG_MESSAGE_IO)
    sockaddr_in* foo = (sockaddr_in*)srcaddr;
    if (msg->sysid == 255) {
        printf("Message %03u %u %u from: %s:%u\n", msg->msgid, msg->sysid, msg->compid, inet_ntoa(foo->sin_addr), ntohs(foo->sin_port));
        fflush(stdout);
    }
#endif
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG:
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg, &cmd);
        mavlink_command_long_t digicam_ctrl_cmd;
        if (cmd.target_component == MAV_COMP_ID_CAMERA) {
            switch (cmd.command) {
            case MAV_CMD_IMAGE_START_CAPTURE:
                // Take one picture
                printf("Handle Start Capture\n");
                if (cmd.param3 == 1) {
                    _send_cmd_ack(msg->sysid, msg->compid,
                                  MAV_CMD_REQUEST_CAMERA_INFORMATION, MAV_RESULT_ACCEPTED, srcaddr);
                    TakePicture();
                } else {
                    _send_cmd_ack(msg->sysid, msg->compid,
                                  MAV_CMD_REQUEST_CAMERA_INFORMATION, MAV_RESULT_UNSUPPORTED, srcaddr);
                }
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
            }
        }
        break;
    }
}

void GeotaggedImagesPlugin::send_mavlink_message(const mavlink_message_t *message, struct sockaddr* srcaddr)
{
    struct sockaddr* target = srcaddr ? srcaddr : (struct sockaddr *)&_gcsaddr;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int packetlen = mavlink_msg_to_send_buffer(buffer, message);
    ssize_t len = sendto(_fd, buffer, packetlen, 0, target, sizeof(_gcsaddr));
    if (len <= 0) {
        printf("Failed sending mavlink message\n");
#if defined(DEBUG_MESSAGE_IO)
    } else {
        sockaddr_in* foo = (sockaddr_in*)target;
        printf("Message %03u %u %u to   %s:%u\n", message->msgid, message->sysid, message->compid, inet_ntoa(foo->sin_addr), ntohs(foo->sin_port));
        fflush(stdout);
#endif
    }
}

void GeotaggedImagesPlugin::_send_cmd_ack(uint8_t target_sysid, uint8_t target_compid, uint16_t cmd, unsigned char result, struct sockaddr* srcaddr)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack_chan(
        1,
        MAV_COMP_ID_CAMERA,
        MAVLINK_COMM_1,
        &msg,
        cmd,
        result,
        100,
        0,
        target_sysid,
        target_compid);
    send_mavlink_message(&msg, srcaddr);
}

void GeotaggedImagesPlugin::_send_heartbeat()
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack_chan(1, MAV_COMP_ID_CAMERA, MAVLINK_COMM_1, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
    // Send to GCS port directly
    send_mavlink_message(&msg);
}

void GeotaggedImagesPlugin::cameraThread() {
    mavlink_status_t  status;
    mavlink_message_t msg;
    while (true) {
        ::poll(&fds[0], (sizeof(fds[0]) / sizeof(fds[0])), 1000);
        if (fds[0].revents & POLLIN) {
            struct sockaddr srcaddr;
            socklen_t addrlen = sizeof(srcaddr);
            int len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
            if (len > 0) {
                for (unsigned i = 0; i < len; ++i)
                {
                    if (mavlink_parse_char(MAVLINK_COMM_1, _buf[i], &msg, &status))
                    {
                        // have a message, handle it
                        handle_message(&msg, &srcaddr);
                        memset(&status, 0, sizeof(status));
                        memset(&msg, 0, sizeof(msg));
                    }
                }
            }
        }
        // Heartbeat
#if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = scene_->SimTime();
#else
        common::Time current_time = scene_->GetSimTime();
#endif
        double elapsed = (current_time - last_heartbeat_).Double();
        if (elapsed > 1.0) {
            last_heartbeat_ = current_time;
            _send_heartbeat();
        }
    }
}

bool GeotaggedImagesPlugin::_init_udp(sdf::ElementPtr sdf) {
    //-- Target
    uint32_t mavlink_addr = htonl(INADDR_LOOPBACK);
    /* Need to find if it's either localhost or broadcast
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
        printf("create socket failed\n");
        return false;
    }
    memset((char *)&_myaddr, 0, sizeof(_myaddr));
    _myaddr.sin_family = AF_INET;
    _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    // Choose the default cam port
    _myaddr.sin_port = htons(14530);
    if (::bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
        printf("bind failed\n");
        return false;
    }
    _gcsaddr.sin_family = AF_INET;
    _gcsaddr.sin_addr.s_addr = mavlink_addr;
    _gcsaddr.sin_port = htons(14550);
    _addrlen = sizeof(_gcsaddr);
    fds[0].fd = _fd;
    fds[0].events = POLLIN;
    mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_1);
    chan_state->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    return true;
}

void GeotaggedImagesPlugin::_handle_request_camera_capture_status(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(pMsg, &cmd);
    // Should we execute the command
    if (cmd.param1 != 1)
    {
        printf("Camera capture status request ignored\n");
        return;
    }
    // ACK command received and accepted
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS, MAV_RESULT_ACCEPTED, srcaddr);
    //-- Specs call for recording time in ms. get_recording_time returns seconds.
    _send_capture_status(srcaddr);
}

void GeotaggedImagesPlugin::_handle_camera_info(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    printf("Send camera info\n");
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_REQUEST_CAMERA_INFORMATION, MAV_RESULT_ACCEPTED, srcaddr);
    static const char* vendor = "PX4.io";
    static const char* model  = "Gazebo";
    char uri[128] = {};
    uint32_t camera_capabilities = CAMERA_CAP_FLAGS_CAPTURE_IMAGE;
    mavlink_message_t msg;
    mavlink_msg_camera_information_pack_chan(
        1,
        MAV_COMP_ID_CAMERA,
        MAVLINK_COMM_1,
        &msg,
        0,                         // time_boot_ms
        (const uint8_t *)vendor,   // const uint8_t * vendor_name
        (const uint8_t *)model,    // const uint8_t * model_name
        0x01,                      // uint32_t firmware_version
        50.0f,                     // float focal_lenth
        35.0f,                     // float  sensor_size_h
        24.0f,                     // float  sensor_size_v
        width_,                    // resolution_h
        height_,                   // resolution_v
        0,                         // lens_id
        camera_capabilities,       // CAP_FLAGS
        0,                         // Camera Definition Version
        uri                        // URI
    );
    send_mavlink_message(&msg, srcaddr);
}

void GeotaggedImagesPlugin::_send_capture_status(struct sockaddr* srcaddr)
{
    printf("Send capture status\n");
    float available_mib = 0.0f;
    boost::filesystem::space_info si = boost::filesystem::space(".");
    available_mib = (float)((double)si.available / (1024.0 * 1024.0));
    mavlink_message_t msg;
    mavlink_msg_camera_capture_status_pack_chan(
        1,
        MAV_COMP_ID_CAMERA,
        MAVLINK_COMM_1,
        &msg,
        0,
        0,                                     // image status (Idle)
        0,                                     // video status (Idle)
        0,                                     // image interval
        0,                                     // recording_time_s
        available_mib);                        // available_capacity
    send_mavlink_message(&msg, srcaddr);
}

void GeotaggedImagesPlugin::_handle_storage_info(const mavlink_message_t *pMsg, struct sockaddr* srcaddr)
{
    printf("Send storage info\n");
    float total_mib     = 0.0f;
    float available_mib = 0.0f;
    boost::filesystem::space_info si = boost::filesystem::space(".");
    available_mib = (float)((double)si.available / (1024.0 * 1024.0));
    total_mib     = (float)((double)si.capacity  / (1024.0 * 1024.0));
    _send_cmd_ack(pMsg->sysid, pMsg->compid, MAV_CMD_REQUEST_STORAGE_INFORMATION, MAV_RESULT_ACCEPTED, srcaddr);
    mavlink_message_t msg;
    mavlink_msg_storage_information_pack_chan(
        1,
        MAV_COMP_ID_CAMERA,
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
        NAN                                 // write_speed
    );
    send_mavlink_message(&msg, srcaddr);
}
