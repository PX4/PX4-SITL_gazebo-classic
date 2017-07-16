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

GZ_REGISTER_SENSOR_PLUGIN(GeotaggedImagesPlugin)


GeotaggedImagesPlugin::GeotaggedImagesPlugin()
: SensorPlugin(), width_(0), height_(0), depth_(0), imageCounter_(0)
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
  this->width_ = this->camera_->ImageWidth();
  this->height_ = this->camera_->ImageHeight();
  this->depth_ = this->camera_->ImageDepth();
  this->format_ = this->camera_->ImageFormat();
#else
  this->width_ = this->camera_->GetImageWidth();
  this->height_ = this->camera_->GetImageHeight();
  this->depth_ = this->camera_->GetImageDepth();
  this->format_ = this->camera_->GetImageFormat();
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

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GeotaggedImagesPlugin::OnUpdate, this, _1));

  //Create socket
  // udp socket data
  mavlink_addr_ = htonl(INADDR_ANY);
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
  if (sdf->HasElement("mavlink_telem_udp_port")) {
    mavlink_udp_port_ = sdf->GetElement("mavlink_telem_udp_port")->Get<int>();
  }

  // try to setup udp socket for communcation with simulator
  if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    printf("create socket failed\n");
    return;
  }

  memset((char *)&_myaddr, 0, sizeof(_myaddr));
  _myaddr.sin_family = AF_INET;
  _myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  // Choose the default cam port
  _myaddr.sin_port = htons(14530);

  if (::bind(_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
    printf("bind failed\n");
    return;
  }

  _srcaddr.sin_family = AF_INET;
  _srcaddr.sin_addr.s_addr = mavlink_addr_;
  _srcaddr.sin_port = htons(mavlink_udp_port_);
  _addrlen = sizeof(_srcaddr);

  fds[0].fd = _fd;
  fds[0].events = POLLIN;

  mavlink_status_t* chan_state = mavlink_get_channel_status(MAVLINK_COMM_1);
  chan_state->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
}

// This gets called by the world update start event.
void GeotaggedImagesPlugin::OnUpdate(const common::UpdateInfo& /*_info*/) {
#if GAZEBO_MAJOR_VERSION >= 8
  common::Time current_time = scene_->SimTime();
#else
  common::Time current_time = scene_->GetSimTime();
#endif
  double dt = (current_time - last_time_).Double();

  pollForMAVLinkMessages(dt, 1000);

  // TODO: This is the camera main loop

  last_time_ = current_time;
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
  mavlink_msg_camera_image_captured_pack_chan(1,
                                   1 /*MAV_COMP_ID_CAMERA*/,
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
  send_mavlink_message(&msg, 14550);

  ++imageCounter_;
  capture_ = false;
}

void GeotaggedImagesPlugin::TakePicture()
{
  capture_ = true;
}

void GeotaggedImagesPlugin::pollForMAVLinkMessages(double _dt, uint32_t _timeoutMs)
{
  // poll with immediate return
  ::poll(&fds[0], (sizeof(fds[0])/sizeof(fds[0])), 0);

  if (fds[0].revents & POLLIN) {
    int len;

    len = recvfrom(_fd, _buf, sizeof(_buf), 0, (struct sockaddr *)&_srcaddr, &_addrlen);
    if (len > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (unsigned i = 0; i < len; ++i)
      {
        if (mavlink_parse_char(MAVLINK_COMM_1, _buf[i], &msg, &status))
        {
          // have a message, handle it
          handle_message(&msg);
        }
      }
    }
  }
}

void GeotaggedImagesPlugin::handle_message(mavlink_message_t *msg)
{
  switch(msg->msgid) {
  case MAVLINK_MSG_ID_COMMAND_LONG:
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(msg, &cmd);
    mavlink_command_long_t digicam_ctrl_cmd;
    if (cmd.target_component == MAV_COMP_ID_CAMERA
        && cmd.command == MAV_CMD_DO_DIGICAM_CONTROL
        && cmd.param5 == 1) {
      // Take one picture
      TakePicture();
      mavlink_message_t msg;
      mavlink_msg_command_ack_pack_chan(1,
                                       MAV_COMP_ID_CAMERA,
                                       MAVLINK_COMM_1,
                                       &msg,
                                       MAV_CMD_DO_DIGICAM_CONTROL,
                                       MAV_RESULT_ACCEPTED,
                                       100);
      send_mavlink_message(&msg);
    }
    break;
  }
}

void GeotaggedImagesPlugin::send_mavlink_message(const mavlink_message_t *message, const int destination_port)
{

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);

  if (destination_port != 0) {
    _srcaddr.sin_port = htons(destination_port);
  }

  ssize_t len = sendto(_fd, buffer, packetlen, 0, (struct sockaddr *)&_srcaddr, sizeof(_srcaddr));

  if (len <= 0) {
    printf("Failed sending mavlink message\n");
  }
}
