/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo_opticalFlow_plugin.h"

#include <highgui.h>
#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>

extern "C" {
  #include "settings.h"
  #include "flow.h"
}

using namespace cv;
using namespace std;

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(OpticalFlowPlugin)

/////////////////////////////////////////////////
OpticalFlowPlugin::OpticalFlowPlugin()
: SensorPlugin(), width(0), height(0), depth(0), timer_()
{
  // load optical flow parameters
  global_data_reset_param_defaults();
}

/////////////////////////////////////////////////
OpticalFlowPlugin::~OpticalFlowPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();
}

/////////////////////////////////////////////////
void OpticalFlowPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  if (!_sensor)

    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor =
#if GAZEBO_MAJOR_VERSION >= 7
    std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
#else
    boost::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
#endif

  if (!this->parentSensor)
  {
    gzerr << "OpticalFlowPlugin requires a CameraSensor.\n";
#if GAZEBO_MAJOR_VERSION >= 7
    if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
#else
    if (boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
#endif
      gzmsg << "It is a depth camera sensor\n";
  }

#if GAZEBO_MAJOR_VERSION >= 7
  this->camera = this->parentSensor->Camera();
#else
  this->camera = this->parentSensor->GetCamera();
#endif

  if (!this->parentSensor)
  {
    gzerr << "OpticalFlowPlugin not attached to a camera sensor\n";
    return;
  }

#if GAZEBO_MAJOR_VERSION >= 7
  this->width = this->camera->ImageWidth();
  this->height = this->camera->ImageHeight();
  this->depth = this->camera->ImageDepth();
  this->format = this->camera->ImageFormat();
#else
  this->width = this->camera->GetImageWidth();
  this->height = this->camera->GetImageHeight();
  this->depth = this->camera->GetImageDepth();
  this->format = this->camera->GetImageFormat();
#endif


  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_optical_flow_plugin] Please specify a robotNamespace.\n";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

#if GAZEBO_MAJOR_VERSION >= 7
  const string scopedName = _sensor->ParentName();
#else
  const string scopedName = _sensor->GetParentName();
#endif
  string topicName = "~/" + scopedName + "/opticalFlow";
  boost::replace_all(topicName, "::", "/");

  opticalFlow_pub_ = node_handle_->Advertise<opticalFlow_msgs::msgs::opticalFlow>(topicName, 10);



  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      boost::bind(&OpticalFlowPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void OpticalFlowPlugin::OnNewFrame(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int _depth,
                              const std::string &_format)
{
#if GAZEBO_MAJOR_VERSION >= 7
  _image = this->camera->ImageData(0);
#else
  _image = this->camera->GetImageData(0);
#endif

#if GAZEBO_MAJOR_VERSION >= 7
  const float hfov = float(this->camera->HFOV().Radian());
#else
  const float hfov = float(this->camera->GetHFOV().Radian());
#endif
  const float focal_length = (_width/2)/tan(hfov/2);

  timer_.stop();
  float rate = 100; // assume 100 hz
  float dt = 1.0/rate;
  Mat frame = Mat(_height, _width, CV_8UC3);
  Mat frameHSV = Mat(_height, _width, CV_8UC3);
  Mat frameBGR = Mat(_height, _width, CV_8UC3);
  frame.data = (uchar*)_image; //frame is not the right color now -> convert
  // dynamically scale image using HSV full conversion
  cvtColor(frame, frameHSV, CV_RGB2HSV_FULL); 
  cvtColor(frameHSV, frameBGR, CV_HSV2BGR); 
  cvtColor(frameBGR, frame_gray, CV_BGR2GRAY);
  float x_gyro_rate = 0;
  float y_gyro_rate = 0;
  float z_gyro_rate = 0;
  float x_flow = 0;
  float y_flow = 0;

  // check for correct image size
  if (frame_gray.rows != 64 || frame_gray.cols != 64) {
    printf("incorrect sizes, expected 64x64, got (%dx%d)\n",
        frame_gray.rows, frame_gray.cols);
    return;
  }

  // if no old image yet, create it and return
  if (old_gray.rows != 64 || old_gray.cols != 64) {
    old_gray = frame_gray.clone();
    return;
  }

  uint8_t quality = compute_flow((uint8_t *)old_gray.data, (uint8_t * )frame_gray.data,
      x_gyro_rate, y_gyro_rate, z_gyro_rate, &x_flow, &y_flow);
  float flow_x_ang = atan(x_flow/focal_length);
  float flow_y_ang = atan(y_flow/focal_length);
  old_gray = frame_gray.clone();
  //printf("x_flow: %10.4f y_flow: %10.4f rate: %10.2f f: %10.2f pixel flow x: %10.2f rad\t\tflow y: %10.2f rad\n",
          //x_flow, y_flow, rate, focal_length, flow_x_ang, flow_y_ang);
  opticalFlow_message.set_time_usec(100000000000000);//big number to prevent timeout in inav
  opticalFlow_message.set_sensor_id(2.0);
  opticalFlow_message.set_integration_time_us(dt * 1000000);
  opticalFlow_message.set_integrated_x(flow_x_ang);
  opticalFlow_message.set_integrated_y(flow_y_ang);
  opticalFlow_message.set_integrated_xgyro(0.0); //get real values in gazebo_mavlink_interface.cpp
  opticalFlow_message.set_integrated_ygyro(0.0); //get real values in gazebo_mavlink_interface.cpp
  opticalFlow_message.set_integrated_zgyro(0.0); //get real values in gazebo_mavlink_interface.cpp
  opticalFlow_message.set_temperature(20.0);
  opticalFlow_message.set_quality(quality);
  opticalFlow_message.set_time_delta_distance_us(0.0);
  opticalFlow_message.set_distance(0.0); //get real values in gazebo_mavlink_interface.cpp

  opticalFlow_pub_->Publish(opticalFlow_message);
  timer_.start();
}

/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
