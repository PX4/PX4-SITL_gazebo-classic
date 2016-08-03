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

#include <cv.h>
#include <highgui.h>
#include <math.h>
#include <string>
#include <iostream>


using namespace cv;
using namespace std;

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(OpticalFlowPlugin)

/////////////////////////////////////////////////
OpticalFlowPlugin::OpticalFlowPlugin()
: SensorPlugin(), width(0), height(0), depth(0), timer_()
{
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
  // TODO(tfoote) Find a way to namespace this within the model to allow multiple models
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

#if GAZEBO_MAJOR_VERSION >= 7
  float rate = this->camera->RenderRate();
#else
  float rate = this->camera->GetRenderRate();
#endif
  //printf("render rate raw: %10.2f\n", rate);
  if (!isfinite(rate))
       rate =  30.0;
  float dt = 1.0 / rate;


  Mat frame = Mat(_height, _width, CV_8UC3);
  Mat frameBGR = Mat(_height, _width, CV_8UC3);
  frame.data = (uchar*)_image; //frame is not the right color now -> convert
  cvtColor(frame, frameBGR, CV_RGB2BGR); 
  cvtColor(frameBGR, frame_gray, CV_BGR2GRAY);
  
  featuresPrevious = featuresCurrent;

  goodFeaturesToTrack(frame_gray, featuresCurrent, maxfeatures, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k); //calculate the features

  if (!old_gray.empty() && featuresPrevious.size() > 0){
    calcOpticalFlowPyrLK(old_gray, frame_gray, featuresPrevious, featuresNextPos, featuresFound, err);
  }

  /*/// Set the needed parameters to find the refined corners
  Size winSize = Size( 5, 5 );
  Size zeroZone = Size( -1, -1 );
  TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );

  // Calculate the refined corner locations
  cornerSubPix(frame_gray, featuresNextPos, winSize, zeroZone, criteria);
  cornerSubPix(old_gray, featuresPrevious, winSize, zeroZone, criteria);*/

  //calc diff
  int n_features = 0;
  for (int i = 0; i < featuresNextPos.size(); i++) {
    if (featuresFound[i] == true) {
        flowX_[i] =  featuresNextPos[i].x - featuresPrevious[i].x;
        flowY_[i] =  featuresNextPos[i].y - featuresPrevious[i].y;
        n_features += 1;
    }       
  }

  float median_flow_x = computeMedian(flowX_, n_features);
  float median_flow_y = computeMedian(flowY_, n_features);

  float flow_x_ang = atan(median_flow_x/focal_length);
  float flow_y_ang = atan(median_flow_y/focal_length);
  uint8_t quality = 255*(float(n_features)/ maxfeatures);

  old_gray = frame_gray.clone();

  //printf("rate: %10.2f f: %10.2f pixel flow x: %10.2f rad\t\tflow y: %10.2f rad\n",
          //rate, focal_length, flow_x_ang, flow_y_ang);

  opticalFlow_message.set_time_usec(100000000000000);//big number to prevent timeout in inav
  opticalFlow_message.set_sensor_id(2.0);
  opticalFlow_message.set_integration_time_us(dt * 1000000);
  opticalFlow_message.set_integrated_x(flow_x_ang);
  opticalFlow_message.set_integrated_y(flow_y_ang);
  opticalFlow_message.set_integrated_xgyro(0.0); //get real values in gazebo_mavlink_interface.cpp
  opticalFlow_message.set_integrated_ygyro(0.0); //get real values in gazebo_mavlink_interface.cpp
  opticalFlow_message.set_integrated_zgyro(0.0); //get real values in gazebo_mavlink_interface.cpp
  opticalFlow_message.set_temperature(20.0);
  opticalFlow_message.set_quality(quality); //features?
  opticalFlow_message.set_time_delta_distance_us(0.0);
  opticalFlow_message.set_distance(0.0); //get real values in gazebo_mavlink_interface.cpp

  opticalFlow_pub_->Publish(opticalFlow_message);


  // TODO handle case when more than one vehicles are using the plugin, then
  // update rate will be n_vehicle*rate, need to store in vehicle structure somehow.
  timer_.stop();
  float rate_real = 1.0e9/timer_.elapsed().wall;
  if (rate_real < 0.75*rate) {
    printf("!!! gazebo opticaal flow plugin running slow: %10.0f Hz\n", rate_real);
  }
  timer_.start();
}

float OpticalFlowPlugin::computeMedian(float * array, int iSize) {
    for (int i = iSize - 1; i > 0; --i) {
        for (int j = 0; j < i; ++j) {
            if (array[j] > array[j+1]) {
                float dTemp = array[j];
                array[j] = array[j+1];
                array[j+1] = dTemp;
            }
        }
    }

    // Middle or average of middle values in the sorted array.
    float dMedian = 0.0;
    if ((iSize % 2) == 0) {
        dMedian = (array[iSize/2] + array[(iSize/2) - 1])/2.0;
    } else {
        dMedian = array[iSize/2];
    }
    return dMedian;
}


/* vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 : */
