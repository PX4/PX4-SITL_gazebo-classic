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
#ifndef _GAZEBO_CAMERA_PLUGIN_HH_
#define _GAZEBO_CAMERA_PLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "opticalFlow.pb.h"

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

namespace gazebo
{
  class GAZEBO_VISIBLE CameraPlugin : public SensorPlugin
  {
    public: CameraPlugin();

    /// \brief Destructor
    public: virtual ~CameraPlugin();

    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    protected: unsigned int width, height, depth;
    protected: std::string format;

    protected: sensors::CameraSensorPtr parentSensor;
    protected: rendering::CameraPtr camera;

    private: event::ConnectionPtr newFrameConnection;

    private: 
	 vector<Point2f> featuresPrevious;
     vector<Point2f> featuresCurrent;
     vector<Point2f> featuresNextPos;
     vector<uchar> featuresFound;

     Mat err;
     Mat old_gray;
     Mat frame_gray;

	 transport::PublisherPtr opticalFlow_pub_;
	 transport::NodePtr node_handle_;
	 opticalFlow_msgs::msgs::opticalFlow opticalFlow_message;
	 std::string namespace_;
	 const string topicName = "opticalFlow";

     const int maxfeatures = 20;
     const double qualityLevel = 0.01;
     const double minDistance = 10;
     const int blockSize = 5;
     const bool useHarrisDetector = false;
     const double k = 0.04;

  };
}
#endif
