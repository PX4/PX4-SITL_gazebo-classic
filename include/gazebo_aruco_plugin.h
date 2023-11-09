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
#ifndef _GAZEBO_ARUCO_PLUGIN_HH_
#define _GAZEBO_ARUCO_PLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/ImuSensor.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

#include <stdio.h>
#include <boost/bind.hpp>

#include <common.h>

#include "TargetRelative.pb.h"

#include <opencv2/opencv.hpp>

#include <opencv2/aruco.hpp>
// Check for OpenCV version 4.7 or later
#if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
#include <opencv2/objdetect/aruco_dictionary.hpp>
#else
#include <opencv2/aruco/dictionary.hpp>
#endif

#include <opencv2/core.hpp>

#include <vector>

#include <iostream>
#include <ignition/math.hh>

#define M_PI 3.14159265358979323846
#define M_TWOPI (M_PI * 2.0)
#define D2R 0.017453292519943295769 // this is PI/180.0

using namespace cv;
using namespace std;

namespace gazebo
{
  // static const std::string kDefaultGyroTopic = "/px4flow/imu";

  class GAZEBO_VISIBLE arucoMarkerPlugin : public SensorPlugin
  {
    public:
      arucoMarkerPlugin();
      virtual ~arucoMarkerPlugin();
      virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
      virtual void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height, const std::string &_format);

    protected:
      unsigned int width, height;
      std::string format;
      sensors::CameraSensorPtr parentSensor;
      rendering::CameraPtr camera;
      physics::WorldPtr world_;
      physics::ModelPtr model_;

    private:
      event::ConnectionPtr newFrameConnection;
      transport::PublisherPtr arucoMarker_pub_;
      transport::NodePtr node_handle_;
      sensor_msgs::msgs::TargetRelative targetRelative_message;
      std::string namespace_;
      std::string model_name_;
      std::string land_pad_name_;
      
      float markerLength_mm; 

      cv::Ptr<cv::aruco::Dictionary> marker_dict;
      cv::Ptr<cv::aruco::DetectorParameters> detectorParams;

      cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << 3.9877559882561974e+02, 0., 3.2546639260681752e+02, 0., 3.9651789974056226e+02, 2.3906162259594086e+02, 0., 0., 1.);
      cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << -7.4500551440667862e-02, -1.5464660896318899e-01, -4.7134015104217627e-04, 3.6767321906851489e-03, 2.8738933210835571e-01);

      void computeRPY(cv::Mat R, float &roll, float &pitch, float &yaw);
      float wrap_2pi(const float &yaw);
  };
}
#endif
