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
/**
 * @brief Aruco marker Plugin
 *
 * This plugin detects an aruco marker and publishes the relative position of the marker in vehicle-carried NED frame.
 *
 * @author Jonas Perolinir <jonas.perolini@epfl.ch>
 */

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include "gazebo_aruco_plugin.h"

#include <math.h>
#include <string>
#include <iostream>
#include <boost/algorithm/string.hpp>

using namespace cv;
using namespace std;

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(arucoMarkerPlugin)

/////////////////////////////////////////////////
arucoMarkerPlugin::arucoMarkerPlugin()
: SensorPlugin(), width(0), height(0)
{

}

/////////////////////////////////////////////////
arucoMarkerPlugin::~arucoMarkerPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();
}

/////////////////////////////////////////////////
void arucoMarkerPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{

  if (!_sensor){
    gzerr << "Invalid sensor pointer.\n";
    return; 
  }

  this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "arucoMarkerPlugin not attached to a camera sensor\n";
    return;
  }

  this->world_ = physics::get_world(this->parentSensor->WorldName());

  float hfov;
  float update_rate = -1; 

#if GAZEBO_MAJOR_VERSION >= 7
  this->camera = this->parentSensor->Camera();
  this->width = this->camera->ImageWidth();
  this->height = this->camera->ImageHeight();
  this->format = this->camera->ImageFormat();
  hfov = float(this->camera->HFOV().Degree());
  const string scopedName = _sensor->ParentName();
#else
  this->camera = this->parentSensor->GetCamera();
  this->width = this->camera->GetImageWidth();
  this->height = this->camera->GetImageHeight();
  this->format = this->camera->GetImageFormat();
  hfov = float(this->camera->GetHFOV().Degree());
  const string scopedName = _sensor->GetParentName();
#endif

  if (_sdf->HasElement("landPadName")) {
    land_pad_name_ = _sdf->GetElement("landPadName")->Get<std::string>();
  } else {
    gzwarn << "[Gazebo Aruco Plugin] Please specify a land pad name (model on which the ArUco marker is attached).\n";
  }

  if (_sdf->HasElement("update_rate")) {
    update_rate = _sdf->GetElement("update_rate")->Get<int>();
  }

  std::cout << "[Gazebo Aruco Plugin] Image info -- width: " << this->width << " height: " << this->height << " format: " <<  this->format << " camera hov: " << hfov << " [deg]" << std::endl; 

  if (_sdf->HasElement("robotNamespace")){
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  else{
    gzwarn << "[Gazebo Aruco Plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  string topicName = "~/" + scopedName + "/arucoMarker";
  boost::replace_all(topicName, "::", "/");

  arucoMarker_pub_ = node_handle_->Advertise<sensor_msgs::msgs::TargetRelative>(topicName, 100);

  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      boost::bind(&arucoMarkerPlugin::OnNewFrame, this, _1, this->width, this->height, this->format));

  this->parentSensor->SetActive(true);

  // Get the root model name
  vector<std::string> names_splitted;
  boost::split(names_splitted, scopedName, boost::is_any_of("::"));
  names_splitted.erase(std::remove_if(begin(names_splitted), end(names_splitted),
                            [](const string& name)
                            { return name.size() == 0; }), end(names_splitted));

  // get the root model name
  const string rootModelName = names_splitted.front();

  // store the model name
  model_name_ = names_splitted.at(0);

  std::cout << "[Gazebo Aruco Plugin] Camera attached to: "<< model_name_ << std::endl; 

  // Store the pointer to the model.
  if (model_ == NULL)
#if GAZEBO_MAJOR_VERSION >= 9
    model_ = world_->ModelByName(model_name_);
#else
    model_ = world_->GetModel(model_name_);
#endif

  /* Get the land pad size */
  auto mo = world_->ModelByName(land_pad_name_);
  auto link = mo->GetLink("link"); 
  auto box = link->BoundingBox();

  double box_length = box.XLength(); 
  double box_width = box.YLength(); 

  if (box_length != box_width)
  {
    std::cout << "[Gazebo Aruco Plugin] WARNING: land pad not square." << std::endl;
  }

  std::cout << "[Gazebo Aruco Plugin] Land pad size: " << box_length << " x " << box_width << " [m]" << std::endl; 

  //init marker detection
  this->markerLength_mm = 6.f/8 * 1000 * box_length; 
  int dict_id = 0; 
  // Check for OpenCV version 4.7 or later
  #if CV_VERSION_MAJOR >= 4 && CV_VERSION_MINOR >= 7
    this->marker_dict = makePtr<aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)); 
    this->detectorParams = makePtr<aruco::DetectorParameters>(cv::aruco::DetectorParameters());
  #else
    this->marker_dict = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dict_id));  
    this->detectorParams = cv::aruco::DetectorParameters::create();
  #endif
}

/////////////////////////////////////////////////
void arucoMarkerPlugin::OnNewFrame(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              const std::string &_format)
{

  //get data depending on gazebo version
  #if GAZEBO_MAJOR_VERSION >= 7
    _image = this->camera->ImageData(0);
    double frame_time = this->camera->LastRenderWallTime().Double();
  #else
    _image = this->camera->GetImageData(0);
    double frame_time = this->camera->GetLastRenderWallTime().Double();
  #endif

  #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d T_W_I = model_->WorldPose();
  #else
    ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(model_->GetWorldPose());
  #endif

  /* Get the attitude of the drone to convert observation from body to vehicle-carried NED frame */
  ignition::math::Quaterniond& att_W_I = T_W_I.Rot();

  /* Get camera frame */
  cv::Mat frame_gray = cv::Mat(_height, _width, CV_8UC1);
	frame_gray.data = (uchar *)_image;

  /* Marker detection */
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners, rejected_marker;
  cv::aruco::detectMarkers(frame_gray, this->marker_dict, marker_corners, marker_ids, this->detectorParams, rejected_marker);

  if (this->camMatrix.total() != 0 && marker_ids.size() > 0)
  {
    std::vector<cv::Vec3d> rvec, tvec;

    /* Marker pose estimation in FRD [mm]*/
    cv::aruco::estimatePoseSingleMarkers(marker_corners, this->markerLength_mm, this->camMatrix, this->distCoeffs, rvec, tvec);
    
    if(rvec.size() > 0){

      /* Convert tvec [mm] to body frame FRD [m] */
      ignition::math::Vector3d body_pose(tvec[0](0) / 1000, tvec[0](1) / 1000, tvec[0](2) / 1000);

      /* Get the current simulation time */
      #if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world_->SimTime();
      #else
        common::Time now = world_->GetSimTime();
      #endif

      /* Send the message */
      targetRelative_message.set_time_usec(now.Double() * 1e6);
      targetRelative_message.set_pos_x(body_pose[0]);
      targetRelative_message.set_pos_y(body_pose[1]);
      targetRelative_message.set_pos_z(body_pose[2]);

      /* Save the attitude of the drone when the frame was grabbed, will allow to transform from FRD to vc-NED */
      ignition::math::Quaterniond q_FLU_to_NED = q_ENU_to_NED * att_W_I;
      ignition::math::Quaterniond q_nb = q_FLU_to_NED * q_FLU_to_FRD.Inverse();

      targetRelative_message.set_attitude_q_w(q_nb.W());
      targetRelative_message.set_attitude_q_x(q_nb.X());
      targetRelative_message.set_attitude_q_y(q_nb.Y());
      targetRelative_message.set_attitude_q_z(q_nb.Z());

      targetRelative_message.set_std_x(0.5);
      targetRelative_message.set_std_y(0.5);
      targetRelative_message.set_std_z(1.0);
      targetRelative_message.set_yaw_std(0.0);

      /* Convert rvec to the target orientation with respect to the drone's body frame */
      cv::Mat rotMat;
      cv::Rodrigues(rvec[0], rotMat);
      float roll, pitch, yaw;
      computeRPY(rotMat, roll, pitch, yaw);

      ignition::math::Quaterniond q(roll, pitch, yaw);

      targetRelative_message.set_orientation_q_w(q.W());
      targetRelative_message.set_orientation_q_x(q.X());
      targetRelative_message.set_orientation_q_y(q.Y());
      targetRelative_message.set_orientation_q_z(q.Z());

      arucoMarker_pub_->Publish(targetRelative_message);
    }
  }
}

float arucoMarkerPlugin::wrap_2pi(const float &yaw)
{
    float yaw_2pi = yaw;

    if (!(0.f <= yaw && yaw < M_TWOPI))
    {
        yaw_2pi = yaw - M_TWOPI * floor(yaw / M_TWOPI);
    }

    return yaw_2pi;
}

void arucoMarkerPlugin::computeRPY(cv::Mat R, float &roll, float &pitch, float &yaw)
{
    // https://learnopencv.com/rotation-matrix-to-euler-angles/

    // Check for singularity
    float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

    bool singular = sy < 1e-6;
 
    if (!singular)
    {
        roll = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        roll = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        pitch = atan2(-R.at<double>(2,0), sy);
        yaw = 0;
    }
}
