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
  // model_ = GetParentModel(_sensor);

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

  arucoMarker_pub_ = node_handle_->Advertise<sensor_msgs::msgs::ArucoMarker>(topicName, 100);

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
  auto mo = world_->ModelByName("land_pad");
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
  this->marker_dict = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dict_id));  
  this->detectorParams = cv::aruco::DetectorParameters::create();
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
  ignition::math::Vector3d& pos_W_I = T_W_I.Pos();
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

      /* Convert tvec FRD [mm] to vc-NED [m] */
      ignition::math::Vector3d body_pose(tvec[0](0) / 1000, tvec[0](1) / 1000, tvec[0](2) / 1000);
      ignition::math::Quaterniond q_FLU_to_NED = q_ENU_to_NED * att_W_I;
      ignition::math::Quaterniond q_nb = q_FLU_to_NED * q_FLU_to_FRD.Inverse();
      ignition::math::Vector3d vcNED_pose =  q_nb.RotateVector(body_pose);

      /* Get the current simulation time */
      #if GAZEBO_MAJOR_VERSION >= 9
        common::Time now = world_->SimTime();
      #else
        common::Time now = world_->GetSimTime();
      #endif

      /* Send the message */
      arucoMarker_message.set_time_usec(now.Double() * 1e6);
      arucoMarker_message.set_pos_x(vcNED_pose[0]);
      arucoMarker_message.set_pos_y(vcNED_pose[1]);
      arucoMarker_message.set_pos_z(vcNED_pose[2]);

      arucoMarker_pub_->Publish(arucoMarker_message);
    }
  }
}
