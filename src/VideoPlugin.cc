/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "VideoPlugin.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>

#define DEPTH_PUB_FREQ_HZ 60
#define COLOR_PUB_FREQ_HZ 60
#define IRED1_PUB_FREQ_HZ 60
#define IRED2_PUB_FREQ_HZ 60

#define COLOR_CAMERA_NAME "_color"

#define COLOR_CAMERA_TOPIC "color"

#define DEPTH_NEAR_CLIP_M 0.3
#define DEPTH_FAR_CLIP_M 10.0
#define DEPTH_SCALE_M 0.001

using namespace gazebo;

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(VideoPlugin)

namespace gazebo {
struct VideoPluginPrivate {
  /// \brief Pointer to the model containing the plugin.
public:
  physics::ModelPtr rsModel;
  physics::WorldPtr world;
  rendering::CameraPtr colorCam;
  transport::NodePtr transportNode;
  transport::PublisherPtr colorPub;
  event::ConnectionPtr newColorFrameConn;
  event::ConnectionPtr updateConnection;
};
} // namespace gazebo

/////////////////////////////////////////////////
VideoPlugin::VideoPlugin() : dataPtr(new VideoPluginPrivate) {
  this->dataPtr->colorCam = nullptr;
}

/////////////////////////////////////////////////
VideoPlugin::~VideoPlugin() { delete data_; }

/////////////////////////////////////////////////

void VideoPlugin::video_trigger(
    std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (recording_ == false) {
    std::cout << "start recording" << std::endl;
    recording_ = true;
  } else {
    std::cout << "stop recording" << std::endl;
    recording_ = false;
    videoPtr_->release();
  }
  response->success = true;
}

void VideoPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  recording_ = false;
  // Output the name of the model
  std::cout << std::endl
            << "VideoPlugin: The rs_camera plugin is attach to model: "
            << _model->GetName() << std::endl;

  // Read in parameters
  if (_sdf->HasElement("fileName")) {
    file_name_ = _sdf->GetElement("fileName")->Get<std::string>();
  } else {
    std::cout << "[Gazebo Video] Please specify the filename for the video."
              << std::endl;
  }
  // if (!(cap_.isOpened())) {
  //   std::cout << "Could not save video to file" << std::endl;
  //   return;
  // }
  // Store a pointer to the this model
  this->dataPtr->rsModel = _model;

  // Store a pointer to the world
  this->dataPtr->world = this->dataPtr->rsModel->GetWorld();

  // Sensors Manager
  sensors::SensorManager *smanager = sensors::SensorManager::Instance();

  std::cout << "connecting to camera: "
            << this->dataPtr->rsModel->GetName() + COLOR_CAMERA_NAME << "..."
            << std::endl;
  this->dataPtr->colorCam =
      std::dynamic_pointer_cast<sensors::CameraSensor>(
          smanager->GetSensor(this->dataPtr->rsModel->GetName() +
                              COLOR_CAMERA_NAME))
          ->Camera();

  if (!this->dataPtr->colorCam) {
    std::cerr << "VideoPlugin: Color Camera has not been found" << std::endl;
    return;
  } else {
    std::cout << "VideoPlugin: Color Camera has been found" << std::endl;
  }

  // set up video recorder
  int frame_width = this->dataPtr->colorCam->ImageWidth();
  int frame_height = this->dataPtr->colorCam->ImageHeight();

  std::cout << "video format:\t[ " << frame_width << "\t" << frame_height
            << " ]" << std::endl;
  // Define the codec and create VideoWriter object.The output
  // is stored in 'outcpp.avi' file.
  std::cout << "Saving video to following file: " << file_name_ << std::endl;
  videoPtr_ = std::make_unique<cv::VideoWriter>(
      "/home/aurel/Videos/test.avi",
      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
      cv::Size(frame_width, frame_height));
  // cv::VideoWriter::fourcc('M', 'J', 'P', 'G')
  if (!videoPtr_->isOpened()) {
    std::cerr << "FAIL: Could NOT open the output video file for write"
              << std::endl;
    return;
  } else {
    std::cout << "SUCCESS: Could open the output video file for write"
              << std::endl;
  }
  // create Data array
  data_ = new char[(this->dataPtr->colorCam->ImageDepth()) *
                       (this->dataPtr->colorCam->ImageHeight()) *
                       (this->dataPtr->colorCam->ImageWidth()) +
                   1];
  // Setup Transport Node
  this->dataPtr->transportNode = transport::NodePtr(new transport::Node());

  this->dataPtr->transportNode->Init(this->dataPtr->world->Name());
  // this->dataPtr->transportNode->Init(this->dataPtr->rsModel->GetName());

  // Setup Publishers
  std::string rsTopicRoot = "~/" + this->dataPtr->rsModel->GetName() +
                            "/rs/stream/"; // here lies the problem...

  this->dataPtr->colorPub =
      this->dataPtr->transportNode->Advertise<msgs::ImageStamped>(
          rsTopicRoot + COLOR_CAMERA_TOPIC, 1, COLOR_PUB_FREQ_HZ);

  this->dataPtr->newColorFrameConn =
      this->dataPtr->colorCam->ConnectNewImageFrame(
          std::bind(&VideoPlugin::OnNewColorFrame, this,
                    this->dataPtr->colorCam, this->dataPtr->colorPub));

  // Listen to the update event
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VideoPlugin::OnUpdate, this));

  // ROS2 stuff
  // Initialize ROS2, if it has not already been initialized.
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);
  }

  // create ROS2 publisher node
  this->ros_node_ = rclcpp::Node::make_shared(
      "video_node"); // TODO change string to something esle

  service_ = this->ros_node_->create_service<std_srvs::srv::Trigger>(
      "videoTrigger", std::bind(&VideoPlugin::video_trigger, this,
                                std::placeholders::_1, std::placeholders::_2));
}

/////////////////////////////////////////////////
void VideoPlugin::OnNewColorFrame(const rendering::CameraPtr cam,
                                  const transport::PublisherPtr pub) const {
  if (recording_ == true) {
    memcpy(data_, cam->ImageData(),
           (cam->ImageDepth()) * (cam->ImageHeight()) * (cam->ImageWidth()));

    cv::Mat image(int(cam->ImageHeight()), int(cam->ImageWidth()), CV_8UC3,
                  data_);

    videoPtr_->write(image);
  }
}

/////////////////////////////////////////////////
void VideoPlugin::OnUpdate() { rclcpp::spin_some(this->ros_node_); }
