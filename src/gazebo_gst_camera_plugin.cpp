/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include "gazebo_gst_camera_plugin.h"
#include <gst/app/gstappsrc.h>

#include <math.h>
#include <string>
#include <iostream>
#include <thread>
#include <time.h>
#include "Int32.pb.h"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace gazebo;
using namespace cv;

GZ_REGISTER_SENSOR_PLUGIN(GstCameraPlugin)


static void* start_thread(void* param) {
  GstCameraPlugin* plugin = (GstCameraPlugin*)param;
  plugin->startGstThread();
  return nullptr;
}

/////////////////////////////////////////////////
void GstCameraPlugin::startGstThread() {
  gst_init(nullptr, nullptr);

  this->gst_loop = g_main_loop_new(nullptr, FALSE);
  if (!this->gst_loop) {
    gzerr << "Create loop failed. \n";
    return;
  }

  GstElement* pipeline = gst_pipeline_new(nullptr);
  if (!pipeline) {
    gzerr << "ERR: Create pipeline failed. \n";
    return;
  }

  GstElement* source = gst_element_factory_make("appsrc", nullptr);
  GstElement* queue = gst_element_factory_make("queue", nullptr);
  GstElement* converter  = gst_element_factory_make("videoconvert", nullptr);

  GstElement* encoder;
  if (useCuda) {
    encoder = gst_element_factory_make("nvh264enc", nullptr);
    g_object_set(G_OBJECT(encoder), "bitrate", 800, "preset", 1, nullptr);
  } else {
    encoder = gst_element_factory_make("x264enc", nullptr);
    g_object_set(G_OBJECT(encoder), "bitrate", 800, "speed-preset", 6, "tune", 4, "key-int-max", 10, nullptr);
  }

  GstElement* payloader;
  GstElement* sink;

  if (useRtmp) {
    payloader = gst_element_factory_make("flvmux", nullptr);
    sink = gst_element_factory_make("rtmpsink", nullptr);
    g_object_set(G_OBJECT(sink), "location", this->rtmpLocation.c_str(), nullptr);
  } else {
    payloader = gst_element_factory_make("rtph264pay", nullptr);
    sink  = gst_element_factory_make("udpsink", nullptr);
    g_object_set(G_OBJECT(sink), "host", this->udpHost.c_str(), "port", this->udpPort, nullptr);
  }

  if (!source || !queue || !converter || !encoder || !payloader || !sink) {
    gzerr << "ERR: Create elements failed. \n";
    return;
  }

  // gzerr <<"width"<< this->width<<"\n";
  // gzerr <<"height"<< this->height<<"\n";
  // gzerr <<"rate"<< this->rate<<"\n";

  // Configure source element
  g_object_set(G_OBJECT(source), "caps",
      gst_caps_new_simple ("video/x-raw",
        "format", G_TYPE_STRING, "I420",
        "width", G_TYPE_INT, this->width,
        "height", G_TYPE_INT, this->height,
        "framerate", GST_TYPE_FRACTION, (unsigned int)this->rate, 1, nullptr),
      "is-live", TRUE,
      "do-timestamp", TRUE,
      "stream-type", GST_APP_STREAM_TYPE_STREAM,
      "format", GST_FORMAT_TIME, nullptr);

  // Connect all elements to pipeline
  gst_bin_add_many(GST_BIN(pipeline), source, queue, converter, encoder, payloader, sink, nullptr);

  // Link all elements
  if (gst_element_link_many(source, queue, converter, encoder, payloader, sink, nullptr) != TRUE) {
    gzerr << "ERR: Link all the elements failed. \n";
    return;
  }

  this->source = source;
  gst_object_ref(this->source);

  // Start
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  g_main_loop_run(this->gst_loop);

  // Clean up
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(pipeline));
  gst_object_unref(this->source);
  g_main_loop_unref(this->gst_loop);
  this->gst_loop = nullptr;
  this->source = nullptr;
}

/////////////////////////////////////////////////
void GstCameraPlugin::stopGstThread()
{
  if(this->gst_loop) {
    g_main_loop_quit(this->gst_loop);
  }
}

/////////////////////////////////////////////////
GstCameraPlugin::GstCameraPlugin()
: SensorPlugin(), width(0), height(0), depth(0), gst_loop(nullptr), source(nullptr), mIsActive(false)
{
}

/////////////////////////////////////////////////
GstCameraPlugin::~GstCameraPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();
  if (this->gst_loop) {
    g_main_loop_quit(this->gst_loop);
  }
}

/////////////////////////////////////////////////
void GstCameraPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  if (!sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor =
#if GAZEBO_MAJOR_VERSION >= 7
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
#else
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
#endif

  if (!this->parentSensor)
  {
    gzerr << "GstCameraPlugin requires a CameraSensor.\n";
#if GAZEBO_MAJOR_VERSION >= 7
    if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor))
#else
    if (boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor))
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
    gzerr << "GstCameraPlugin not attached to a camera sensor\n";
    return;
  }

#if GAZEBO_MAJOR_VERSION >= 7
  this->width = this->camera->ImageWidth();
  this->height = this->camera->ImageHeight();
  this->depth = this->camera->ImageDepth();
  this->format = this->camera->ImageFormat();
  this->rate = this->camera->RenderRate();
#else
  this->width = this->camera->GetImageWidth();
  this->height = this->camera->GetImageHeight();
  this->depth = this->camera->GetImageDepth();
  this->format = this->camera->GetImageFormat();
  this->rate = this->camera->GetRenderRate();
#endif

 if (!isfinite(this->rate)) {
   this->rate =  60.0;
 }

  if (sdf->HasElement("robotNamespace"))
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_gst_camera_plugin] Please specify a robotNamespace.\n";

  this->udpHost = "127.0.0.1";
  const char *host_ip = std::getenv("PX4_VIDEO_HOST_IP");
  if (host_ip) {
  	this->udpHost = std::string(host_ip);
  } else if (sdf->HasElement("udpHost")) {
  	this->udpHost = sdf->GetElement("udpHost")->Get<string>();
  }

  this->udpPort = 5600;
  if (sdf->HasElement("udpPort")) {
    this->udpPort = sdf->GetElement("udpPort")->Get<int>();
  }
  gzwarn << "[gazebo_gst_camera_plugin] Streaming video to ip: " << this->udpHost << " port: "  << this->udpPort << std::endl;

  if (sdf->HasElement("rtmpLocation")) {
    this->rtmpLocation = sdf->GetElement("rtmpLocation")->Get<string>();
    this->useRtmp = true;
  } else {
    this->useRtmp = false;
  }

  // Use CUDA for video encoding
  if (sdf->HasElement("useCuda")) {
    this->useCuda = sdf->GetElement("useCuda")->Get<bool>();
  } else {
    this->useCuda = false;
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Listen to Gazebo topic
  mVideoSub = node_handle_->Subscribe<msgs::Int>(mTopicName, &GstCameraPlugin::cbVideoStream, this);

  // And start by default
  startStreaming();

}

/////////////////////////////////////////////////
void GstCameraPlugin::cbVideoStream(const boost::shared_ptr<const msgs::Int> &_msg)
{
  gzwarn << "Video Streaming callback: " << _msg->data() << "\n";
  int enable = _msg->data();
  if(enable)
    startStreaming();
  else
    stopStreaming();
}

/////////////////////////////////////////////////
void GstCameraPlugin::startStreaming()
{
  if(!mIsActive) {
    this->newFrameConnection = this->camera->ConnectNewImageFrame(
        boost::bind(&GstCameraPlugin::OnNewFrame, this, _1, this->width, this->height, this->depth, this->format));

    this->parentSensor->SetActive(true);

    /* start the gstreamer event loop */
    pthread_create(&mThreadId, NULL, start_thread, this);
    mIsActive = true;
  }

}

/////////////////////////////////////////////////
void GstCameraPlugin::stopStreaming()
{
  if(mIsActive) {
    stopGstThread();

    pthread_join(mThreadId, NULL);

    this->parentSensor->SetActive(false);

    this->newFrameConnection->~Connection();
    mIsActive = false;
  }

}

/////////////////////////////////////////////////
void GstCameraPlugin::OnNewFrame(const unsigned char * image,
                              unsigned int width,
                              unsigned int height,
                              unsigned int depth,
                              const std::string &format)
{

#if GAZEBO_MAJOR_VERSION >= 7
  image = this->camera->ImageData(0);
#else
  image = this->camera->GetImageData(0);
#endif

  // Alloc buffer
  const guint size = width * height * 1.5;
  GstBuffer* buffer = gst_buffer_new_allocate(NULL, size, NULL);

  if (!buffer) {
    gzerr << "gst_buffer_new_allocate failed" << endl;
    return;
  }

  GstMapInfo map;

  if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    gzerr << "gst_buffer_map failed" << endl;
    return;
  }

  // Color Conversion from RGB to YUV
  Mat frame = Mat(height, width, CV_8UC3);
  Mat frameYUV = Mat(height, width, CV_8UC3);
  frame.data = (uchar*)image;

  cvtColor(frame, frameYUV, COLOR_RGB2YUV_I420);
  memcpy(map.data, frameYUV.data, size);
  gst_buffer_unmap(buffer, &map);

  GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(this->source), buffer);

  if (ret != GST_FLOW_OK) {
    /* something wrong, stop pushing */
    gzerr << "gst_app_src_push_buffer failed" << endl;
    g_main_loop_quit(this->gst_loop);
  }
}
