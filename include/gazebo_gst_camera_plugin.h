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
#pragma once

#include <string>
#include <mutex>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <gst/gst.h>

namespace gazebo
{
/**
 * @class GstCameraPlugin
 * A Gazebo plugin that can be attached to a camera and then streams the video data using gstreamer.
 * It streams to a configurable UDP IP and UDP Port, defaults are respectively 127.0.0.1 and 5600.
 *
 * Connect to the stream via command line with:
 * gst-launch-1.0  -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
 *  ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false
 */
class GAZEBO_VISIBLE GstCameraPlugin : public SensorPlugin
{
  public: GstCameraPlugin();

  public: virtual ~GstCameraPlugin();

  public: virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);

  public: virtual void OnNewFrame(const unsigned char *image,
		unsigned int width, unsigned int height,
		unsigned int depth, const std::string &format);

  public: void startGstThread();
  public: void stopGstThread();
  public: void gstCallback(GstElement *appsrc);

  public: void cbVideoStream(const boost::shared_ptr<const msgs::Int> &_msg);
  private: void startStreaming();
  private: void stopStreaming();

  protected: unsigned int width, height, depth;
  float rate;
  protected: std::string format;

  protected:
    std::string udpHost;
    int udpPort;
    bool useRtmp;
    std::string rtmpLocation;
    bool useCuda;
    bool useCudaCustomParams;
    bool useVaapi;
    bool convFbImgToI420;

  protected: sensors::CameraSensorPtr parentSensor;
  protected: rendering::CameraPtr camera;

  private: event::ConnectionPtr newFrameConnection;

  private: transport::NodePtr node_handle_;
  private: std::string namespace_;

  private: transport::SubscriberPtr mVideoSub;
  private: pthread_t mThreadId;
  private: const std::string mTopicName = "~/video_stream";
  private: bool mIsActive;

  GMainLoop *gst_loop;
  GstElement *source;
};

} /* namespace gazebo */
