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

#ifndef _GZRS_PLUGIN_HH_
#define _GZRS_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>
#include <string>

namespace gazebo
{
  // Forward declare private data class
  struct RealSensePluginPrivate;

  /// \brief A plugin that simulates Real Sense camera streams.
  class GAZEBO_VISIBLE RealSensePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public:
    RealSensePlugin();

    /// \brief Destructor.
    public:
    ~RealSensePlugin();

    // Documentation Inherited.
    public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback for the World Update event.
    public:
    void OnUpdate();

    /// \brief Callback that publishes a received Depth Camera Frame as an
    /// ImageStamped message.
    public:
    virtual void OnNewDepthFrame() const;

    /// \brief Callback that publishes a received Camera Frame as an
    /// ImageStamped message.
    public:
    virtual void OnNewFrame(const rendering::CameraPtr cam,
                            const transport::PublisherPtr pub) const;

    /// \brief Private data pointer.
    private: std::unique_ptr<RealSensePluginPrivate> dataPtr;
  };
}
#endif

