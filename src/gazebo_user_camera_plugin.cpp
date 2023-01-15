/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief User Camera Plugin
 *
 * This plugin controls the camera view in gzclient
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include "Int32.pb.h"
#include "gazebo_user_camera_plugin.h"

using namespace gazebo;

GZ_REGISTER_GUI_PLUGIN(UserCameraPlugin)

UserCameraPlugin::UserCameraPlugin()
  : GUIPlugin()
{
  // This is a workaround to make the button invisible
  this->resize(0, 0);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  update_connection_ =
      event::Events::ConnectPreRender(
          boost::bind(&UserCameraPlugin::OnUpdate, this));

  const char *model = std::getenv("PX4_SIM_MODEL");
  if (model) {
    model_name_ = std::string(model);

    // Remove gazebo_ substring
    // Model name in `PX4_SIM_MODEL` includes the gazebo substring after
    // https://github.com/PX4/PX4-Autopilot/pull/20867
    std:: string prefix = "gazebo-classic_";
    std::size_t ind = model_name_.find(prefix);
    if(ind !=std::string::npos){
        model_name_.erase(ind,prefix.length());
    }
  }
}

/////////////////////////////////////////////////
UserCameraPlugin::~UserCameraPlugin()
{
}

void UserCameraPlugin::OnUpdate() {

  rendering::UserCameraPtr user_camera = gui::get_active_camera();

  if(user_camera && !model_name_.empty()) {
    user_camera->TrackVisual(model_name_);
  }

}
