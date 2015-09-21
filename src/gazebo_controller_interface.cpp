/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "gazebo_controller_interface.h"

namespace gazebo {

GazeboControllerInterface::~GazeboControllerInterface() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboControllerInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;

  world_ = model_->GetWorld();

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboControllerInterface::OnUpdate, this, _1));
  printf("Hello World!\n");

}

// This gets called by the world update start event.
void GazeboControllerInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {

  common::Time now = world_->GetSimTime();
}


GZ_REGISTER_MODEL_PLUGIN(GazeboControllerInterface);
}
