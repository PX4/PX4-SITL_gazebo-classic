/*
 * Copyright (C) 2012-2017 Open Source Robotics Foundation
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
 * @brief GPS Plugin
 *
 * This plugin publishes GPS data to be used and propagated
 *
 * @author Amy Wagoner <arwagoner@gmail.com>
 */

#ifndef _GAZEBO_GPS_PLUGIN_HH_
#define _GAZEBO_GPS_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/math/gzmath.hh"

#include "Gps.pb.h"

#include "common.h"
#include <sdf/sdf.hh>
#include <stdio.h>

namespace gazebo
{
    class GAZEBO_VISIBLE GpsPlugin : public ModelPlugin
    {
        public:
            GpsPlugin();
            virtual ~GpsPlugin();
            
        protected:
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            virtual void OnUpdate(const common::UpdateInfo&);
            
        private:
            
            void reproject(math::Vector3& pos, double& lat_rad, double& lon_rad);
            
            std::string namespace_;
            std::default_random_engine random_generator_;
            std::normal_distribution<float> standard_normal_distribution_;
            bool gps_noise_;
            
            physics::ModelPtr model_;
            physics::WorldPtr world_;
            event::ConnectionPtr updateConnection_;
            transport::PublisherPtr gps_pub_;
            transport::NodePtr node_handle_;
            gps_msgs::msgs::Gps gps_msg;
            
            common::Time last_gps_time_;

            double gps_update_interval_;
            double gps_delay_;
            double std_xy;
            double std_z;
            double lat_rad;
            double lon_rad;
            double dt_gps;
            
            math::Vector3 gps_bias_;
            math::Vector3 noise_gps;
            math::Vector3 random_walk_gps;
            math::Vector3 gravity_W_;
            math::Vector3 velocity_prev_W_;
            
            static const unsigned n_out_max = 16;

            // gps noise parameters
            static constexpr double gps_corellation_time = 30.0; // s
            static constexpr double gps_random_walk = 1.0; // (m/s) / sqrt(hz)
            static constexpr double gps_noise_density = 2e-4; // (m) / sqrt(hz)
    };
}
#endif