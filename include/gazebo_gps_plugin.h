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

#include "SITLGps.pb.h"

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/util/system.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/math/gzmath.hh"

#include "common.h"

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
            gps_msgs::msgs::SITLGps gps_msg;

	    common::Time last_gps_time_;
	    common::Time last_time_;

	    // Set global reference point
	    // Zurich Irchel Park: 47.397742, 8.545594, 488m
	    // Seattle downtown (15 deg declination): 47.592182, -122.316031, 86m
	    // Moscow downtown: 55.753395, 37.625427, 155m

	    // The home position can be specified using the environment variables:
	    // PX4_HOME_LAT, PX4_HOME_LON, and PX4_HOME_ALT

	    // Zurich Irchel Park
	    double lat_home = 47.397742 * M_PI / 180.0;  // rad
	    double lon_home = 8.545594 * M_PI / 180.0;  // rad
	    double alt_home = 488.0; // meters
	    // Seattle downtown (15 deg declination): 47.592182, -122.316031
	    // static const double lat_home = 47.592182 * M_PI / 180;  // rad
	    // static const double lon_home = -122.316031 * M_PI / 180;  // rad
	    // static const double alt_home = 86.0; // meters
	    static constexpr const double earth_radius = 6353000.0;  // m

	    // gps delay related
	    static constexpr double gps_update_interval_ = 0.2;  // seconds (5hz)
	    static constexpr double gps_delay = 0.3; // seconds
	    static constexpr int gps_buffer_size_max = 1000;
	    std::queue<gps_msgs::msgs::SITLGps> gps_delay_buffer;

            double lat_rad;
            double lon_rad;

            math::Vector3 gps_bias;
            math::Vector3 noise_gps_pos;
            math::Vector3 noise_gps_vel;
            math::Vector3 random_walk_gps;
            math::Vector3 gravity_W_;
            math::Vector3 velocity_prev_W_;

            static const unsigned n_out_max = 16;

            // gps noise parameters
            static constexpr double gps_corellation_time = 30.0; // s

            // Assuming 0.1 Hz and 1 m error

            static constexpr double gps_random_walk = 0.01; // (m/s) / sqrt(hz)
            static constexpr double gps_noise_density = 2e-4; // (m) / sqrt(hz)
    };
}
#endif
