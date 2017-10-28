/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
 * Copyright 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
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

#ifndef HIL_INTERFACE_NODE_H_
#define HIL_INTERFACE_NODE_H_

#include <Eigen/Dense>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros_msgs/HilGPS.h>
#include <mavros_msgs/HilSensor.h>
#include <mavros_msgs/HilStateQuaternion.h>
#include <mavros_msgs/HilControls.h>

namespace hil_interface {
// message_filters ApproximateTime policies
using SyncGPSPolicy = message_filters::sync_policies::ApproximateTime
		< sensor_msgs::NavSatFix,		// GPS
		geometry_msgs::TwistStamped >;		// Ground Speed
using SyncSensorPolicy = message_filters::sync_policies::ApproximateTime
		< geometry_msgs::TwistStamped,		// Air Speed
		sensor_msgs::Imu,			// IMU
		sensor_msgs::MagneticField,		// Magnetometer
		sensor_msgs::FluidPressure >;		// Pressure Sensor
using SyncStatePolicy = message_filters::sync_policies::ApproximateTime
		< geometry_msgs::TwistStamped,		// Air Speed
		sensor_msgs::NavSatFix,			// GPS
		geometry_msgs::TwistStamped,		// Ground Speed
		sensor_msgs::Imu,			// IMU
		sensor_msgs::MagneticField >;		// Magnetometer

// message_filters Synchronizers
using SyncGPS = message_filters::Synchronizer<SyncGPSPolicy>;
using SyncSensor = message_filters::Synchronizer<SyncSensorPolicy>;
using SyncState = message_filters::Synchronizer<SyncStatePolicy>;

// Default values
static constexpr bool kDefaultSensorLevelHil = true;
static constexpr double kDefaultHilFrequency = 100.0;
static constexpr double kDefaultGpsFrequency = 5.0;
static const std::string kDefaultHilGPSPubTopic = "/mavros/hil/gps";
static const std::string kDefaultHilSensorPubTopic = "/mavros/hil/imu_ned";
static const std::string kDefaultHilStatePubTopic = "/mavros/hil/state";
static const std::string kDefaultHilControlsSubTopic = "/mavros/hil/controls";
static const std::string kDefaultPressureSubTopic = "air_pressure";

// Constants
static constexpr float kAirDensity_kg_per_m3 = 1.18;
static constexpr float kGravityMagnitude_m_per_s2 = 9.8068;
static constexpr float kStandardPressure_Pascal = 10.1325;
static constexpr float kTemperature_C = 15.0;
static constexpr int kFixNone = 0;
static constexpr int kFix3D = 3;
static constexpr int kHDOP = 100;
static constexpr int kVDOP = 100;
static constexpr int kSatellitesVisible = 4;
static constexpr int kUnknown = 65535;
static constexpr int kAllFieldsUpdated = 4095;

// Conversions
static constexpr float kFeetToMeters = 0.3048;
static constexpr float kPressureToAltExp = 0.190284;
static constexpr float kPressureToAltMult = 145366.45;
static constexpr float kSecToNsec = 1e9;

class HilInterfaceNode {
public:
	HilInterfaceNode();
	virtual ~HilInterfaceNode();

	//! @brief Main execution loop.
	void MainTask();

	/**
	 * brief Sync Callback for handling HIL GPS data and publish MAVROS HilGPS messages.
	 * param[in] gps_msg A GPS message.
	 * param[in] ground_speed_msg A ground speed message.
	 */
	void HilGPSPubCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg,
			const geometry_msgs::TwistStampedConstPtr &ground_speed_msg);

	/**
	 * brief Sync Callback for handling HIL Sensor data and publish MAVROS HilSensor messages.
	 * param[in] gps_msg A GPS message.
	 * param[in] ground_speed_msg A ground speed message.
	 */
	void HilSensorPubCallback(const geometry_msgs::TwistStampedConstPtr& air_speed_msg,
			const sensor_msgs::ImuConstPtr& imu_msg,
			const sensor_msgs::MagneticFieldConstPtr &mag_msg,
			const sensor_msgs::FluidPressureConstPtr &pressure_msg);

	/**
	 * brief Sync Callback for handling HIL State data and publish MAVROS HilStateQuaternion messages.
	 * param[in] air_speed_msg An Air Speed message.
	 * param[in] gps_msg A GPS message.
	 * param[in] ground_speed_msg A ground speed message.
	 * param[in] imu_msg An IMU message.
	 * param[in] mag_msg A Magnetometer message.
	 */
	void HilStateQuaternionPubCallback(const geometry_msgs::TwistStampedConstPtr& air_speed_msg,
			const sensor_msgs::NavSatFixConstPtr& gps_msg,
			const geometry_msgs::TwistStampedConstPtr &ground_speed_msg,
			const sensor_msgs::ImuConstPtr& imu_msg,
			const sensor_msgs::MagneticFieldConstPtr &mag_msg);

	/**
	 * @brief Callback for handling MAVROS HilControls messages.
	 * param[in] hil_controls_msg A HilControls message.
	 */
	void HilControlsCallback(const mavros_msgs::HilControlsConstPtr& hil_controls_msg);

private:
	//! ROS node handle.
	ros::NodeHandle nh_;

	//! Choose the interface level.
	bool sensor_level_hil;

	//! Pointer for GPS level data messages syncronous subscribers.
	std::unique_ptr<SyncGPS> sync_gps;

	//! Pointer for Sensor level data messages syncronous subscribers.
	std::unique_ptr<SyncSensor> sync_sensor;

	//! Pointer for State level data messages syncronous subscribers.
	std::unique_ptr<SyncState> sync_state;

	//! ROS subscriber for handling TwistStamped Air Speed messages.
	message_filters::Subscriber<geometry_msgs::TwistStamped> air_speed_sub_;

	//! ROS subscriber for handling NavSatFix GPS messages.
	message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub_;

	//! ROS subscriber for handling TwistStamped Ground Speed messages.
	message_filters::Subscriber<geometry_msgs::TwistStamped> ground_speed_sub_;

	//! ROS subscriber for handling Imu messages.
	message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;

	//! ROS subscriber for handling MagneticField messages.
	message_filters::Subscriber<sensor_msgs::MagneticField> mag_sub_;

	//! ROS subscriber for handling FluidPressure sensor messages.
	message_filters::Subscriber<sensor_msgs::FluidPressure> pressure_sub_;

	//! ROS subscriber for handling HilControls messages.
	ros::Subscriber hil_controls_sub_;

	//! ROS publisher for sending actuator commands.
	ros::Publisher actuators_pub_;

	//! ROS publisher for sending MAVROS HilGPS messages.
	ros::Publisher hil_gps_pub_;

	//! ROS publisher for sending MAVROS HilSensor messages.
	ros::Publisher hil_sensor_pub_;

	//! ROS publisher for sending MAVROS HilStateQuaternion messages.
	ros::Publisher hil_state_pub_;

	//! ROS parameters that define the Publisher topics.
	std::string actuators_pub_topic;
	std::string hil_gps_pub_topic;
	std::string hil_sensor_pub_topic;
	std::string hil_state_pub_topic;

	//! ROS parameters that define the Subscriber topics.
	std::string hil_controls_sub_topic;
	std::string air_speed_sub_topic;
	std::string gps_sub_topic;
	std::string ground_speed_sub_topic;
	std::string imu_sub_topic;
	std::string mag_sub_topic;
	std::string pressure_sub_topic;

	//! Object for spinning.
	ros::Rate rate_;

	//! HilGPS message publish rate.
	double gps_freq;

	//! Interval between outgoing HilGPS messages.
	uint64_t gps_interval_nsec_;

	//! Nanosecond portion of the last HilGPS message timestamp.
	uint64_t last_gps_pub_time_nsec_;
};
}	// namespace hil_interface

#endif	// HIL_INTERFACE_NODE_H_
