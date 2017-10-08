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

#include "gazebo_hil_interface.h"

namespace hil_interface {
HilInterfaceNode::HilInterfaceNode() :
	sensor_level_hil(kDefaultSensorLevelHil),
	rate_(kDefaultHilFrequency),
	gps_freq(kDefaultGpsFrequency),
	actuators_pub_topic(std::string(mav_msgs::default_topics::COMMAND_ACTUATORS)),
	hil_gps_pub_topic(kDefaultHilGPSPubTopic),
	hil_sensor_pub_topic(kDefaultHilSensorPubTopic),
	hil_state_pub_topic(kDefaultHilStatePubTopic),
	hil_controls_sub_topic(kDefaultHilControlsSubTopic),
	air_speed_sub_topic(std::string(mav_msgs::default_topics::AIR_SPEED)),
	gps_sub_topic(std::string(mav_msgs::default_topics::GPS)),
	ground_speed_sub_topic(std::string(mav_msgs::default_topics::GROUND_SPEED)),
	imu_sub_topic(std::string(mav_msgs::default_topics::IMU)),
	mag_sub_topic(std::string(mav_msgs::default_topics::MAGNETIC_FIELD)),
	pressure_sub_topic(kDefaultPressureSubTopic) {
	double hil_freq;

	nh_.param("sensor_level_hil", sensor_level_hil, kDefaultSensorLevelHil);
	nh_.param("hil_frequency", hil_freq, kDefaultHilFrequency);
	nh_.param("gps_frequency", gps_freq, kDefaultGpsFrequency);
	nh_.param("actuators_pub_topic", actuators_pub_topic, std::string(mav_msgs::default_topics::COMMAND_ACTUATORS));
	nh_.param("hil_gps_pub_topic", hil_gps_pub_topic, kDefaultHilGPSPubTopic);
	nh_.param("hil_sensor_pub_topic", hil_sensor_pub_topic, kDefaultHilSensorPubTopic);
	nh_.param("hil_state_pub_topic", hil_state_pub_topic, kDefaultHilStatePubTopic);
	nh_.param("hil_controls_sub_topic", hil_controls_sub_topic, kDefaultHilControlsSubTopic);
	nh_.param("air_speed_sub_topic", air_speed_sub_topic, std::string(mav_msgs::default_topics::AIR_SPEED));
	nh_.param("gps_sub_topic", gps_sub_topic, std::string(mav_msgs::default_topics::GPS));
	nh_.param("ground_speed_sub_topic", ground_speed_sub_topic, std::string(mav_msgs::default_topics::GROUND_SPEED));
	nh_.param("imu_sub_topic", imu_sub_topic, std::string(mav_msgs::default_topics::IMU));
	nh_.param("mag_sub_topic", mag_sub_topic, std::string(mav_msgs::default_topics::MAGNETIC_FIELD));
	nh_.param("pressure_sub_topic", pressure_sub_topic, kDefaultPressureSubTopic);

	rate_ = ros::Rate(hil_freq);

	// Compute the desired interval between published GPS messages.
	gps_interval_nsec_ = static_cast<uint64_t>(kSecToNsec / gps_freq);

	// Publishers
	actuators_pub_ = nh_.advertise<mav_msgs::Actuators>(actuators_pub_topic, 10);
	hil_gps_pub_ = nh_.advertise<mavros_msgs::HilGPS>(hil_gps_pub_topic, 10);
	hil_sensor_pub_ = nh_.advertise<mavros_msgs::HilSensor>(hil_sensor_pub_topic, 10);
	hil_state_pub_ = nh_.advertise<mavros_msgs::HilStateQuaternion>(hil_state_pub_topic, 10);

	// Subscribers
	hil_controls_sub_ = nh_.subscribe(hil_controls_sub_topic, 1, &HilInterfaceNode::HilControlsCallback, this);

	// Synced subscribers:
	air_speed_sub_.subscribe(nh_, air_speed_sub_topic, 1);
	gps_sub_.subscribe(nh_, gps_sub_topic, 1);
	ground_speed_sub_.subscribe(nh_, ground_speed_sub_topic, 1);
	imu_sub_.subscribe(nh_, imu_sub_topic, 1);
	mag_sub_.subscribe(nh_, mag_sub_topic, 1);
	pressure_sub_.subscribe(nh_, pressure_sub_topic, 1);
}

HilInterfaceNode::~HilInterfaceNode() {}

void HilInterfaceNode::MainTask() {
	while (ros::ok()) {
		if (sensor_level_hil) {
			sync_gps.reset(new SyncGPS(SyncGPSPolicy(10), gps_sub_, ground_speed_sub_));
			sync_gps->registerCallback(boost::bind(&HilInterfaceNode::HilGPSPubCallback, this, _1, _2));

			sync_sensor.reset(new SyncSensor(SyncSensorPolicy(10), air_speed_sub_, imu_sub_, mag_sub_, pressure_sub_));
			sync_sensor->registerCallback(boost::bind(&HilInterfaceNode::HilSensorPubCallback, this, _1, _2, _3, _4));
		}
		else {
			sync_state.reset(new SyncState(SyncStatePolicy(10), air_speed_sub_, gps_sub_, ground_speed_sub_, imu_sub_, mag_sub_));
			sync_state->registerCallback(boost::bind(&HilInterfaceNode::HilStateQuaternionPubCallback, this, _1, _2, _3, _4, _5));
		}

		ros::spinOnce();
		rate_.sleep();
	}
}

void HilInterfaceNode::HilGPSPubCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg,
		const geometry_msgs::TwistStampedConstPtr &ground_speed_msg) {
	// Handle Ground Speed data.
	Eigen::Vector3f gps_vel(Eigen::Vector3f(ground_speed_msg->twist.linear.x,
				ground_speed_msg->twist.linear.y,
				ground_speed_msg->twist.linear.z));

	auto hil_gps_msg = boost::make_shared<mavros_msgs::HilGPS>();

	// Check if we need to publish a HIL_GPS message.
	if ((ros::Time::now().toNSec() - last_gps_pub_time_nsec_) >= gps_interval_nsec_) {
		last_gps_pub_time_nsec_ = ros::Time::now().toNSec();

		// Fill in a MAVROS HilGPS message.
		hil_gps_msg->header.stamp = ros::Time::now();
		hil_gps_msg->fix_type = (gps_msg->status.status > sensor_msgs::NavSatStatus::STATUS_NO_FIX) ? kFix3D : kFixNone;
		hil_gps_msg->geo.latitude = gps_msg->latitude;
		hil_gps_msg->geo.longitude = gps_msg->longitude;
		hil_gps_msg->geo.altitude = gps_msg->altitude;
		hil_gps_msg->eph = kHDOP;
		hil_gps_msg->epv = kVDOP;
		hil_gps_msg->vel = gps_vel.norm();
		hil_gps_msg->vn = gps_vel.x();
		hil_gps_msg->ve = gps_vel.y();
		hil_gps_msg->vd = gps_vel.z();
		hil_gps_msg->cog = kUnknown;
		hil_gps_msg->satellites_visible = kSatellitesVisible;

		// Publish MAVROS HilGPS message.
		hil_gps_pub_.publish(hil_gps_msg);
	}
}

void HilInterfaceNode::HilSensorPubCallback(const geometry_msgs::TwistStampedConstPtr& air_speed_msg,
		const sensor_msgs::ImuConstPtr& imu_msg,
		const sensor_msgs::MagneticFieldConstPtr &mag_msg,
		const sensor_msgs::FluidPressureConstPtr &pressure_msg) {
	// Handle Air speed data.
	Eigen::Vector3d air_velocity(air_speed_msg->twist.linear.x,
			air_speed_msg->twist.linear.y,
			air_speed_msg->twist.linear.z);

	// Handle IMU data.
	Eigen::Vector3f acc(imu_msg->linear_acceleration.x,
			imu_msg->linear_acceleration.y,
			imu_msg->linear_acceleration.z);

	Eigen::Quaterniond att(imu_msg->orientation.w,
			imu_msg->orientation.x,
			imu_msg->orientation.y,
			imu_msg->orientation.z);

	Eigen::Vector3f gyro(imu_msg->angular_velocity.x,
			imu_msg->angular_velocity.y,
			imu_msg->angular_velocity.z);

	// Handle Mag data
	Eigen::Vector3f mag(mag_msg->magnetic_field.x,
			mag_msg->magnetic_field.y,
			mag_msg->magnetic_field.z);

	// From the following formula: p_stag - p_static = 0.5 * rho * v^2.
	float pressure_diff = 0.5 * kAirDensity_kg_per_m3 * air_velocity.norm() * air_velocity.norm();

	float pressure_alt =
			(1 - pow((pressure_msg->fluid_pressure / kStandardPressure_Pascal), kPressureToAltExp)) *
			kPressureToAltMult * kFeetToMeters;

	auto hil_sensor_msg = boost::make_shared<mavros_msgs::HilSensor>();

	// Fill in a MAVROS HilSensor message.
	hil_sensor_msg->header.stamp = ros::Time::now();
	hil_sensor_msg->acc.x = acc.x();
	hil_sensor_msg->acc.y = acc.y();
	hil_sensor_msg->acc.z = acc.z();
	hil_sensor_msg->gyro.x = gyro.x();
	hil_sensor_msg->gyro.y = gyro.y();
	hil_sensor_msg->gyro.z = gyro.z();
	hil_sensor_msg->mag.x = mag.x();
	hil_sensor_msg->mag.y = mag.y();
	hil_sensor_msg->mag.z = mag.z();
	hil_sensor_msg->abs_pressure = pressure_msg->fluid_pressure;
	hil_sensor_msg->diff_pressure = pressure_diff;
	hil_sensor_msg->pressure_alt = pressure_alt;
	hil_sensor_msg->temperature = kTemperature_C;
	hil_sensor_msg->fields_updated = kAllFieldsUpdated;

	// Publish MAVROS HilSensor message.
	hil_sensor_pub_.publish(hil_sensor_msg);
}

void HilInterfaceNode::HilStateQuaternionPubCallback(const geometry_msgs::TwistStampedConstPtr& air_speed_msg,
		const sensor_msgs::NavSatFixConstPtr& gps_msg,
		const geometry_msgs::TwistStampedConstPtr &ground_speed_msg,
		const sensor_msgs::ImuConstPtr& imu_msg,
		const sensor_msgs::MagneticFieldConstPtr &mag_msg) {
	// Handle Air speed data.
	Eigen::Vector3d air_velocity(air_speed_msg->twist.linear.x,
			air_speed_msg->twist.linear.y,
			air_speed_msg->twist.linear.z);

	// Handle Ground Speed data.
	Eigen::Vector3f gps_vel(Eigen::Vector3f(ground_speed_msg->twist.linear.x,
				ground_speed_msg->twist.linear.y,
				ground_speed_msg->twist.linear.z));

	// Handle IMU data.
	Eigen::Vector3f acc(imu_msg->linear_acceleration.x,
			imu_msg->linear_acceleration.y,
			imu_msg->linear_acceleration.z);

	Eigen::Quaterniond att(imu_msg->orientation.w,
			imu_msg->orientation.x,
			imu_msg->orientation.y,
			imu_msg->orientation.z);

	Eigen::Vector3f gyro(imu_msg->angular_velocity.x,
			imu_msg->angular_velocity.y,
			imu_msg->angular_velocity.z);

	// Handle Mag data
	Eigen::Vector3f mag(mag_msg->magnetic_field.x,
			mag_msg->magnetic_field.y,
			mag_msg->magnetic_field.z);

	auto hil_state_msg = boost::make_shared<mavros_msgs::HilStateQuaternion>();

	// Fill in a MAVROS HilStateQuaternion message.
	hil_state_msg->header.stamp = ros::Time::now();
	hil_state_msg->orientation.w = att.w();
	hil_state_msg->orientation.x = att.x();
	hil_state_msg->orientation.y = att.y();
	hil_state_msg->orientation.z = att.z();
	hil_state_msg->angular_velocity.x = gyro.x();
	hil_state_msg->angular_velocity.y = gyro.y();
	hil_state_msg->angular_velocity.z = gyro.z();
	hil_state_msg->geo.latitude = gps_msg->latitude;
	hil_state_msg->geo.longitude = gps_msg->longitude;
	hil_state_msg->geo.altitude = gps_msg->altitude;
	hil_state_msg->linear_velocity.x = gps_vel.x();
	hil_state_msg->linear_velocity.y = gps_vel.y();
	hil_state_msg->linear_velocity.z = gps_vel.z();
	hil_state_msg->ind_airspeed = air_velocity.norm();
	hil_state_msg->true_airspeed = air_velocity.norm();
	hil_state_msg->linear_acceleration.x = acc.x() * kGravityMagnitude_m_per_s2;
	hil_state_msg->linear_acceleration.y = acc.y() * kGravityMagnitude_m_per_s2;
	hil_state_msg->linear_acceleration.z = acc.z() * kGravityMagnitude_m_per_s2;

	// Publish MAVROS HilStateQuaternion message.
	hil_state_pub_.publish(hil_state_msg);
}

void HilInterfaceNode::HilControlsCallback(const mavros_msgs::HilControlsConstPtr& hil_controls_msg) {
	mav_msgs::Actuators act_msg;

	ros::Time current_time = ros::Time::now();

	act_msg.normalized.push_back(hil_controls_msg->roll_ailerons);
	act_msg.normalized.push_back(hil_controls_msg->pitch_elevator);
	act_msg.normalized.push_back(hil_controls_msg->yaw_rudder);
	act_msg.normalized.push_back(hil_controls_msg->aux1);
	act_msg.normalized.push_back(hil_controls_msg->aux2);
	act_msg.normalized.push_back(hil_controls_msg->throttle);

	act_msg.header.stamp.sec = current_time.sec;
	act_msg.header.stamp.nsec = current_time.nsec;

	actuators_pub_.publish(act_msg);
}
}	// namespace hil_interface

int main(int argc, char** argv) {
	ros::init(argc, argv, "hil_interface_node");
	hil_interface::HilInterfaceNode hil_interface_node;

	hil_interface_node.MainTask();

	return 0;
}
