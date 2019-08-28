/*
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright 2019 Swift Engineering, Inc. <nuno.marques@dronesolutions.io>
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
 */

#include <stdio.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <boost/bind.hpp>

#include <gazebo_ros_interface.h>

namespace gazebo {

GazeboRosInterface::GazeboRosInterface() :
  WorldPlugin()
  { }

GazeboRosInterface::~GazeboRosInterface() {
  updateConnection_.reset();
  // Shutdown and delete ROS node handle
  ros_node_handle_->shutdown();
  ros_node_handle_.reset();
}

void GazeboRosInterface::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
  // Store the pointer to the world
  world_ = _world;

  // Get models on the world
  auto models = world_->Models();

  // Get Gazebo node handle
  gz_node_handle_ = transport::NodePtr(new transport::Node());
  gz_node_handle_->Init();

  // Initialize ros, if it has not already been initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  ros_node_handle_.reset(new ros::NodeHandle("gazebo_client"));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosInterface::OnUpdate, this, _1));

  // Check for each model loaded in the world
  for (size_t i = 0; i < models.size(); i++) {
    // Get each model info
    auto sdf_ptr = models[i]->GetSDF();
    ignition::msgs::Plugin_V plugins;
    bool success;
    common::URI pluginUri;
    pluginUri.Parse("data://world/default/model/" + models[i]->GetName() + "/plugin/");

    models[i]->PluginInfo(pluginUri, plugins, success);

    // If the info is correctly retrieved and there are plugins loaded into each model
    // then it check for each plugin of interest that we want to get the topics from
    if (success && plugins.plugins_size() > 0) {
      for (size_t j = 0; j < plugins.plugins_size(); j++) {
        // Verify if the GPS plugin is loaded
        if (plugins.plugins(j).filename() == "libgazebo_gps_plugin.so") {
          // Create ROS publishers for GPS
          ros::Publisher ros_navsatfix_pub = ros_node_handle_->advertise<sensor_msgs::NavSatFix>("/gazebo/" + models[i]->GetName() + "/gps", 1);
          std::vector<ros::Publisher> gps_ros_publishers = {ros_navsatfix_pub};

          // handle GPS plugin GPS data
          ConnectHelper<sensor_msgs::msgs::SITLGps>(&GazeboRosInterface::GzSITLGpsMsgCallback, this, "",
            "~/" + models[i]->GetName() + "/gps", gps_ros_publishers, gz_node_handle_);
        }
        // Verify if the IMU plugin is loaded
        if (plugins.plugins(j).filename() == "libgazebo_imu_plugin.so") {
          // Create ROS publisher
          ros::Publisher ros_imu_pub = ros_node_handle_->advertise<sensor_msgs::Imu>("/gazebo/" + models[i]->GetName() + "/imu", 1);
          std::vector<ros::Publisher> imu_ros_publishers = {ros_imu_pub};

          // handle IMU plugin data
          ConnectHelper<sensor_msgs::msgs::Imu>(&GazeboRosInterface::GzImuMsgCallback, this, "",
            "~/" + models[i]->GetName() + "/imu", imu_ros_publishers, gz_node_handle_);
        }
        // Verify if the Baro plugin is loaded
        if (plugins.plugins(j).filename() == "libgazebo_barometer_plugin.so") {
          // Create ROS publishers (needs to be an array since the same callback handles multiple publishers)
          ros::Publisher pressure_pub = ros_node_handle_->advertise<sensor_msgs::FluidPressure>("/gazebo/" + models[i]->GetName() + "/baro/abs_pressure", 1);
          ros::Publisher pressure_alt_pub = ros_node_handle_->advertise<geometry_msgs::Vector3Stamped>("/gazebo/" + models[i]->GetName() + "/baro/pressure_alt", 1);
          ros::Publisher temperature_pub = ros_node_handle_->advertise<sensor_msgs::Temperature>("/gazebo/" + models[i]->GetName() + "/baro/temperature", 1);
          std::vector<ros::Publisher> baro_ros_publishers = {pressure_pub, pressure_alt_pub, temperature_pub};

          // handle Baro plugin data
          ConnectHelper<sensor_msgs::msgs::Pressure>(&GazeboRosInterface::GzBaroMsgCallback, this, "",
            "~/" + models[i]->GetName() + "/baro", baro_ros_publishers, gz_node_handle_);
        }
      }
    }

    // Get model joints in order to check for sensors
    // Probably not the best way but the one that does work for now
    auto joints = models[i]->GetJoints();
    std::vector<std::string> joint_names;
    for(size_t y = 0; y < joints.size(); y ++) {
      joint_names.push_back(joints[y]->GetName());
    }

    // Verify if the IRLock sensor joint exists
    if (std::find(joint_names.begin(), joint_names.end(), "irlock_joint") != joint_names.end()) {
      // Create ROS publisher
      ros::Publisher irlock_pub = ros_node_handle_->advertise<mavlink_sitl_gazebo::IRLock>("/gazebo/" + models[i]->GetName() + "/irlock/data", 1);
      std::vector<ros::Publisher> irlock_ros_publishers = {irlock_pub};

      // handle Diff pressure plugin data
      ConnectHelper<sensor_msgs::msgs::IRLock>(&GazeboRosInterface::GzIRLockMsgCallback, this, "",
        "~/" + models[i]->GetName() + "/camera/link/irlock", irlock_ros_publishers, gz_node_handle_);
    }
    // Verify if the Lidar (or model specific) sensor joint exists
    if (std::find(joint_names.begin(), joint_names.end(), "lidar_joint") != joint_names.end() ||
        std::find(joint_names.begin(), joint_names.end(), "sf10a_joint") != joint_names.end()) {
      // Create ROS publisher
      ros::Publisher lidar_pub = ros_node_handle_->advertise<sensor_msgs::Range>("/gazebo/" + models[i]->GetName() + "/lidar/data", 1);
      std::vector<ros::Publisher> lidar_ros_publishers = {lidar_pub};

      // handle Diff pressure plugin data
      ConnectHelper<sensor_msgs::msgs::Range>(&GazeboRosInterface::GzLidarMsgCallback, this, "",
        "~/" + models[i]->GetName() + "lidar/link/laser/scan", lidar_ros_publishers, gz_node_handle_);
    }
    // Verify if the Sonar (or model specific) sensor joint exists
    if (std::find(joint_names.begin(), joint_names.end(), "sonar_joint") != joint_names.end() ||
        std::find(joint_names.begin(), joint_names.end(), "mb1240-xl-ez4_joint") != joint_names.end()) {
      // Create ROS publisher
      ros::Publisher sonar_pub = ros_node_handle_->advertise<sensor_msgs::Range>("/gazebo/" + models[i]->GetName() + "/sonar/data", 1);
      std::vector<ros::Publisher> sonar_ros_publishers = {sonar_pub};

      // handle Diff pressure plugin data
      ConnectHelper<sensor_msgs::msgs::Range>(&GazeboRosInterface::GzSonarMsgCallback, this, "",
        "~/" + models[i]->GetName() + "/sonar_model/link/sonar", sonar_ros_publishers, gz_node_handle_);
    }

  }
}

void GazeboRosInterface::OnUpdate(const common::UpdateInfo& _info) {
  // Do nothing, as the actions are done through callbacks.
}

/// \brief      A helper class that provides storage for additional parameters that are inserted into the callback.
/// \details    GazeboMsgT  The type of the message that will be subscribed to the Gazebo framework.
template <typename GazeboMsgT>
struct ConnectHelperStorage {
  /// \brief    Pointer to the ROS interface plugin class.
  GazeboRosInterface* ptr;

  /// \brief    Function pointer to the subscriber callback with additional parameters.
  void (GazeboRosInterface::*fp)(const boost::shared_ptr<GazeboMsgT const>&, std::vector<ros::Publisher>);

  /// \brief    The vector of ROS publishers that is passed into the modified callback.
  std::vector<ros::Publisher> ros_publishers;

  /// \brief    This is what gets passed into the Gazebo Subscribe method as a callback,
  ///           and hence can onlyhave one parameter (note boost::bind() does not work with the
  ///           current Gazebo Subscribe() definitions).
  void callback(const boost::shared_ptr<GazeboMsgT const>& msg_ptr) {
    (ptr->*fp)(msg_ptr, ros_publishers);
  }
};

template <typename GazeboMsgT>
void GazeboRosInterface::ConnectHelper(
    void (GazeboRosInterface::*fp)(const boost::shared_ptr<GazeboMsgT const>&, std::vector<ros::Publisher>),
    GazeboRosInterface* ptr, std::string gazeboNamespace,
    std::string gazeboTopicName, std::vector<ros::Publisher> ros_publishers,
    transport::NodePtr gz_node_handle) {
  // One map will be created for each Gazebo message type
  static std::map<std::string, ConnectHelperStorage<GazeboMsgT> > callback_map;

  // Store the callback entries
  auto callback_entry = callback_map.emplace(
      gazeboTopicName,
      ConnectHelperStorage<GazeboMsgT>{ptr, fp, ros_publishers});

  // Check if element was already present
  if (!callback_entry.second)
    gzerr << "Tried to add element to map but the gazebo topic name was "
             "already present in map."
          << std::endl;

  // Create subscriber
  gazebo::transport::SubscriberPtr subscriberPtr;
  subscriberPtr = gz_node_handle->Subscribe(
      gazeboTopicName, &ConnectHelperStorage<GazeboMsgT>::callback,
      &callback_entry.first->second);

  // Save a reference to the subscriber pointer so subscriber won't be deleted.
  subscriberPtrs_.push_back(subscriberPtr);
}

void GazeboRosInterface::GzImuMsgCallback(GzImuMsgPtr& gz_imu_msg, std::vector<ros::Publisher> ros_publishers) {
  auto imu_msg = boost::make_shared<sensor_msgs::Imu>();

#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif

  imu_msg->header.frame_id = "gz_base_link";
  imu_msg->header.stamp.sec = current_time.sec;
  imu_msg->header.stamp.nsec = current_time.nsec;

  imu_msg->orientation.x = gz_imu_msg->orientation().x();
  imu_msg->orientation.y = gz_imu_msg->orientation().y();
  imu_msg->orientation.z = gz_imu_msg->orientation().z();
  imu_msg->orientation.w = gz_imu_msg->orientation().w();

  // Orientation covariance should have 9 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  GZ_ASSERT(gz_imu_msg->orientation_covariance_size() == 9,
            "The Gazebo IMU message does not have 9 orientation covariance "
            "elements.");
  GZ_ASSERT(
      imu_msg->orientation_covariance.size() == 9,
      "The ROS IMU message does not have 9 orientation covariance elements.");
  for (int i = 0; i < gz_imu_msg->orientation_covariance_size(); i++) {
    imu_msg->orientation_covariance[i] =
        gz_imu_msg->orientation_covariance(i);
  }

  imu_msg->angular_velocity.x = gz_imu_msg->angular_velocity().x();
  imu_msg->angular_velocity.y = gz_imu_msg->angular_velocity().y();
  imu_msg->angular_velocity.z = gz_imu_msg->angular_velocity().z();

  GZ_ASSERT(gz_imu_msg->angular_velocity_covariance_size() == 9,
            "The Gazebo IMU message does not have 9 angular velocity "
            "covariance elements.");
  GZ_ASSERT(imu_msg->angular_velocity_covariance.size() == 9,
            "The ROS IMU message does not have 9 angular velocity covariance "
            "elements.");
  for (int i = 0; i < gz_imu_msg->angular_velocity_covariance_size(); i++) {
    imu_msg->angular_velocity_covariance[i] =
        gz_imu_msg->angular_velocity_covariance(i);
  }

  imu_msg->linear_acceleration.x = gz_imu_msg->linear_acceleration().x();
  imu_msg->linear_acceleration.y = gz_imu_msg->linear_acceleration().y();
  imu_msg->linear_acceleration.z = gz_imu_msg->linear_acceleration().z();

  GZ_ASSERT(gz_imu_msg->linear_acceleration_covariance_size() == 9,
            "The Gazebo IMU message does not have 9 linear acceleration "
            "covariance elements.");
  GZ_ASSERT(imu_msg->linear_acceleration_covariance.size() == 9,
            "The ROS IMU message does not have 9 linear acceleration "
            "covariance elements.");
  for (int i = 0; i < gz_imu_msg->linear_acceleration_covariance_size(); i++) {
    imu_msg->linear_acceleration_covariance[i] =
        gz_imu_msg->linear_acceleration_covariance(i);
  }

  // Publish to ROS
  ros_publishers[0].publish(imu_msg);
}

void GazeboRosInterface::GzSITLGpsMsgCallback(GzSITLGpsMsgPtr& gz_sitl_gps_msg, std::vector<ros::Publisher> ros_publishers) {
  auto fix_msg = boost::make_shared<sensor_msgs::NavSatFix>();

  fix_msg->header.frame_id = "map";
  fix_msg->header.stamp = ros::Time(gz_sitl_gps_msg->time_usec() * 1000UL / 1000000000UL, gz_sitl_gps_msg->time_usec() * 1000UL % 1000000000UL);

  fix_msg->latitude = gz_sitl_gps_msg->latitude_deg();
  fix_msg->longitude = gz_sitl_gps_msg->longitude_deg();
  fix_msg->altitude = gz_sitl_gps_msg->altitude();

  fix_msg->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  fix_msg->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

  fix_msg->position_covariance[0] = fix_msg->position_covariance[4] = gz_sitl_gps_msg->eph() * gz_sitl_gps_msg->eph();
  fix_msg->position_covariance[8] = gz_sitl_gps_msg->epv() * gz_sitl_gps_msg->epv();
  fix_msg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

  // Publish to ROS
  ros_publishers[0].publish(fix_msg);
}

void GazeboRosInterface::GzBaroMsgCallback(GzBaroMsgPtr& gz_baro_msg, std::vector<ros::Publisher> ros_publishers) {
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif

  auto header = boost::make_shared<std_msgs::Header>();
  header->frame_id = "base_link";
  header->stamp.sec = current_time.sec;
  header->stamp.nsec = current_time.nsec;

  // Pressure data
  auto pressure_msg = boost::make_shared<sensor_msgs::FluidPressure>();
  pressure_msg->header = *header;
  pressure_msg->fluid_pressure = gz_baro_msg->absolute_pressure();       // Pascal
  pressure_msg->variance = 0; // unknown?
  ros_publishers[0].publish(pressure_msg);

  // Altitude from pressure
  auto pressure_alt_msg = boost::make_shared<geometry_msgs::Vector3Stamped>();
  pressure_alt_msg->header = *header;
  pressure_alt_msg->vector.z = gz_baro_msg->pressure_altitude();     // meters
  ros_publishers[1].publish(pressure_alt_msg);

  // Temperature
  auto temperature_msg = boost::make_shared<sensor_msgs::Temperature>();
  temperature_msg->header = *header;
  temperature_msg->temperature = gz_baro_msg->temperature() - 273.15;    // convert degrees Kelvin to degrees Celsius
  temperature_msg->variance = 0; // unknown?
  ros_publishers[2].publish(temperature_msg);
}

void GazeboRosInterface::GzIRLockMsgCallback(GzIRLockMsgPtr& gz_irlock_msg, std::vector<ros::Publisher> ros_publishers) {
  auto irlock_msg = boost::make_shared<mavlink_sitl_gazebo::IRLock>();

  irlock_msg->header.frame_id = "base_link";
  irlock_msg->header.stamp = ros::Time(gz_irlock_msg->time_usec() * 1000UL / 1000000000UL, gz_irlock_msg->time_usec() * 1000UL % 1000000000UL);
  irlock_msg->pos_x = gz_irlock_msg->pos_x();   // meters
  irlock_msg->pos_y = gz_irlock_msg->pos_y();   // meters
  irlock_msg->size_x = gz_irlock_msg->size_x();   // radians
  irlock_msg->size_y = gz_irlock_msg->size_y();   // radians

  ros_publishers[0].publish(irlock_msg);
}

void GazeboRosInterface::GzLidarMsgCallback(GzRangeMsgPtr& gz_lidar_msg, std::vector<ros::Publisher> ros_publishers) {
  auto lidar_msg = boost::make_shared<sensor_msgs::Range>();

  lidar_msg->header.frame_id = "lidar_link";
  lidar_msg->header.stamp = ros::Time(gz_lidar_msg->time_usec() * 1000UL / 1000000000UL, gz_lidar_msg->time_usec() * 1000UL % 1000000000UL);
  lidar_msg->radiation_type = sensor_msgs::Range::INFRARED;
  lidar_msg->field_of_view = 0.05235987756;   // rad [default: 3 degrees]
  lidar_msg->min_range = gz_lidar_msg->min_distance();   // meters
  lidar_msg->max_range = gz_lidar_msg->max_distance();   // meters
  lidar_msg->range = gz_lidar_msg->current_distance();   // meters

  ros_publishers[0].publish(lidar_msg);
}

void GazeboRosInterface::GzSonarMsgCallback(GzRangeMsgPtr& gz_sonar_msg, std::vector<ros::Publisher> ros_publishers) {
  auto sonar_msg = boost::make_shared<sensor_msgs::Range>();

  sonar_msg->header.frame_id = "sonar_link";
  sonar_msg->header.stamp = ros::Time(gz_sonar_msg->time_usec() * 1000UL / 1000000000UL, gz_sonar_msg->time_usec() * 1000UL % 1000000000UL);
  sonar_msg->radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_msg->field_of_view = 0.5235987756;   // rad [default: 30 degrees]
  sonar_msg->min_range = gz_sonar_msg->min_distance();   // meters
  sonar_msg->max_range = gz_sonar_msg->max_distance();   // meters
  sonar_msg->range = gz_sonar_msg->current_distance();   // meters

  ros_publishers[0].publish(sonar_msg);
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosInterface);

}  // namespace gazebo
