#ifndef _GAZEBO_COLLISION_PLUGIN_HH_
#define _GAZEBO_COLLISION_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/ContactSensor.hh"
#include "gazebo/util/system.hh"

#include "Contacts.pb.h"

namespace gazebo
{
  /// \brief A Ray Sensor Plugin
  class GAZEBO_VISIBLE CollisionPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: CollisionPlugin();

    /// \brief Destructor
    public: virtual ~CollisionPlugin();

    /// \brief Update callback
    public: virtual void OnUpdate();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Pointer to parent
    protected: physics::WorldPtr world;

    /// \brief The parent sensor
    private:
      sensors::ContactSensorPtr parentSensor;
      transport::NodePtr node_handle_;
      transport::PublisherPtr collision_pub_;
      std::string namespace_;


    /// \brief The connection tied to RayPlugin::OnNewLaserScans()
    private:
      event::ConnectionPtr updateConnection;

      contacts_msgs::msgs::Contacts contact_message;
  };
}
#endif
