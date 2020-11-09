#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
	/// \brief An example plugin for a contact sensor.
	class ContactPlugin : public SensorPlugin
	{
		/// \brief Contructor.
		public: ContactPlugin();
		
		/// \brief Destructor.
		public: virtual ~ContactPlugin();
		
		/// \brief Load the sensor plugin.
		/// \param[in] _sensor Point to the sensor that loaded this plugin.
		/// \param[in] _sdf SDF element that describes the plugin.
		public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
		
		/// \brief Callback that receives the contact sensor's update signal.
		private: virtual void OnUpdate();
		
		/// \brief Point to the contact sensor
		private: sensors::ContactSensorPtr parentSensor;
		
		/// \brief Conntection that maintains a link between the contact sensor's 
		/// updated signal and the OnUpdate callback.
		private: event::ConnectionPtr updateConnection;
	};
}
#endif
