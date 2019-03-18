#include "gazebo/physics/physics.hh"
#include "gazebo_collision_plugin.h"

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <boost/algorithm/string.hpp>

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CollisionPlugin)

/////////////////////////////////////////////////
CollisionPlugin::CollisionPlugin()
{
}

/////////////////////////////////////////////////
CollisionPlugin::~CollisionPlugin()
{
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void CollisionPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{

//  Get then name of the parent sensor
  this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_parent);

  if (!this->parentSensor)
    gzthrow("CollisionPlugin requires a ContactSensor");

  this->world = physics::get_world(this->parentSensor->WorldName());
  // this->parentSensor->SetActive(false);
  this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&CollisionPlugin::OnUpdate, this));
  this->parentSensor->SetActive(true);

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzwarn << "[gazebo_collision_plugin] Please specify a robotNamespace.\n";

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  const string scopedName = _parent->ParentName();
  string topicName = "~/" + scopedName + "/contacts";
  boost::replace_all(topicName, "::", "/");

  std::cout << topicName << "\n";

  collision_pub_ = node_handle_->Advertise<contacts_msgs::msgs::Contacts>(topicName, 10);
}

void CollisionPlugin::OnUpdate()
{
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world->SimTime();
#else
  common::Time now = world->GetSimTime();
#endif

  msgs::Contacts contacts;
  contact_message.set_time_usec(now.Double() * 1e6);
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    contact_message.set_collision1(contacts.contact(i).collision1());
    contact_message.set_collision2(contacts.contact(i).collision2());

    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      gazebo::msgs::Vector3d* pos = new gazebo::msgs::Vector3d();
      pos->set_x(contacts.contact(i).position(j).x());
      pos->set_y(contacts.contact(i).position(j).y());
      pos->set_z(contacts.contact(i).position(j).z());
      contact_message.set_allocated_position(pos);

      gazebo::msgs::Vector3d* normal = new gazebo::msgs::Vector3d();
      normal->set_x(contacts.contact(i).normal(j).x());
      normal->set_y(contacts.contact(i).normal(j).y());
      normal->set_z(contacts.contact(i).normal(j).z());
      contact_message.set_allocated_normal(normal);

      contact_message.set_depth(contacts.contact(i).depth(j));
      collision_pub_->Publish(contact_message);
    }
  }

}
