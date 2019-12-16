/**
 * @brief Model reset plugin
 *
 * This plugin reset the simulated model into their home position
 *
 * @author Omar Shrit <shrit@lri.fr>
 */

#ifndef _GAZEBO_RESET_PLUGIN_HH_
#define _GAZEBO_RESET_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class ResetPlugin : public ModelPlugin
  {
  public:
    ResetPlugin();

    void Load(physics::ModelPtr _parent, sdf::ElementPtr);
    void OnMsg(ConstVector2dPtr &_msg);

  private:
    physics::ModelPtr model;
    transport::NodePtr node;
    transport::SubscriberPtr sub;

  };
}
#endif
