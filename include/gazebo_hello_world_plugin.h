
#include <stdio.h>

#include "CommandMotorSpeed.pb.h"
#include "Float.pb.h"
#include "MotorSpeed.pb.h"
#include "Wind.pb.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include <Eigen/Eigen>
#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <rotors_model/motor_model.hpp>

#include "common.h"
namespace gazebo {

class HelloWorldPlugin : public ModelPlugin {
public:
  HelloWorldPlugin();
  virtual ~HelloWorldPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo &_info);

private:
  // std::string joint_name_;
  // std::string link_name_;
  std::string namespace_;

  transport::NodePtr node_handle_;

  physics::ModelPtr model_;
  // physics::JointPtr joint_;
  // physics::LinkPtr link_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;
};
} // namespace gazebo
