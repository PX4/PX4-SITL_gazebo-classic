#include <gazebo_hello_world_plugin.h>

namespace gazebo {

HelloWorldPlugin::HelloWorldPlugin() : ModelPlugin() {}

HelloWorldPlugin::~HelloWorldPlugin() { updateConnection_->~Connection(); }

void HelloWorldPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  namespace_.clear();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&HelloWorldPlugin::OnUpdate, this, boost::placeholders::_1));
}

void HelloWorldPlugin::OnUpdate(const common::UpdateInfo &_info) {
  std::cout << "Hello World" << std::endl;
}

} // namespace gazebo
