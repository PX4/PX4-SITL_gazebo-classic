#include "ServoPlugin.hh"

using namespace gazebo;

// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(ServoPlugin)

namespace gazebo {
struct ServoPluginPrivate {
public:
  physics::ModelPtr rsModel;
  physics::WorldPtr world;
  transport::NodePtr transportNode;
  event::ConnectionPtr updateConnection;
};
} // namespace gazebo

/////////////////////////////////////////////////
ServoPlugin::ServoPlugin() : dataPtr(new ServoPluginPrivate) {}

/////////////////////////////////////////////////
ServoPlugin::~ServoPlugin() { delete joint_controller_; }

void ServoPlugin::set_angle(const std_msgs::msg::Float64::SharedPtr msg) {
  if (td_ == false) {
    angle_ = msg->data * M_PI / 180.0;
  } else {
    angle_ = -msg->data * M_PI / 180.0;
  }
}

void ServoPlugin::set_servo(
    const std::shared_ptr<raptor_interface::srv::SetServo::Request> request,
    std::shared_ptr<raptor_interface::srv::SetServo::Response> response) {
  if (td_ == false) {
    angle_ = request->angle * M_PI / 180.0;
  } else {
    angle_ = -request->angle * M_PI / 180.0;
  }
  response->success = true;
}
/////////////////////////////////////////////////

// TODO add parameters: initial position
void ServoPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store a pointer to the this model
  model_ = _model;

  // Store a pointer to the this model
  this->dataPtr->rsModel = _model;

  // Store a pointer to the world
  this->dataPtr->world = this->dataPtr->rsModel->GetWorld();

  // Read in parameters
  if (_sdf->HasElement("subTopic")) {
    subTopic_ = _sdf->GetElement("subTopic")->Get<std::string>();
  } else {
    std::cout << "[Gazebo Servo] Please specify a SubTopic for the servo."
              << std::endl;
  }

  if (_sdf->HasElement("turningDirection")) {
    td_ = _sdf->GetElement("turningDirection")->Get<bool>();
  } else {
    std::cout
        << "[Gazebo Servo] Please specify a turningDirection for the servo."
        << std::endl;
  }

  if (_sdf->HasElement("initialAngle")) {
    if (td_ == false) {
      angle_ = _sdf->GetElement("initialAngle")->Get<double>() * M_PI / 180.0;
    } else {
      angle_ = -_sdf->GetElement("initialAngle")->Get<double>() * M_PI / 180.0;
    }

  } else {
    std::cout << "[Gazebo Servo] Please specify an initialAngle for the servo."
              << std::endl;
  }

  if (_sdf->HasElement("jointName")) {
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  } else {
    std::cout
        << "[gazebo_motor_model] Please specify a jointName, where the servo "
           "is attached.\n";
  }

  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL) {
    gzthrow("[gazebo_motor_model] Couldn't find specified joint \""
            << joint_name_ << "\".");
  }

  if (_sdf->HasElement("linkName")) {
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  } else {
    std::cout
        << "[gazebo_motor_model] Please specify a linkName of the rotor.\n";
  }

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL) {
    gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_
                                                                   << "\".");
  }

  // initialize joint controller
  joint_controller_ = new physics::JointController(model_);
  joint_controller_->AddJoint(this->joint_);

  // ROS 1 stuff
  //  Setup Transport Node
  this->dataPtr->transportNode = transport::NodePtr(new transport::Node());

  this->dataPtr->transportNode->Init(this->dataPtr->world->Name());
  //
  this->dataPtr->transportNode->Init(this->dataPtr->rsModel->GetName());

  // Setup Publishers
  std::string rsTopicRoot =
      "~/" + this->dataPtr->rsModel->GetName() + "/rs/stream/";

  // Listen to the update event
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ServoPlugin::OnUpdate, this));

  // ROS2 stuff
  // Initialize ROS2, if it has not already been initialized.
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = NULL;
    rclcpp::init(argc, argv);
  }

  // create ROS2 publisher node
  this->ros_node_ = rclcpp::Node::make_shared(
      subTopic_ + "_node"); // TODO change string to something esle

  // publisher_ =
  //     this->ros_node_->create_publisher<std_msgs::msg::String>(pubTopic_,
  //     10);

  subscription_ = this->ros_node_->create_subscription<std_msgs::msg::Float64>(
      subTopic_, 10, std::bind(&ServoPlugin::set_angle, this, std::placeholders::_1));

  service_ = this->ros_node_->create_service<raptor_interface::srv::SetServo>(
      subTopic_, std::bind(&ServoPlugin::set_servo, this, std::placeholders::_1,
                           std::placeholders::_2));
  // Spin ROS2 node
  // rclcpp::spin(this->ros_node_);

  std::cout << "servo plugin successfully initialized" << std::endl;
}

void ServoPlugin::OnUpdate() {

  // ignition::math::Vector3d angle_error(this->joint_->Orientation().Euler());
  // std::cout << angle_error.at(0) << "\t" << angle_error.at(1) << "\t"
  //           << angle_error.at(2) << std::endl;
  // // set force
  // link_->AddRelativeTorque(ignition::math::Vector3d(0, 0, 0.00001));
  // this->joint_->setAngle(0, angle_);
  // gazebo::physics::JointController::SetJointPosition(this->joint_, 1.3);

  // std::cout << "updating" << std::endl;
  rclcpp::spin_some(this->ros_node_);

  joint_controller_->SetJointPosition(this->joint_, angle_);
  joint_controller_->Update();
  // timestep

  // sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  // prev_sim_time_ = _info.simTime.Double();

  // ROS2 publisher
  // std_msgs::msg::String msg;
  // msg.data = "this is a test";
  // publisher_->publish(msg);
}
