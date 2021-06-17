#include "force_visual/force_visual.h"

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(ForceVisualPlugin)

ForceVisualPlugin::ForceVisualPlugin() {}

ForceVisualPlugin::~ForceVisualPlugin() {}

void ForceVisualPlugin::Load(rendering::VisualPtr _visual,
                             sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_visual, "ForceVisualPlugin _visual pointer is NULL");
  GZ_ASSERT(_sdf, "ForceVisualPlugin _sdf pointer is NULL");

  this->visual = _visual;
  this->sdf = _sdf;

  gzdbg << "Loading ForceVisual plugin" << std::endl;
}

void ForceVisualPlugin::Init()
{
  this->node.reset(new gazebo::transport::Node());
  this->node->Init();
  this->subs.reset();

  if (this->sdf->HasElement("topic_name")) {
      const auto lift_force_topic = this->sdf->Get<std::string>("topic_name");
      this->subs = this->node->Subscribe("~/" + lift_force_topic, &ForceVisualPlugin::OnUpdate, this);
      gzdbg << "Subscribing on ~/" << lift_force_topic << std::endl;
  }

  auto forceVector = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  this->forceVector.reset(forceVector);
  this->forceVector->setVisibilityFlags(GZ_VISIBILITY_GUI);

  if (this->sdf->HasElement("color")) {
      this->forceVector->setMaterial(this->sdf->Get<std::string>("color"));
  } else {
      this->forceVector->setMaterial("Gazebo/Blue");
  }

  if (this->sdf->HasElement("material")) {
    this->forceVector->setMaterial(this->sdf->Get<std::string>("material"));
  }

  for(int k = 0; k < 6; ++k) { // -> needs three lines, so 6 points
    this->forceVector->AddPoint(ignition::math::Vector3d::Zero);
  }

  this->forceVector->Update();
}

void ForceVisualPlugin::OnUpdate(ConstForcePtr& force_msg)
{
  ignition::math::Vector3d force;
  auto force_vector_msg = force_msg->force();
  force.Set(force_vector_msg.x(), force_vector_msg.y(), force_vector_msg.z());

  ignition::math::Vector3d center;
  auto force_center_msg = force_msg->center();
  center.Set(force_center_msg.x(), force_center_msg.y(), force_center_msg.z());

  // Note that if the visual is scaled, the center needs to be scaled, too,
  // such that the force appears on the visual surface control and not on
  // the actual joint (which will look weird).
  // I am not sure what happens if both the visual and the joint are scaled,
  // but I don't see a reason for doing that, like, ever.
  const auto scale = this->visual->Scale();
  center = center / scale;

  this->UpdateVector(center, force);
}

void ForceVisualPlugin::UpdateVector(const ignition::math::Vector3d& center, const ignition::math::Vector3d& force)
{
  const float arrow_scale = 0.1;

  ignition::math::Vector3d begin = center;
  ignition::math::Vector3d end = center + force;

  this->forceVector->SetPoint(0, begin);
  this->forceVector->SetPoint(1, end);
  this->forceVector->SetPoint(2, end);
  this->forceVector->SetPoint(3, end - arrow_scale * ignition::math::Matrix3d(1, 0, 0, 0, 0.9848, -0.1736, 0,  0.1736, 0.9848) * (end - begin));
  this->forceVector->SetPoint(4, end);
  this->forceVector->SetPoint(5, end - arrow_scale * ignition::math::Matrix3d(1, 0, 0, 0, 0.9848,  0.1736, 0, -0.1736, 0.9848) * (end - begin));
}
