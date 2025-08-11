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

  if (this->sdf->HasElement("scale")) {
    try { this->forceScale = this->sdf->Get<double>("scale"); }
    catch (...) { this->forceScale = 1.0; }
  }
  // Read link_name (used to attach the line to a specified link's Visual)
  if (this->sdf->HasElement("link_name")) {
    this->frameLinkName = this->sdf->Get<std::string>("link_name");
  }


  this->host.reset();
  {
    auto scene = this->visual->GetScene();
    std::string visName = this->visual->Name();
    std::string modelName;
    auto pos = visName.find("::");
    if (pos != std::string::npos) {
      modelName = visName.substr(0, pos);
    }

    if (!this->frameLinkName.empty()) {
      if (!modelName.empty()) {
        auto v = scene->GetVisual(modelName + std::string("::") + this->frameLinkName);
        if (v) this->host = v;
      }
      if (!this->host) {
        auto v = scene->GetVisual(this->frameLinkName);
        if (v) this->host = v;
      }
    }

    if (!this->host) {
      this->host = this->visual;
    }
  }

  auto forceVector = this->host->CreateDynamicLine(rendering::RENDERING_LINE_LIST);

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

  // The force is applied to the cp point of "link_name", and is not affected by the current visual tag.
  // Visualize scaling of force length (<scale>)
  force *= this->forceScale;

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
