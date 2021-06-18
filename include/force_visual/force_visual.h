#ifndef _GAZEBO_FORCE_VISUAL_PLUGIN_HH_
#define _GAZEBO_FORCE_VISUAL_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/TransportTypes.hh>

#include "Force.pb.h"

namespace gazebo
{
  typedef const boost::shared_ptr<const physics_msgs::msgs::Force> ConstForcePtr;

  /// \brief A visual plugin that draws a force published on a topic
  class GAZEBO_VISIBLE ForceVisualPlugin : public VisualPlugin
  {
    /// \brief Constructor.
    public: ForceVisualPlugin();

    /// \brief Destructor.
    public: ~ForceVisualPlugin();

    // Documentation Inherited.
    public: virtual void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Pointer to visual containing plugin.
    protected: rendering::VisualPtr visual;

    /// \brief Pointer to element containing sdf.
    protected: sdf::ElementPtr sdf;

    private: void OnUpdate(ConstForcePtr& force_msg);
    private: void UpdateVector(const ignition::math::Vector3d& center, const ignition::math::Vector3d& force);

    private: transport::SubscriberPtr subs;
    private: transport::NodePtr node;
    private: rendering::DynamicLinesPtr forceVector;
  };
}

#endif
