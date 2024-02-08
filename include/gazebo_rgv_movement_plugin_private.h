#ifndef _GAZEBO_RGVMOVEMENT_PLUGIN_PRIVATE_HH_
#define _GAZEBO_RGVMOVEMENT_PLUGIN_PRIVATE_HH_

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include <gazebo/physics/Link.hh>
#include <gazebo/common/Time.hh>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the RandomVelocityPlugin.
  class RgvMovementPluginPrivate
  {
    public: RgvMovementPluginPrivate()
            : velocityFactor(1.0),
              updatePeriod(10, 0),
              xRange(-IGN_DBL_MAX, IGN_DBL_MAX),
              yRange(-IGN_DBL_MAX, IGN_DBL_MAX),
              zRange(-IGN_DBL_MAX, IGN_DBL_MAX)
            {
            }

    /// \brief Velocity scaling factor.
    public: double velocityFactor;

    public: common::Time startTime;
    public: common::Time moveDuration;
    public: common::Time stopDuration;
    public: common::Time stopTime;


    /// \brief Time between recomputing a new velocity vector
    public: common::Time updatePeriod;

    /// \brief Time the of the last update.
    public: common::Time prevUpdate;

    /// \brief Velocity to apply.
    public: ignition::math::Vector3d velocity;

    /// \brief Connects to world update event.
    public: event::ConnectionPtr updateConnection;

    /// \brief X velocity clamping values
    public: ignition::math::Vector2d xRange;

    /// \brief Y velocity clamping values
    public: ignition::math::Vector2d yRange;

    /// \brief Z velocity clamping values
    public: ignition::math::Vector2d zRange;

    /// \brief Pointer to the link that will receive the velocity.
    public: physics::LinkPtr link;
  };
}
#endif
