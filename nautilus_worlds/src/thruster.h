#ifndef THRUSTER_H
#define THRUSTER_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>

namespace gazebo
{
  /**
   * Thruster abstraction class
   */
  class Thruster {
    public:
      /**
       * Constructor
       * @param name Name of thruster
       * @param parent Parent model
       * @param max_force Link to thruster
       */
      Thruster(const std::string name, physics::ModelPtr parent,
               double max_force, bool t100);
      
      /**
       * Simulate applying pwm onto thruster
       * Applies a force onto the body of nautilus, not the thurster itself
       */
      void addLinkForce(int pwm);
    private:
      double current_force_;
      const double max_force_;
      const std::string name_;
      bool t100_;
      physics::LinkPtr link_ptr_;
      physics::LinkPtr body_ptr_;
  }
}

#endif // THRUSTER_H