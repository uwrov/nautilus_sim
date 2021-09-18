#ifndef THRUSTER_H
#define THRUSTER_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>

#define PWM_ZERO 1500
#define PWM_MAX 2000
#define PWM_MIN 1000

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
               bool t100);
      
      /**
       * Simulate applying pwm onto thruster
       * Applies a force onto the body of nautilus, not the thurster itself
       */
      void addLinkForce();

      /**
       *  Set the pwm of the thruster 
       */
      void setPWM(int pwm);

    private:
      int pwm_;

      // Taken from blue robotics for for/rev max thrust,
      // converted to N from kg f
      // not a pp directive because these are computed
      const double t100_for_max_ = 2.36 * 9.8;
      const double t100_rev_max_ = -1.85 * 9.8;
      const double t200_for_max_ = 5.25 * 9.8;
      const double t200_rev_max_ = -4.1 * 9.8;

      double forwardMax;
      double reverseMax;
      double slopeForward;
      double slopeReverse;

      const std::string name_;
      bool t100_;
      physics::LinkPtr link_ptr_;
      physics::LinkPtr body_ptr_;

      double pwmToForce(int pwm);
  };
}

#endif // THRUSTER_H