#ifndef THRUSTER_H
#define THRUSTER_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>

#define PWM_ZERO 1500
#define PWM_DEAD_ZONE 25
#define PWM_MAX 1900
#define PWM_MIN 1100

#define PWM_MIN_START (PWM_ZERO - PWM_DEAD_ZONE)
#define PWM_MAX_START (PWM_ZERO + PWM_DEAD_ZONE)

namespace gazebo
{
  /**
   * Thruster abstraction class, used by the ThrusterManager class.
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

      // https://web.archive.org/web/20171019111512/http://bluerobotics.com/store/thrusters/t100-thruster/
      const double t100_for_max_ = 2.36 * 9.8;
      const double t100_rev_max_ = -1.85 * 9.8;

      // https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/
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