#include "thruster.h"

namespace gazebo {

Thruster::Thruster(const std::string name, physics::ModelPtr parent,
                   bool t100) :
  name_(name),
  t100_(t100),
  link_ptr_(parent->GetLink(name)),
  body_ptr_(parent->GetLink("body")) {
    this->pwm_ = 1500;
    this->forwardMax = t100_ ? this->t100_for_max_ : this->t200_for_max_;
    this->reverseMax = t100_ ? this->t100_rev_max_ : this->t200_rev_max_;
    this->slopeForward = forwardMax / (PWM_MAX - PWM_ZERO);
    this->slopeReverse = -1.0 * reverseMax / (PWM_ZERO - PWM_MIN);
  }

void Thruster::addLinkForce() {
  // convert pwm into a force, then apply that force onto the link
  double f = this->pwmToForce(this->pwm_);

  // thrusters are simulated with cylinders, force
  // is applied on the "bottom" (when upright neutral) normal face
  ignition::math::Vector3d force = ignition::math::Vector3d(0, 0, f);

  this->link_ptr_->AddLinkForce(force);
}

void Thruster::setPWM(int pwm) {
  this->pwm_ = pwm;
}

double Thruster::pwmToForce(int pwm) {
  double ret = 0.0;
  
  // TODO: make scaling more accurate to life (check datashet), not linear
  if (pwm < PWM_ZERO) {
    // map (PWM_MIN, PWM_ZERO) onto (reverseMax, 0)
    ret = this->reverseMax + this->slopeReverse * (pwm - PWM_MIN);
  }
  else {
    // map (PWM_ZERO, PWM_MAX) onto (0, forwardMax)
    ret = this->slopeForward * (pwm - PWM_ZERO);
  }

  return ret;
}

}