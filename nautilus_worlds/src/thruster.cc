#include "thruster.h"

namespace gazebo {

Thruster::Thruster(const std::string name, physics::ModelPtr parent,
                   bool t100) :
  name_(name),
  t100_(t100),
  link_ptr_(parent->GetLink(name)),
  body_ptr_(parent->GetLink("body")) {
    const double forwardMax = t100_ ? this->t100_for_max_ : this->t200_for_max_;
    const double reverseMax = t100_ ? this->t100_rev_max_ : this->t200_rev_max_;
    this->slopeForward = forwardMax / (PWM_MAX - PWM_MIN);
    this->slopeReverse = reverseMax / (PWM_MAX - PWM_MIN);
  }

void Thruster::addLinkForce() {
  // convert pwm into a force, then apply that force onto the link
  double f = this->pwmToForce(this->pwm_);

  // force gets applied at the "top" of the thruster
  ignition::math::Vector3d force = ignition::math::Vector3d(0, 0, f);

  this->link_ptr_->SetForce(force);
}

void Thruster::setPWM(int pwm) {
  this->pwm_ = pwm;
}

double Thruster::pwmToForce(int pwm) {
  double ret = 0.0;
  
  // TODO: make scaling more accurate to life (check datashet), not linear
  if (pwm > PWM_ZERO) {
    // map onto (0, forwardMax)
    ret = this->slopeForward * (pwm - PWM_MIN);
  }
  else {
    // map onto (0, reverseMax)
    ret = this->slopeReverse * (pwm - PWM_MIN);

    // flip to go backwards
    ret *= -1.0;
  }

  return ret;
}

}