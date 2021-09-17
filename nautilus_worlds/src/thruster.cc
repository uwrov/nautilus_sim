#include "thruster.h"

Thruster::Thruster(const std::string name, physics::ModelPtr parent,
                   double max_force, bool t100) :
  current_force_(0),
  max_force_(max_force),
  name_(name),
  t100_(t100)
  link_ptr_(parent->GetLink(name)),
  _body_ptr_(parent->GetLink("body")) {}

void Thruster::addLinkForce(int pwm) {
  // convert pwm into a force, then apply that force onto the frame
}