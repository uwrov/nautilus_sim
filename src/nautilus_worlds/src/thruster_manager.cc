#include "thruster_manager.h"

namespace gazebo {

std::unique_ptr<std::thread> sub_thread_ptr_;

ThrusterManager::ThrusterManager() {}

ThrusterManager::~ThrusterManager() {}

void ThrusterManager::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  num_thrusters_ = 6;
  ROS_INFO("Initializing Thruster Manager");

  // gather motor links
  for (int i = 0; i < this->num_thrusters_; i++) {
    std::string motor_link_name = "motor_" + std::string(1, char('A' + i));
    if (i < this->num_t100_)
      this->thrusters_.push_back(Thruster(motor_link_name, _parent, true));
    else
      this->thrusters_.push_back(Thruster(motor_link_name, _parent, false));
  }

  // bind update function
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ThrusterManager::OnUpdate, this));

  //
  // Initialize ROS Node
  //
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gz_thruster_man", ros::init_options::NoSigintHandler);
  this->nh_.reset(new ros::NodeHandle("gz_thruster_man"));

  // start a new thread for the subscriber to live in
  sub_thread_ptr_.reset(new std::thread(&ThrusterManager::subThread, this));
  
  // handling a sigint might override gazebo's own cleanup stuff
  // replacing it with our own causes a hang on sigint instead
  // of a good cleanup
  // signal(SIGINT, &ThrusterManager::sigintHandler);

  this->update_time_ = ros::Duration(1.0 / 10.0);
  ROS_INFO("Thruster Manager Initialized");
}

void ThrusterManager::OnUpdate() {
  // apply forces on each of the motor links
  for (int i = 0; i < this->num_thrusters_; i++) {
    this->thrusters_.at(i).addLinkForce();
  }
}

void ThrusterManager::pwmCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
  // ROS_INFO("Received thruster command");
  for (int i = 0; i < this->num_thrusters_; i++) {
    this->thrusters_.at(i).setPWM(msg->data[i]);
  }
}

void ThrusterManager::subThread() {
  ROS_INFO("SUB THREAD CREATED");
  ros::Subscriber sub = this->nh_->subscribe("/nautilus/motors/pwm", 1,
                                             &ThrusterManager::pwmCallback, this);
  ros::Rate r(60);

  while (ros::ok() || !ros::isShuttingDown()) {
    ros::spinOnce();
    r.sleep();
  }
}

void ThrusterManager::sigintHandler(int sig) {
  ROS_INFO("Shutting down thruster man");
  ros::shutdown();
  sub_thread_ptr_->join();
}

GZ_REGISTER_MODEL_PLUGIN(ThrusterManager)
}
