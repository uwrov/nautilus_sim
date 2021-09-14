#include "thruster_manager.h"

namespace gazebo {

ThrusterManager::ThrusterManager() {}

ThrusterManager::~ThrusterManager() {}

void ThrusterManager::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  num_thrusters_ = 6;
  ROS_INFO("Initializing Thruster Manager");

  // Initialize our Gazebo publishers
  this->gznode_ = transport::NodePtr(new transport::Node());
  this->gznode_->Init();

  this->thruster_pubs_ = std::vector<transport::PublisherPtr>(num_thrusters_);
  for (int i = 0; i < num_thrusters_; i++) {
    std::string topic_name = "/motor_" + std::string(1, char('A' + i));
    this->thruster_pubs_[i] = this->gznode_->Advertise<msgs::Int>(topic_name, 1);
  }

  // Initialize ROS Node and Subscriber
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gz_thruster_man");

  // this->nh_ = new ros::NodeHandle("gz_thruster_man");
  this->nh_.reset(new ros::NodeHandle("gz_thruster_man"));

  ThrusterManager tm;
  ros::Subscriber sub = this->nh_->subscribe("/nautilus/motors/pwm", 1, &ThrusterManager::pwmCallback, this);
  signal(SIGINT, &ThrusterManager::sigintHandler);

  ROS_INFO("Thruster Manager Initialized");

  ros::spin();
}

void ThrusterManager::OnUpdate() {}

void ThrusterManager::pwmCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
  // ROS_INFO("Received thruster command");
  for (int i = 0; i < this->num_thrusters_; i++) {
    msgs::Int m;
    m.set_data(int(msg->data[i]));
    this->thruster_pubs_[i]->Publish(m, false);
  }
}

void ThrusterManager::sigintHandler(int sig) {
  ROS_INFO("Thruster man - shutting down");

  ros::shutdown();
}

GZ_REGISTER_MODEL_PLUGIN(ThrusterManager)
}
