#include "thruster_manager.h"

namespace gazebo {

std::unique_ptr<std::thread> sub_thread_ptr_;

ThrusterManager::ThrusterManager() {}

ThrusterManager::~ThrusterManager() {}

void ThrusterManager::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  num_thrusters_ = 6;
  ROS_INFO("Initializing Thruster Manager");

  //
  // Initialize our Gazebo publishers
  //
  this->gznode_ = transport::NodePtr(new transport::Node());
  this->gznode_->Init(_parent->GetWorld()->Name());

  this->thruster_pubs_ = std::vector<transport::PublisherPtr>(num_thrusters_);
  for (int i = 0; i < num_thrusters_; i++) {
    std::string topic_name = "/motor_" + std::string(1, char('A' + i));
    this->thruster_pubs_[i] = this->gznode_->Advertise<msgs::Int>(topic_name, 1);
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
  signal(SIGINT, &ThrusterManager::sigintHandler);

  this->update_time_ = ros::Duration(1.0 / 10.0);
  ROS_INFO("Thruster Manager Initialized");
}

void ThrusterManager::pwmCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
  // ROS_INFO("Received thruster command");
  for (int i = 0; i < this->num_thrusters_; i++) {
    msgs::Int m;
    m.set_data(int(msg->data[i]));
    this->thruster_pubs_[i]->Publish(m, false);
  }
}

void ThrusterManager::subThread() {
  ROS_INFO("SUB THREAD CREATED");
  ros::Subscriber sub = this->nh_->subscribe("/nautilus/motors/pwm", 1,
                                             &ThrusterManager::pwmCallback, this);
  ros::Rate r(10);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}

void ThrusterManager::sigintHandler(int sig) {
  ROS_INFO("Shutting down thruster man");
  ros::shutdown();
  sub_thread_ptr_->join();
}

void ThrusterManager::OnUpdate() {}


GZ_REGISTER_MODEL_PLUGIN(ThrusterManager)
}
