#ifndef THRUSTER_MAN_H
#define THRUSTER_MAN_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>

#include <vector>
#include <memory>
#include <signal.h>
/**
 * ROS Subscriber which reads in an array of 6 floats representing
 * pwms to be applied onto T100/T200 motors. Uses a gazebo topic
 * to forward the pwms to the appropriate thrusters.
 */ 

namespace gazebo {
class ThrusterManager : public ModelPlugin {
  public:
    ThrusterManager();
    ~ThrusterManager();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    virtual void OnUpdate();

  private:
    std::unique_ptr<ros::NodeHandle>  nh_;
    ros::Subscriber thruster_sub_;

    transport::NodePtr gznode_;
    std::vector<transport::PublisherPtr> thruster_pubs_;

    int num_thrusters_;

    void pwmCallback(const std_msgs::Int16MultiArray::ConstPtr& msg);
    static void sigintHandler(int sig);
};
}

#endif // THRUSTER_MAN_H