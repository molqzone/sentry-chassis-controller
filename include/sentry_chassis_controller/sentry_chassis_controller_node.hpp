#pragma once

#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include "sentry_chassis_controller/kinematics.hpp"

namespace sentry_chassis_controller {

class SentryChassisControllerNode {
 public:
  explicit SentryChassisControllerNode(ros::NodeHandle& node_handle);

 private:
  bool ReadParameters();

  void CmdVelCallback(const geometry_msgs::TwistConstPtr& message);

  ros::NodeHandle& node_handle_;
  ros::Subscriber cmd_vel_subscriber_;
  Kinematics kinematics_;
  std::string cmd_vel_topic_;
};

}  // namespace sentry_chassis_controller
