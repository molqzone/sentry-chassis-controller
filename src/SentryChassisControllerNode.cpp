#include "sentry_chassis_controller/SentryChassisControllerNode.hpp"

namespace sentry_chassis_controller {

SentryChassisControllerNode::SentryChassisControllerNode(ros::NodeHandle& node_handle)
    : node_handle_(node_handle), cmd_vel_topic_("/cmd_vel") {
  if (!readParameters()) {
    ROS_ERROR("Failed to read sentry_chassis_controller node parameters.");
    ros::requestShutdown();
    return;
  }

  cmd_vel_subscriber_ = node_handle_.subscribe(cmd_vel_topic_, 10,
                                                &SentryChassisControllerNode::cmdVelCallback, this);
  ROS_INFO("sentry_chassis_controller_node started. cmd_vel_topic=%s", cmd_vel_topic_.c_str());
}

bool SentryChassisControllerNode::readParameters() {
  Kinematics::Geometry geometry;
  node_handle_.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
  node_handle_.param("geometry/wheel_base", geometry.wheel_base, 0.50);
  node_handle_.param("geometry/track_width", geometry.track_width, 0.40);
  node_handle_.param("geometry/wheel_radius", geometry.wheel_radius, 0.076);

  kinematics_.setGeometry(geometry);
  return true;
}

void SentryChassisControllerNode::cmdVelCallback(const geometry_msgs::TwistConstPtr& message) {
  const auto targets = kinematics_.computeWheelAngularVelocity(
      message->linear.x, message->linear.y, message->angular.z);

  ROS_DEBUG("wheel targets(rad/s): fl=%.3f fr=%.3f rl=%.3f rr=%.3f", targets.front_left,
            targets.front_right, targets.rear_left, targets.rear_right);
}

}  // namespace sentry_chassis_controller
