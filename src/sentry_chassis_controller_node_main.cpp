#include <ros/ros.h>

#include "sentry_chassis_controller/sentry_chassis_controller_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sentry_chassis_controller_node");
  ros::NodeHandle node_handle("~");

  sentry_chassis_controller::SentryChassisControllerNode node(node_handle);
  ros::spin();
  return 0;
}
