#include "sentry_chassis_controller/sentry_chassis_controller.h"

#include <pluginlib/class_list_macros.h>

namespace sentry_chassis_controller {

bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) {
  (void)nh;

  std::vector<std::string> steer_joint_names;
  std::vector<std::string> wheel_joint_names;

  if (!nh.getParam("steer_joints", steer_joint_names) || steer_joint_names.size() != 4U) {
    return false;
  }

  if (!nh.getParam("wheel_joints", wheel_joint_names) || wheel_joint_names.size() != 4U) {
    return false;
  }

  steer_joints_.clear();
  wheel_joints_.clear();

  for (const auto& name : steer_joint_names) {
    steer_joints_.push_back(hw->getHandle(name));
  }

  for (const auto& name : wheel_joint_names) {
    wheel_joints_.push_back(hw->getHandle(name));
  }

  return true;
}

void SentryChassisController::starting(const ros::Time& time) {
  (void)time;
  for (auto& joint : steer_joints_) {
    joint.setCommand(0.0);
  }
  for (auto& joint : wheel_joints_) {
    joint.setCommand(0.0);
  }
}

void SentryChassisController::update(const ros::Time& time, const ros::Duration& period) {
  (void)time;
  (void)period;

  for (auto& joint : steer_joints_) {
    joint.setCommand(0.0);
  }
  for (auto& joint : wheel_joints_) {
    joint.setCommand(0.0);
  }
}

void SentryChassisController::stopping(const ros::Time& time) {
  (void)time;
}

}  // namespace sentry_chassis_controller

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController,
                       controller_interface::ControllerBase)
