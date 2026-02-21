#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_

#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>

namespace sentry_chassis_controller {

class SentryChassisController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

  void starting(const ros::Time& time) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void stopping(const ros::Time& time) override;

 private:
  std::vector<hardware_interface::JointHandle> steer_joints_;
  std::vector<hardware_interface::JointHandle> wheel_joints_;
};

}  // namespace sentry_chassis_controller

#endif  // SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_
