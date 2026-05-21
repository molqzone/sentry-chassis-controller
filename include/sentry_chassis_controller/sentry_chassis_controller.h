#pragma once

#include <effort_controllers/joint_position_controller.h>
#include <rm_chassis_controllers/chassis_base.h>

#include <Eigen/Dense>
#include <array>
#include <memory>
#include <vector>

namespace sentry_chassis_controller
{
class SentryChassisController
    : public rm_chassis_controllers::ChassisBase<hardware_interface::EffortJointInterface>
{
 public:
  SentryChassisController() = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

 private:
  struct Module
  {
    Eigen::Vector2d position{Eigen::Vector2d::Zero()};
    double pivot_offset = 0.0;
    double wheel_radius = 0.0;
    int wheel_rolling_sign = 1;
    int vx_sign = 1;
    int vy_sign = 1;
    int wz_sign = 1;
    std::unique_ptr<effort_controllers::JointPositionController> pivot;
    std::unique_ptr<effort_controllers::JointVelocityController> wheel;
  };

  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  geometry_msgs::Twist odometry() override;

  std::vector<Module> modules_;
};

}  // namespace sentry_chassis_controller
