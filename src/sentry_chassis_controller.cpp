#include "sentry_chassis_controller/sentry_chassis_controller.h"
#include "sentry_chassis_controller/controller_math.hpp"
#include "controller_internal.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace sentry_chassis_controller
{
bool SentryChassisController::IsCommandTimedOut(bool command_valid,
                                                const ros::Time& command_stamp,
                                                const ros::Time& now, double timeout_sec)
{
  if (!command_valid)
  {
    return true;
  }
  if (timeout_sec < 0.0)
  {
    return true;
  }
  const double AGE = (now - command_stamp).toSec();
  return AGE < 0.0 || AGE > timeout_sec;
}

double SentryChassisController::NormalizeAngle(double angle)
{
  return Sophus::SO2d::exp(angle).log();
}

SentryChassisController::OdomState SentryChassisController::IntegrateOdom(
    const OdomState& state, const Kinematics::ChassisTwist& twist, double dt)
{
  if (dt <= 0.0)
  {
    return state;
  }

  const Sophus::SE2d WORLD_FROM_BASE(
      Sophus::SO2d::exp(state.yaw), Eigen::Vector2d(state.x, state.y));
  const Eigen::Vector3d BODY_TANGENT(twist.vx * dt, twist.vy * dt, twist.wz * dt);
  const Sophus::SE2d WORLD_FROM_BASE_NEXT = WORLD_FROM_BASE * Sophus::SE2d::exp(BODY_TANGENT);

  OdomState result;
  result.x = WORLD_FROM_BASE_NEXT.translation().x();
  result.y = WORLD_FROM_BASE_NEXT.translation().y();
  result.yaw = NormalizeAngle(WORLD_FROM_BASE_NEXT.so2().log());
  return result;
}

bool SentryChassisController::TransformTwistWithTransform(
    const Kinematics::ChassisTwist& input,
    const geometry_msgs::TransformStamped& transform, Kinematics::ChassisTwist* output)
{
  if (output == nullptr)
  {
    return false;
  }

  Eigen::Matrix3d source_to_target_rotation;
  if (!controller_math::BuildRotationFromQuaternion(
          transform.transform.rotation, MIN_QUATERNION_NORM,
          &source_to_target_rotation))
  {
    return false;
  }

  const Eigen::Vector3d LINEAR_INPUT(input.vx, input.vy, 0.0);
  const Eigen::Vector3d ANGULAR_INPUT(0.0, 0.0, input.wz);
  const Eigen::Vector3d LINEAR_OUTPUT = source_to_target_rotation * LINEAR_INPUT;
  const Eigen::Vector3d ANGULAR_OUTPUT = source_to_target_rotation * ANGULAR_INPUT;

  output->vx = LINEAR_OUTPUT.x();
  output->vy = LINEAR_OUTPUT.y();
  output->wz = ANGULAR_OUTPUT.z();
  return true;
}

void SentryChassisController::CmdVelCallback(const geometry_msgs::TwistConstPtr& message)
{
  CommandData command;
  command.vx = message->linear.x;
  command.vy = message->linear.y;
  command.wz = message->angular.z;
  command.stamp = ros::Time::now();
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
}

void SentryChassisController::SetAllCommands(
    std::array<hardware_interface::JointHandle, WHEEL_COUNT>* joints, double command)
{
  for (auto& joint : *joints)
  {
    joint.setCommand(command);
  }
}

}  // namespace sentry_chassis_controller

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController,
                       controller_interface::ControllerBase)
