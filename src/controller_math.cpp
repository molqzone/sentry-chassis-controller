#include "sentry_chassis_controller/controller_math.hpp"

#include <cmath>

namespace sentry_chassis_controller
{
namespace controller_math
{

bool BuildRotationFromQuaternion(const geometry_msgs::Quaternion& quaternion,
                                 double min_quaternion_norm,
                                 Eigen::Matrix3d* rotation_matrix)
{
  if (rotation_matrix == nullptr)
  {
    return false;
  }

  const Eigen::Quaterniond raw_quaternion(quaternion.w, quaternion.x, quaternion.y,
                                          quaternion.z);
  if (raw_quaternion.norm() < min_quaternion_norm)
  {
    return false;
  }

  const Eigen::Quaterniond normalized_quaternion = raw_quaternion.normalized();
  *rotation_matrix = normalized_quaternion.toRotationMatrix();
  return true;
}

void ApplyNonLinearReverseCompensation(const ReverseCompensationParams& params,
                                       Eigen::Matrix3d* command_compensation_effective,
                                       Eigen::Vector3d* command_effective)
{
  if (command_compensation_effective == nullptr || command_effective == nullptr)
  {
    return;
  }

  double vx = command_effective->x();
  const double vy = command_effective->y();
  double wz = command_effective->z();
  if (vx < -params.zero_cmd_eps &&
      std::fabs(vy) <= params.reverse_ccw_vy_threshold && wz > params.zero_cmd_eps)
  {
    Eigen::Matrix3d reverse_ccw_scaling = Eigen::Matrix3d::Identity();
    reverse_ccw_scaling(0, 0) = params.reverse_ccw_vx_scale;
    reverse_ccw_scaling(2, 2) = params.reverse_ccw_wz_gain;
    *command_compensation_effective =
        reverse_ccw_scaling * (*command_compensation_effective);
    *command_effective = reverse_ccw_scaling * (*command_effective);
    vx = command_effective->x();
    wz = command_effective->z();
  }
  if (vx < -params.zero_cmd_eps &&
      std::fabs(vy) <= params.reverse_ccw_vy_threshold &&
      std::fabs(wz) <= params.zero_cmd_eps)
  {
    Eigen::Matrix3d reverse_straight_scaling = Eigen::Matrix3d::Identity();
    reverse_straight_scaling(0, 0) = params.reverse_straight_vx_boost;
    *command_compensation_effective =
        reverse_straight_scaling * (*command_compensation_effective);
    *command_effective = reverse_straight_scaling * (*command_effective);
  }
}

}  // namespace controller_math
}  // namespace sentry_chassis_controller
