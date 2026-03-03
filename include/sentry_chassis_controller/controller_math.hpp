#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>

namespace sentry_chassis_controller
{
namespace controller_math
{

struct ReverseCompensationParams
{
  double reverse_ccw_vy_threshold = 0.0;
  double reverse_ccw_vx_scale = 1.0;
  double reverse_ccw_wz_gain = 1.0;
  double zero_cmd_eps = 1e-4;
  double reverse_straight_vx_boost = 1.0;
};

/**
 * @brief Builds a rotation matrix from quaternion with norm guard.
 */
bool BuildRotationFromQuaternion(const geometry_msgs::Quaternion& quaternion,
                                 double min_quaternion_norm,
                                 Eigen::Matrix3d* rotation_matrix);

/**
 * @brief Applies non-linear reverse compensation to command mapping.
 */
void ApplyNonLinearReverseCompensation(const ReverseCompensationParams& params,
                                       Eigen::Matrix3d* command_compensation_effective,
                                       Eigen::Vector3d* command_effective);

}  // namespace controller_math
}  // namespace sentry_chassis_controller
