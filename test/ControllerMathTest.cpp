#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>

#include "sentry_chassis_controller/controller_math.hpp"

namespace sentry_chassis_controller
{
namespace
{

TEST(ControllerMathTest, BuildRotationFromQuaternionRejectsDegenerateInput)
{
  geometry_msgs::Quaternion quaternion;
  quaternion.w = 0.0;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = 0.0;

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  const bool success = controller_math::BuildRotationFromQuaternion(
      quaternion, 1e-12, &rotation);

  EXPECT_FALSE(success);
}

TEST(ControllerMathTest, BuildRotationFromQuaternionProducesNormalizedRotation)
{
  geometry_msgs::Quaternion quaternion;
  // Same orientation as [w=0, z=1] after normalization.
  quaternion.w = 0.0;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = 2.0;

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  const bool success = controller_math::BuildRotationFromQuaternion(
      quaternion, 1e-12, &rotation);

  ASSERT_TRUE(success);
  const Eigen::Vector3d rotated_x = rotation * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(rotated_x.x(), -1.0, 1e-9);
  EXPECT_NEAR(rotated_x.y(), 0.0, 1e-9);
  EXPECT_NEAR(rotated_x.z(), 0.0, 1e-9);
}

TEST(ControllerMathTest, ReverseCcwCompensationScalesVxAndWz)
{
  controller_math::ReverseCompensationParams params;
  params.reverse_ccw_vy_threshold = 0.05;
  params.reverse_ccw_vx_scale = 0.6;
  params.reverse_ccw_wz_gain = 1.8;
  params.zero_cmd_eps = 1e-4;
  params.reverse_straight_vx_boost = 1.1;

  Eigen::Matrix3d compensation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d command(-1.0, 0.0, 2.0);

  controller_math::ApplyNonLinearReverseCompensation(params, &compensation, &command);

  EXPECT_NEAR(command.x(), -0.6, 1e-9);
  EXPECT_NEAR(command.y(), 0.0, 1e-9);
  EXPECT_NEAR(command.z(), 3.6, 1e-9);
  EXPECT_NEAR(compensation(0, 0), 0.6, 1e-9);
  EXPECT_NEAR(compensation(1, 1), 1.0, 1e-9);
  EXPECT_NEAR(compensation(2, 2), 1.8, 1e-9);
}

TEST(ControllerMathTest, ReverseStraightCompensationOnlyBoostsVx)
{
  controller_math::ReverseCompensationParams params;
  params.reverse_ccw_vy_threshold = 0.05;
  params.reverse_ccw_vx_scale = 0.6;
  params.reverse_ccw_wz_gain = 1.8;
  params.zero_cmd_eps = 1e-4;
  params.reverse_straight_vx_boost = 1.1;

  Eigen::Matrix3d compensation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d command(-1.0, 0.01, 0.0);

  controller_math::ApplyNonLinearReverseCompensation(params, &compensation, &command);

  EXPECT_NEAR(command.x(), -1.1, 1e-9);
  EXPECT_NEAR(command.y(), 0.01, 1e-9);
  EXPECT_NEAR(command.z(), 0.0, 1e-9);
  EXPECT_NEAR(compensation(0, 0), 1.1, 1e-9);
  EXPECT_NEAR(compensation(1, 1), 1.0, 1e-9);
  EXPECT_NEAR(compensation(2, 2), 1.0, 1e-9);
}

}  // namespace
}  // namespace sentry_chassis_controller
