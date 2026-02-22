#include <gtest/gtest.h>

#include "sentry_chassis_controller/kinematics.hpp"

using sentry_chassis_controller::Kinematics;

TEST(Kinematics, ZeroInput)
{
  Kinematics kinematics;
  const auto targets = kinematics.ComputeWheelAngularVelocity(0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(0.0, targets.front_left);
  EXPECT_DOUBLE_EQ(0.0, targets.front_right);
  EXPECT_DOUBLE_EQ(0.0, targets.rear_left);
  EXPECT_DOUBLE_EQ(0.0, targets.rear_right);
}

TEST(Kinematics, ForwardOnly)
{
  Kinematics::Geometry geometry;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto targets = kinematics.ComputeWheelAngularVelocity(1.0, 0.0, 0.0);

  EXPECT_NEAR(10.0, targets.front_left, 1e-6);
  EXPECT_NEAR(10.0, targets.front_right, 1e-6);
  EXPECT_NEAR(10.0, targets.rear_left, 1e-6);
  EXPECT_NEAR(10.0, targets.rear_right, 1e-6);
}

TEST(Kinematics, PureYaw)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.60;
  geometry.track_width = 0.40;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto targets = kinematics.ComputeWheelAngularVelocity(0.0, 0.0, 1.0);

  EXPECT_NEAR(-5.0, targets.front_left, 1e-6);
  EXPECT_NEAR(5.0, targets.front_right, 1e-6);
  EXPECT_NEAR(-5.0, targets.rear_left, 1e-6);
  EXPECT_NEAR(5.0, targets.rear_right, 1e-6);
}

TEST(Kinematics, LateralOnly)
{
  Kinematics::Geometry geometry;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto targets = kinematics.ComputeWheelAngularVelocity(0.0, 1.0, 0.0);

  EXPECT_NEAR(-10.0, targets.front_left, 1e-6);
  EXPECT_NEAR(10.0, targets.front_right, 1e-6);
  EXPECT_NEAR(10.0, targets.rear_left, 1e-6);
  EXPECT_NEAR(-10.0, targets.rear_right, 1e-6);
}

TEST(Kinematics, CombinedMotion)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.60;
  geometry.track_width = 0.40;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto targets = kinematics.ComputeWheelAngularVelocity(0.8, 0.2, 1.0);

  EXPECT_NEAR(1.0, targets.front_left, 1e-6);
  EXPECT_NEAR(15.0, targets.front_right, 1e-6);
  EXPECT_NEAR(5.0, targets.rear_left, 1e-6);
  EXPECT_NEAR(11.0, targets.rear_right, 1e-6);
}

TEST(Kinematics, DirectionSignsDefaultKeepsLegacyEquation)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.60;
  geometry.track_width = 0.40;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto targets = kinematics.ComputeWheelAngularVelocity(0.8, 0.2, 1.0);

  EXPECT_NEAR(1.0, targets.front_left, 1e-6);
  EXPECT_NEAR(15.0, targets.front_right, 1e-6);
  EXPECT_NEAR(5.0, targets.rear_left, 1e-6);
  EXPECT_NEAR(11.0, targets.rear_right, 1e-6);
}

TEST(Kinematics, DirectionSignsCustomMatrixAppliesPerAxis)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.60;
  geometry.track_width = 0.40;
  geometry.wheel_radius = 0.10;

  Kinematics::DirectionSigns direction_signs;
  direction_signs.vx = {{1, 1, -1, -1}};
  direction_signs.vy = {{-1, -1, 1, 1}};
  direction_signs.wz = {{-1, -1, 1, 1}};

  Kinematics kinematics(geometry);
  kinematics.SetDirectionSigns(direction_signs);

  const auto vx_targets = kinematics.ComputeWheelAngularVelocity(1.0, 0.0, 0.0);
  EXPECT_NEAR(10.0, vx_targets.front_left, 1e-6);
  EXPECT_NEAR(10.0, vx_targets.front_right, 1e-6);
  EXPECT_NEAR(-10.0, vx_targets.rear_left, 1e-6);
  EXPECT_NEAR(-10.0, vx_targets.rear_right, 1e-6);

  const auto vy_targets = kinematics.ComputeWheelAngularVelocity(0.0, 1.0, 0.0);
  EXPECT_NEAR(10.0, vy_targets.front_left, 1e-6);
  EXPECT_NEAR(-10.0, vy_targets.front_right, 1e-6);
  EXPECT_NEAR(10.0, vy_targets.rear_left, 1e-6);
  EXPECT_NEAR(-10.0, vy_targets.rear_right, 1e-6);

  const auto wz_targets = kinematics.ComputeWheelAngularVelocity(0.0, 0.0, 1.0);
  EXPECT_NEAR(5.0, wz_targets.front_left, 1e-6);
  EXPECT_NEAR(-5.0, wz_targets.front_right, 1e-6);
  EXPECT_NEAR(-5.0, wz_targets.rear_left, 1e-6);
  EXPECT_NEAR(5.0, wz_targets.rear_right, 1e-6);
}
