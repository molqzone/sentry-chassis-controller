#include "sentry_chassis_controller/kinematics.hpp"

#include <gtest/gtest.h>

using sentry_chassis_controller::Kinematics;

TEST(Kinematics, ZeroInput) {
  Kinematics kinematics;
  const auto targets = kinematics.ComputeWheelAngularVelocity(0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(0.0, targets.front_left);
  EXPECT_DOUBLE_EQ(0.0, targets.front_right);
  EXPECT_DOUBLE_EQ(0.0, targets.rear_left);
  EXPECT_DOUBLE_EQ(0.0, targets.rear_right);
}

TEST(Kinematics, ForwardOnly) {
  Kinematics::Geometry geometry;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto targets = kinematics.ComputeWheelAngularVelocity(1.0, 0.0, 0.0);

  EXPECT_NEAR(10.0, targets.front_left, 1e-6);
  EXPECT_NEAR(10.0, targets.front_right, 1e-6);
  EXPECT_NEAR(10.0, targets.rear_left, 1e-6);
  EXPECT_NEAR(10.0, targets.rear_right, 1e-6);
}

TEST(Kinematics, PureYaw) {
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

TEST(Kinematics, LateralOnly) {
  Kinematics::Geometry geometry;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto targets = kinematics.ComputeWheelAngularVelocity(0.0, 1.0, 0.0);

  EXPECT_NEAR(-10.0, targets.front_left, 1e-6);
  EXPECT_NEAR(10.0, targets.front_right, 1e-6);
  EXPECT_NEAR(10.0, targets.rear_left, 1e-6);
  EXPECT_NEAR(-10.0, targets.rear_right, 1e-6);
}

TEST(Kinematics, CombinedMotion) {
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
