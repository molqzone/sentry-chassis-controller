#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <utility>

#include "sentry_chassis_controller/kinematics.hpp"

using sentry_chassis_controller::Kinematics;

namespace
{
std::array<std::pair<double, double>, 4> build_module_positions(
    const Kinematics::Geometry& geometry)
{
  const double HALF_WHEEL_BASE = geometry.wheel_base * 0.5;
  const double HALF_TRACK_WIDTH = geometry.track_width * 0.5;
  return {{
      {HALF_WHEEL_BASE, HALF_TRACK_WIDTH},
      {HALF_WHEEL_BASE, -HALF_TRACK_WIDTH},
      {-HALF_WHEEL_BASE, HALF_TRACK_WIDTH},
      {-HALF_WHEEL_BASE, -HALF_TRACK_WIDTH},
  }};
}

Kinematics::WheelFeedback build_feedback_from_twist(
    const Kinematics::Geometry& geometry, const Kinematics::ChassisTwist& twist,
    const std::array<double, 4>& steer_positions,
    const std::array<double, 4>& steer_zero_offsets,
    const std::array<int, 4>& wheel_rolling_signs)
{
  Kinematics::WheelFeedback feedback;
  feedback.steer_position = steer_positions;

  const auto MODULE_POSITIONS = build_module_positions(geometry);
  for (std::size_t index = 0; index < 4; ++index)
  {
    const double THETA = steer_positions[index] - steer_zero_offsets[index];
    const double DIRECTION_X = std::cos(THETA);
    const double DIRECTION_Y = std::sin(THETA);
    const double MODULE_X = MODULE_POSITIONS[index].first;
    const double MODULE_Y = MODULE_POSITIONS[index].second;
    const double WHEEL_LINEAR_SPEED =
        DIRECTION_X * twist.vx + DIRECTION_Y * twist.vy +
        (-DIRECTION_X * MODULE_Y + DIRECTION_Y * MODULE_X) * twist.wz;
    feedback.wheel_angular_velocity[index] =
        WHEEL_LINEAR_SPEED /
        (static_cast<double>(wheel_rolling_signs[index]) * geometry.wheel_radius);
  }
  return feedback;
}
}  // namespace

TEST(Kinematics, ZeroInput)
{
  Kinematics kinematics;
  const auto TARGETS = kinematics.ComputeWheelAngularVelocity(0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(0.0, TARGETS.front_left);
  EXPECT_DOUBLE_EQ(0.0, TARGETS.front_right);
  EXPECT_DOUBLE_EQ(0.0, TARGETS.rear_left);
  EXPECT_DOUBLE_EQ(0.0, TARGETS.rear_right);
}

TEST(Kinematics, ForwardOnly)
{
  Kinematics::Geometry geometry;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto TARGETS = kinematics.ComputeWheelAngularVelocity(1.0, 0.0, 0.0);

  EXPECT_NEAR(10.0, TARGETS.front_left, 1e-6);
  EXPECT_NEAR(10.0, TARGETS.front_right, 1e-6);
  EXPECT_NEAR(10.0, TARGETS.rear_left, 1e-6);
  EXPECT_NEAR(10.0, TARGETS.rear_right, 1e-6);
}

TEST(Kinematics, PureYaw)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.60;
  geometry.track_width = 0.40;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto TARGETS = kinematics.ComputeWheelAngularVelocity(0.0, 0.0, 1.0);

  EXPECT_NEAR(-5.0, TARGETS.front_left, 1e-6);
  EXPECT_NEAR(5.0, TARGETS.front_right, 1e-6);
  EXPECT_NEAR(-5.0, TARGETS.rear_left, 1e-6);
  EXPECT_NEAR(5.0, TARGETS.rear_right, 1e-6);
}

TEST(Kinematics, LateralOnly)
{
  Kinematics::Geometry geometry;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto TARGETS = kinematics.ComputeWheelAngularVelocity(0.0, 1.0, 0.0);

  EXPECT_NEAR(-10.0, TARGETS.front_left, 1e-6);
  EXPECT_NEAR(10.0, TARGETS.front_right, 1e-6);
  EXPECT_NEAR(10.0, TARGETS.rear_left, 1e-6);
  EXPECT_NEAR(-10.0, TARGETS.rear_right, 1e-6);
}

TEST(Kinematics, CombinedMotion)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.60;
  geometry.track_width = 0.40;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto TARGETS = kinematics.ComputeWheelAngularVelocity(0.8, 0.2, 1.0);

  EXPECT_NEAR(1.0, TARGETS.front_left, 1e-6);
  EXPECT_NEAR(15.0, TARGETS.front_right, 1e-6);
  EXPECT_NEAR(5.0, TARGETS.rear_left, 1e-6);
  EXPECT_NEAR(11.0, TARGETS.rear_right, 1e-6);
}

TEST(Kinematics, DirectionSignsDefaultKeepsLegacyEquation)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.60;
  geometry.track_width = 0.40;
  geometry.wheel_radius = 0.10;

  Kinematics kinematics(geometry);
  const auto TARGETS = kinematics.ComputeWheelAngularVelocity(0.8, 0.2, 1.0);

  EXPECT_NEAR(1.0, TARGETS.front_left, 1e-6);
  EXPECT_NEAR(15.0, TARGETS.front_right, 1e-6);
  EXPECT_NEAR(5.0, TARGETS.rear_left, 1e-6);
  EXPECT_NEAR(11.0, TARGETS.rear_right, 1e-6);
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

  const auto VX_TARGETS = kinematics.ComputeWheelAngularVelocity(1.0, 0.0, 0.0);
  EXPECT_NEAR(10.0, VX_TARGETS.front_left, 1e-6);
  EXPECT_NEAR(10.0, VX_TARGETS.front_right, 1e-6);
  EXPECT_NEAR(-10.0, VX_TARGETS.rear_left, 1e-6);
  EXPECT_NEAR(-10.0, VX_TARGETS.rear_right, 1e-6);

  const auto VY_TARGETS = kinematics.ComputeWheelAngularVelocity(0.0, 1.0, 0.0);
  EXPECT_NEAR(10.0, VY_TARGETS.front_left, 1e-6);
  EXPECT_NEAR(-10.0, VY_TARGETS.front_right, 1e-6);
  EXPECT_NEAR(10.0, VY_TARGETS.rear_left, 1e-6);
  EXPECT_NEAR(-10.0, VY_TARGETS.rear_right, 1e-6);

  const auto WZ_TARGETS = kinematics.ComputeWheelAngularVelocity(0.0, 0.0, 1.0);
  EXPECT_NEAR(5.0, WZ_TARGETS.front_left, 1e-6);
  EXPECT_NEAR(-5.0, WZ_TARGETS.front_right, 1e-6);
  EXPECT_NEAR(-5.0, WZ_TARGETS.rear_left, 1e-6);
  EXPECT_NEAR(5.0, WZ_TARGETS.rear_right, 1e-6);
}

TEST(Kinematics, ForwardKinematicsRecoversKnownTwist)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.62;
  geometry.track_width = 0.46;
  geometry.wheel_radius = 0.09;

  const Kinematics::ChassisTwist EXPECTED_TWIST{0.8, -0.3, 1.2};
  const std::array<double, 4> STEER_POSITIONS{{0.35, -0.4, 2.5, -2.1}};
  const std::array<double, 4> STEER_ZERO_OFFSETS{{0.0, 0.0, 0.0, 0.0}};
  const std::array<int, 4> WHEEL_ROLLING_SIGNS{{1, -1, -1, 1}};

  Kinematics kinematics(geometry);
  const auto FEEDBACK = build_feedback_from_twist(
      geometry, EXPECTED_TWIST, STEER_POSITIONS, STEER_ZERO_OFFSETS, WHEEL_ROLLING_SIGNS);

  Kinematics::ChassisTwist solved_twist;
  ASSERT_TRUE(kinematics.ComputeChassisTwistFromWheelFeedback(
      FEEDBACK, STEER_ZERO_OFFSETS, WHEEL_ROLLING_SIGNS, &solved_twist));
  EXPECT_NEAR(EXPECTED_TWIST.vx, solved_twist.vx, 1e-6);
  EXPECT_NEAR(EXPECTED_TWIST.vy, solved_twist.vy, 1e-6);
  EXPECT_NEAR(EXPECTED_TWIST.wz, solved_twist.wz, 1e-6);
}

TEST(Kinematics, ForwardKinematicsHonorsRollingSigns)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.62;
  geometry.track_width = 0.46;
  geometry.wheel_radius = 0.09;

  const Kinematics::ChassisTwist EXPECTED_TWIST{0.8, -0.3, 1.2};
  const std::array<double, 4> STEER_POSITIONS{{0.35, -0.4, 2.5, -2.1}};
  const std::array<double, 4> STEER_ZERO_OFFSETS{{0.0, 0.0, 0.0, 0.0}};
  const std::array<int, 4> CORRECT_SIGNS{{1, -1, -1, 1}};
  const std::array<int, 4> WRONG_SIGNS{{1, 1, 1, 1}};

  Kinematics kinematics(geometry);
  const auto FEEDBACK = build_feedback_from_twist(
      geometry, EXPECTED_TWIST, STEER_POSITIONS, STEER_ZERO_OFFSETS, CORRECT_SIGNS);

  Kinematics::ChassisTwist solved_correct;
  ASSERT_TRUE(kinematics.ComputeChassisTwistFromWheelFeedback(
      FEEDBACK, STEER_ZERO_OFFSETS, CORRECT_SIGNS, &solved_correct));
  EXPECT_NEAR(EXPECTED_TWIST.vx, solved_correct.vx, 1e-6);
  EXPECT_NEAR(EXPECTED_TWIST.vy, solved_correct.vy, 1e-6);
  EXPECT_NEAR(EXPECTED_TWIST.wz, solved_correct.wz, 1e-6);

  Kinematics::ChassisTwist solved_wrong;
  ASSERT_TRUE(kinematics.ComputeChassisTwistFromWheelFeedback(
      FEEDBACK, STEER_ZERO_OFFSETS, WRONG_SIGNS, &solved_wrong));
  EXPECT_GT(std::fabs(solved_wrong.vx - EXPECTED_TWIST.vx), 0.1);
}

TEST(Kinematics, ForwardKinematicsRejectsSingularSetup)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.62;
  geometry.track_width = 0.46;
  geometry.wheel_radius = 0.09;

  Kinematics::WheelFeedback feedback;
  feedback.steer_position = {{0.0, 0.0, 0.0, 0.0}};
  feedback.wheel_angular_velocity = {{1.0, 1.0, 1.0, 1.0}};

  const std::array<double, 4> STEER_ZERO_OFFSETS{{0.0, 0.0, 0.0, 0.0}};
  const std::array<int, 4> ROLLING_SIGNS{{1, 1, 1, 1}};

  Kinematics kinematics(geometry);
  Kinematics::ChassisTwist solved;
  EXPECT_FALSE(kinematics.ComputeChassisTwistFromWheelFeedback(
      feedback, STEER_ZERO_OFFSETS, ROLLING_SIGNS, &solved));
}
