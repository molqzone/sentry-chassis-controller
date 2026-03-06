#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <utility>

#include "sentry_chassis_controller/kinematics.hpp"

using sentry_chassis_controller::Kinematics;

namespace
{
std::array<std::pair<double, double>, Kinematics::WHEEL_COUNT> build_module_positions(
    const Kinematics::Geometry& geometry)
{
  const double half_wheel_base = geometry.wheel_base * 0.5;
  const double half_track_width = geometry.track_width * 0.5;
  return {{
      {half_wheel_base, half_track_width},
      {half_wheel_base, -half_track_width},
      {-half_wheel_base, half_track_width},
      {-half_wheel_base, -half_track_width},
  }};
}

Kinematics::WheelFeedback build_feedback_from_twist(
    const Kinematics::Geometry& geometry, const Kinematics::ChassisTwist& twist,
    const std::array<double, Kinematics::WHEEL_COUNT>& steer_positions,
    const std::array<double, Kinematics::WHEEL_COUNT>& steer_zero_offsets,
    const std::array<int, Kinematics::WHEEL_COUNT>& wheel_rolling_signs)
{
  Kinematics::WheelFeedback feedback;
  feedback.steer_position = steer_positions;

  const auto module_positions = build_module_positions(geometry);
  for (std::size_t index = 0; index < Kinematics::WHEEL_COUNT; ++index)
  {
    const double theta = steer_positions[index] - steer_zero_offsets[index];
    const double direction_x = std::cos(theta);
    const double direction_y = std::sin(theta);
    const double module_x = module_positions[index].first;
    const double module_y = module_positions[index].second;
    const double wheel_linear_speed =
        direction_x * twist.vx + direction_y * twist.vy +
        (-direction_x * module_y + direction_y * module_x) * twist.wz;
    feedback.wheel_angular_velocity[index] =
        wheel_linear_speed /
        (static_cast<double>(wheel_rolling_signs[index]) * geometry.wheel_radius);
  }
  return feedback;
}
}  // namespace

TEST(Kinematics, ComputeWheelTargetsKeepsIdleModulesInactive)
{
  Kinematics kinematics;
  const Kinematics::ChassisTwist twist{0.0, 0.0, 0.0};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_zero_offsets{{0.1, -0.2, 0.3, -0.4}};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_positions{{0.0, 0.0, 0.0, 0.0}};

  const auto solution =
      kinematics.ComputeWheelTargets(twist, steer_zero_offsets, steer_positions);

  for (std::size_t index = 0; index < Kinematics::WHEEL_COUNT; ++index)
  {
    EXPECT_FALSE(solution.modules[index].active);
    EXPECT_NEAR(steer_zero_offsets[index], solution.modules[index].steer_error, 1e-9);
    EXPECT_NEAR(0.0, solution.modules[index].wheel_angular_velocity, 1e-9);
  }
}

TEST(Kinematics, ComputeWheelTargetsSolvesForwardCommandForAlignedModules)
{
  Kinematics::Geometry geometry;
  geometry.wheel_radius = 0.10;

  const Kinematics::ChassisTwist twist{1.0, 0.0, 0.0};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_zero_offsets{{0.0, 0.0, 0.0, 0.0}};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_positions{{0.0, 0.0, 0.0, 0.0}};

  Kinematics kinematics(geometry);
  const auto solution =
      kinematics.ComputeWheelTargets(twist, steer_zero_offsets, steer_positions);

  for (std::size_t index = 0; index < Kinematics::WHEEL_COUNT; ++index)
  {
    EXPECT_TRUE(solution.modules[index].active);
    EXPECT_NEAR(0.0, solution.modules[index].steer_error, 1e-9);
    EXPECT_NEAR(10.0, solution.modules[index].wheel_angular_velocity, 1e-6);
  }
}

TEST(Kinematics, ComputeWheelTargetsFlipsWheelDirectionForShortestSteerPath)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.60;
  geometry.track_width = 0.40;
  geometry.wheel_radius = 0.10;

  const Kinematics::ChassisTwist twist{0.0, 0.0, 1.0};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_zero_offsets{{0.0, 0.0, 0.0, 0.0}};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_positions{{0.0, 0.0, 0.0, 0.0}};
  const double expected_speed = std::hypot(0.2, 0.3) / geometry.wheel_radius;
  const double expected_error = std::atan2(0.3, 0.2);

  Kinematics kinematics(geometry);
  const auto solution =
      kinematics.ComputeWheelTargets(twist, steer_zero_offsets, steer_positions);

  ASSERT_TRUE(solution.modules[0].active);
  ASSERT_TRUE(solution.modules[1].active);
  ASSERT_TRUE(solution.modules[2].active);
  ASSERT_TRUE(solution.modules[3].active);
  EXPECT_NEAR(-expected_error, solution.modules[0].steer_error, 1e-6);
  EXPECT_NEAR(-expected_speed, solution.modules[0].wheel_angular_velocity, 1e-6);
  EXPECT_NEAR(expected_error, solution.modules[1].steer_error, 1e-6);
  EXPECT_NEAR(expected_speed, solution.modules[1].wheel_angular_velocity, 1e-6);
  EXPECT_NEAR(expected_error, solution.modules[2].steer_error, 1e-6);
  EXPECT_NEAR(-expected_speed, solution.modules[2].wheel_angular_velocity, 1e-6);
  EXPECT_NEAR(-expected_error, solution.modules[3].steer_error, 1e-6);
  EXPECT_NEAR(expected_speed, solution.modules[3].wheel_angular_velocity, 1e-6);
}

TEST(Kinematics, ComputeWheelTargetsHonorsDirectionSigns)
{
  Kinematics::Geometry geometry;
  geometry.wheel_radius = 0.10;

  Kinematics::DirectionSigns direction_signs;
  direction_signs.vx = {{1, 1, -1, -1}};

  const Kinematics::ChassisTwist twist{1.0, 0.0, 0.0};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_zero_offsets{{0.0, 0.0, 0.0, 0.0}};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_positions{{0.0, 0.0, 0.0, 0.0}};

  Kinematics kinematics(geometry);
  kinematics.SetDirectionSigns(direction_signs);
  const auto solution =
      kinematics.ComputeWheelTargets(twist, steer_zero_offsets, steer_positions);

  EXPECT_NEAR(10.0, solution.modules[0].wheel_angular_velocity, 1e-6);
  EXPECT_NEAR(10.0, solution.modules[1].wheel_angular_velocity, 1e-6);
  EXPECT_NEAR(-10.0, solution.modules[2].wheel_angular_velocity, 1e-6);
  EXPECT_NEAR(-10.0, solution.modules[3].wheel_angular_velocity, 1e-6);
  EXPECT_NEAR(0.0, solution.modules[2].steer_error, 1e-9);
  EXPECT_NEAR(0.0, solution.modules[3].steer_error, 1e-9);
}

TEST(Kinematics, ForwardKinematicsRecoversKnownTwist)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.62;
  geometry.track_width = 0.46;
  geometry.wheel_radius = 0.09;

  const Kinematics::ChassisTwist expected_twist{0.8, -0.3, 1.2};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_positions{{0.35, -0.4, 2.5, -2.1}};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_zero_offsets{{0.0, 0.0, 0.0, 0.0}};
  const std::array<int, Kinematics::WHEEL_COUNT> wheel_rolling_signs{{1, -1, -1, 1}};

  Kinematics kinematics(geometry);
  const auto feedback = build_feedback_from_twist(
      geometry, expected_twist, steer_positions, steer_zero_offsets, wheel_rolling_signs);

  Kinematics::ChassisTwist solved_twist;
  ASSERT_TRUE(kinematics.ComputeChassisTwistFromWheelFeedback(
      feedback, steer_zero_offsets, wheel_rolling_signs, &solved_twist));
  EXPECT_NEAR(expected_twist.vx, solved_twist.vx, 1e-6);
  EXPECT_NEAR(expected_twist.vy, solved_twist.vy, 1e-6);
  EXPECT_NEAR(expected_twist.wz, solved_twist.wz, 1e-6);
}

TEST(Kinematics, ForwardKinematicsHonorsRollingSigns)
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.62;
  geometry.track_width = 0.46;
  geometry.wheel_radius = 0.09;

  const Kinematics::ChassisTwist expected_twist{0.8, -0.3, 1.2};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_positions{{0.35, -0.4, 2.5, -2.1}};
  const std::array<double, Kinematics::WHEEL_COUNT> steer_zero_offsets{{0.0, 0.0, 0.0, 0.0}};
  const std::array<int, Kinematics::WHEEL_COUNT> correct_signs{{1, -1, -1, 1}};
  const std::array<int, Kinematics::WHEEL_COUNT> wrong_signs{{1, 1, 1, 1}};

  Kinematics kinematics(geometry);
  const auto feedback = build_feedback_from_twist(
      geometry, expected_twist, steer_positions, steer_zero_offsets, correct_signs);

  Kinematics::ChassisTwist solved_correct;
  ASSERT_TRUE(kinematics.ComputeChassisTwistFromWheelFeedback(
      feedback, steer_zero_offsets, correct_signs, &solved_correct));
  EXPECT_NEAR(expected_twist.vx, solved_correct.vx, 1e-6);
  EXPECT_NEAR(expected_twist.vy, solved_correct.vy, 1e-6);
  EXPECT_NEAR(expected_twist.wz, solved_correct.wz, 1e-6);

  Kinematics::ChassisTwist solved_wrong;
  ASSERT_TRUE(kinematics.ComputeChassisTwistFromWheelFeedback(
      feedback, steer_zero_offsets, wrong_signs, &solved_wrong));
  EXPECT_GT(std::fabs(solved_wrong.vx - expected_twist.vx), 0.1);
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

  const std::array<double, Kinematics::WHEEL_COUNT> steer_zero_offsets{{0.0, 0.0, 0.0, 0.0}};
  const std::array<int, Kinematics::WHEEL_COUNT> rolling_signs{{1, 1, 1, 1}};

  Kinematics kinematics(geometry);
  Kinematics::ChassisTwist solved;
  EXPECT_FALSE(
      kinematics.ComputeChassisTwistFromWheelFeedback(feedback, steer_zero_offsets,
                                                      rolling_signs, &solved));
}
