#include <gtest/gtest.h>

#include <cmath>

#include "sentry_chassis_controller/sentry_chassis_controller.h"

namespace sentry_chassis_controller
{
namespace
{
constexpr double K_PI = 3.14159265358979323846;
}

TEST(SentryChassisControllerOdom, IntegratesStraightMotion)
{
  const SentryChassisController::OdomState START_STATE{0.0, 0.0, 0.0};
  const Kinematics::ChassisTwist TWIST{1.2, 0.0, 0.0};

  const auto INTEGRATED = SentryChassisController::IntegrateOdom(START_STATE, TWIST, 2.0);
  EXPECT_NEAR(2.4, INTEGRATED.x, 1e-9);
  EXPECT_NEAR(0.0, INTEGRATED.y, 1e-9);
  EXPECT_NEAR(0.0, INTEGRATED.yaw, 1e-9);
}

TEST(SentryChassisControllerOdom, IntegratesLateralAndYawMotion)
{
  const SentryChassisController::OdomState START_STATE{0.0, 0.0, 0.0};
  const Kinematics::ChassisTwist TWIST{0.0, 1.0, 1.0};
  const auto INTEGRATED = SentryChassisController::IntegrateOdom(START_STATE, TWIST, 1.0);

  EXPECT_NEAR(-std::sin(0.5), INTEGRATED.x, 1e-9);
  EXPECT_NEAR(std::cos(0.5), INTEGRATED.y, 1e-9);
  EXPECT_NEAR(1.0, INTEGRATED.yaw, 1e-9);
}

TEST(SentryChassisControllerOdom, NormalizesYawToPiRange)
{
  const SentryChassisController::OdomState START_STATE{0.0, 0.0, 3.1};
  const Kinematics::ChassisTwist TWIST{0.0, 0.0, 1.0};

  const auto INTEGRATED = SentryChassisController::IntegrateOdom(START_STATE, TWIST, 1.0);
  EXPECT_GE(INTEGRATED.yaw, -K_PI);
  EXPECT_LE(INTEGRATED.yaw, K_PI);
  EXPECT_NEAR(SentryChassisController::NormalizeAngle(4.1), INTEGRATED.yaw, 1e-9);
}

}  // namespace sentry_chassis_controller
