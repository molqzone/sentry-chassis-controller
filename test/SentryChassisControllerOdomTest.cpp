#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <string>

#include "sentry_chassis_controller/sentry_chassis_controller.h"

namespace sentry_chassis_controller
{
namespace
{
constexpr double K_PI = 3.14159265358979323846;

geometry_msgs::TransformStamped build_transform(const std::string& target_frame,
                                                const std::string& source_frame,
                                                double yaw_radians)
{
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = target_frame;
  transform.child_frame_id = source_frame;
  transform.transform.translation.x = 3.0;
  transform.transform.translation.y = -2.0;
  transform.transform.translation.z = 0.5;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw_radians);
  quaternion.normalize();
  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();
  return transform;
}
}  // namespace

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

TEST(SentryChassisControllerOdom, TransformTwistWithTransformRotatesGlobalToBase)
{
  const Kinematics::ChassisTwist INPUT{1.0, 0.0, 0.6};
  const auto TRANSFORM = build_transform("base_link", "odom", -K_PI * 0.5);

  Kinematics::ChassisTwist output;
  ASSERT_TRUE(
      SentryChassisController::TransformTwistWithTransform(INPUT, TRANSFORM, &output));
  EXPECT_NEAR(0.0, output.vx, 1e-9);
  EXPECT_NEAR(-1.0, output.vy, 1e-9);
  EXPECT_NEAR(0.6, output.wz, 1e-9);
}

TEST(SentryChassisControllerOdom, TransformTwistWithTransformRejectsNullOutput)
{
  const Kinematics::ChassisTwist INPUT{0.5, -0.2, 0.3};
  const auto TRANSFORM = build_transform("base_link", "odom", 0.0);
  EXPECT_FALSE(
      SentryChassisController::TransformTwistWithTransform(INPUT, TRANSFORM, nullptr));
}

}  // namespace sentry_chassis_controller
