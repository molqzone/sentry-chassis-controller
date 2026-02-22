#include <control_toolbox/pid.h>
#include <gtest/gtest.h>
#include <ros/duration.h>
#include <ros/time.h>

#include "sentry_chassis_controller/sentry_chassis_controller.h"

namespace sentry_chassis_controller
{

TEST(SentryChassisControllerPid, WheelPidCommandBecomesNonZeroForVelocityError)
{
  control_toolbox::Pid wheel_pid;
  wheel_pid.initPid(2.0, 0.0, 0.0, 2.0, -2.0, false);

  const double EFFORT = wheel_pid.computeCommand(5.0, ros::Duration(0.01));
  EXPECT_GT(EFFORT, 0.0);
}

TEST(SentryChassisControllerPid, WheelPidCommandReturnsZeroForZeroError)
{
  control_toolbox::Pid wheel_pid;
  wheel_pid.initPid(2.0, 0.0, 0.0, 2.0, -2.0, false);
  wheel_pid.computeCommand(5.0, ros::Duration(0.01));

  const double EFFORT = wheel_pid.computeCommand(0.0, ros::Duration(0.01));
  EXPECT_NEAR(EFFORT, 0.0, 1e-9);
}

TEST(SentryChassisControllerPid, CommandTimeoutDetectsExpiredCommand)
{
  const bool TIMED_OUT = SentryChassisController::IsCommandTimedOut(
      true, ros::Time(10.0), ros::Time(10.4), 0.25);
  EXPECT_TRUE(TIMED_OUT);
}

TEST(SentryChassisControllerPid, CommandTimeoutAcceptsFreshCommand)
{
  const bool TIMED_OUT = SentryChassisController::IsCommandTimedOut(
      true, ros::Time(10.0), ros::Time(10.2), 0.25);
  EXPECT_FALSE(TIMED_OUT);
}

TEST(SentryChassisControllerPid, CommandTimeoutTreatsInvalidCommandAsTimedOut)
{
  const bool TIMED_OUT = SentryChassisController::IsCommandTimedOut(
      false, ros::Time(10.0), ros::Time(10.1), 0.25);
  EXPECT_TRUE(TIMED_OUT);
}

}  // namespace sentry_chassis_controller
