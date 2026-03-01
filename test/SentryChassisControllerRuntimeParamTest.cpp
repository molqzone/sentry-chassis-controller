#include <gtest/gtest.h>
#include <ros/time.h>

#include "sentry_chassis_controller/sentry_chassis_controller.h"

namespace sentry_chassis_controller
{
constexpr double K_MIN_WHEEL_RADIUS = 1e-9;

void EnsureRosTimeInitialized()
{
  if (!ros::Time::isValid())
  {
    ros::Time::init();
  }
}

class SentryChassisControllerRuntimeParamsTestAccessor
{
 public:
  static bool ValidateAndApply(SentryChassisController* controller, bool strict_validation)
  {
    return controller->ValidateAndApplyControllerParams(strict_validation);
  }

  static const SentryChassisController::RuntimeParams* ReadRuntimeParams(
      SentryChassisController* controller)
  {
    return controller->runtime_params_buffer_.readFromRT();
  }

  static void SetModeText(SentryChassisController* controller, const std::string& mode_text)
  {
    controller->command_velocity_mode_text_ = mode_text;
  }

  static void SetCommandFrameId(SentryChassisController* controller,
                                const std::string& command_frame_id)
  {
    controller->command_frame_id_ = command_frame_id;
  }

  static void SetBaseFrameId(SentryChassisController* controller,
                             const std::string& base_frame_id)
  {
    controller->base_frame_id_ = base_frame_id;
  }

  static void SetOdomFrameId(SentryChassisController* controller,
                             const std::string& odom_frame_id)
  {
    controller->odom_frame_id_ = odom_frame_id;
  }

  static void SetCmdVelTimeout(SentryChassisController* controller, double timeout)
  {
    controller->cmd_vel_timeout_ = timeout;
  }

  static void SetOdomStartupHold(SentryChassisController* controller, double hold_seconds)
  {
    controller->odom_startup_hold_sec_ = hold_seconds;
  }

  static void SetOdomMaxLinearSpeed(SentryChassisController* controller, double max_speed)
  {
    controller->odom_max_linear_speed_ = max_speed;
  }

  static void SetOdomMaxAngularSpeed(SentryChassisController* controller, double max_speed)
  {
    controller->odom_max_angular_speed_ = max_speed;
  }

  static void SetWheelEffortLimit(SentryChassisController* controller, double effort_limit)
  {
    controller->wheel_effort_limit_ = effort_limit;
  }

  static void SetReverseCcwVxScale(SentryChassisController* controller, double vx_scale)
  {
    controller->reverse_ccw_vx_scale_ = vx_scale;
  }

  static void SetReverseCcwWzGain(SentryChassisController* controller, double wz_gain)
  {
    controller->reverse_ccw_wz_gain_ = wz_gain;
  }

  static void SetReverseCcwVyThreshold(SentryChassisController* controller,
                                       double vy_threshold)
  {
    controller->reverse_ccw_vy_threshold_ = vy_threshold;
  }

  static void SetReverseCcwSteerPriorityError(
      SentryChassisController* controller, double steer_priority_error)
  {
    controller->reverse_ccw_steer_priority_error_ = steer_priority_error;
  }

  static void SetWheelRadius(SentryChassisController* controller, double wheel_radius)
  {
    controller->geometry_.wheel_radius = wheel_radius;
  }

  static void SetCommandVelocityMode(SentryChassisController* controller,
                                     SentryChassisController::CommandVelocityMode mode)
  {
    controller->command_velocity_mode_ = mode;
  }

  static const std::string& GetModeText(const SentryChassisController* controller)
  {
    return controller->command_velocity_mode_text_;
  }

  static void SetOdomState(SentryChassisController* controller, double x, double y,
                           double yaw)
  {
    controller->odom_state_.x = x;
    controller->odom_state_.y = y;
    controller->odom_state_.yaw = yaw;
  }

  static const SentryChassisController::OdomState& GetOdomState(
      const SentryChassisController* controller)
  {
    return controller->odom_state_;
  }

  static void SetAppliedFrames(SentryChassisController* controller,
                               const std::string& odom_frame_id,
                               const std::string& base_frame_id)
  {
    controller->applied_odom_frame_id_ = odom_frame_id;
    controller->applied_base_frame_id_ = base_frame_id;
  }

  static const std::string& GetAppliedOdomFrameId(
      const SentryChassisController* controller)
  {
    return controller->applied_odom_frame_id_;
  }

  static const std::string& GetAppliedBaseFrameId(
      const SentryChassisController* controller)
  {
    return controller->applied_base_frame_id_;
  }

  static void ApplyRuntimeParamsInUpdate(
      SentryChassisController* controller,
      const SentryChassisController::RuntimeParams& runtime_params)
  {
    controller->ApplyRuntimeParamsInUpdate(runtime_params);
  }

  static void ApplyRuntimeParamsInUpdateWithFrames(
      SentryChassisController* controller,
      const std::string& odom_frame_id,
      const std::string& base_frame_id)
  {
    SentryChassisController::RuntimeParams runtime_params;
    runtime_params.odom_frame_id = odom_frame_id;
    runtime_params.base_frame_id = base_frame_id;
    controller->ApplyRuntimeParamsInUpdate(runtime_params);
  }
};

TEST(SentryChassisControllerRuntimeParams,
     ValidateAndApplyControllerParamsClampsAndPublishesSnapshot)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  SentryChassisControllerRuntimeParamsTestAccessor::SetModeText(&controller, "base_link");
  SentryChassisControllerRuntimeParamsTestAccessor::SetCommandFrameId(&controller, "odom");
  SentryChassisControllerRuntimeParamsTestAccessor::SetBaseFrameId(&controller, "base_link");
  SentryChassisControllerRuntimeParamsTestAccessor::SetCmdVelTimeout(&controller, -1.0);
  SentryChassisControllerRuntimeParamsTestAccessor::SetOdomStartupHold(&controller, -2.0);
  SentryChassisControllerRuntimeParamsTestAccessor::SetOdomMaxLinearSpeed(&controller, 0.0);
  SentryChassisControllerRuntimeParamsTestAccessor::SetOdomMaxAngularSpeed(&controller, -5.0);
  SentryChassisControllerRuntimeParamsTestAccessor::SetWheelEffortLimit(&controller, 0.0);
  SentryChassisControllerRuntimeParamsTestAccessor::SetReverseCcwVxScale(&controller, 0.0);
  SentryChassisControllerRuntimeParamsTestAccessor::SetReverseCcwWzGain(&controller, 0.5);
  SentryChassisControllerRuntimeParamsTestAccessor::SetReverseCcwVyThreshold(&controller,
                                                                              -0.1);
  SentryChassisControllerRuntimeParamsTestAccessor::SetReverseCcwSteerPriorityError(
      &controller, -0.2);
  SentryChassisControllerRuntimeParamsTestAccessor::SetWheelRadius(&controller, 0.0);

  ASSERT_TRUE(
      SentryChassisControllerRuntimeParamsTestAccessor::ValidateAndApply(&controller, false));
  const auto* runtime_params =
      SentryChassisControllerRuntimeParamsTestAccessor::ReadRuntimeParams(&controller);
  ASSERT_NE(runtime_params, nullptr);
  EXPECT_DOUBLE_EQ(0.0, runtime_params->cmd_vel_timeout);
  EXPECT_DOUBLE_EQ(0.0, runtime_params->odom_startup_hold_sec);
  EXPECT_DOUBLE_EQ(8.0, runtime_params->odom_max_linear_speed);
  EXPECT_DOUBLE_EQ(16.0, runtime_params->odom_max_angular_speed);
  EXPECT_DOUBLE_EQ(12.0, runtime_params->wheel_effort_limit);
  EXPECT_DOUBLE_EQ(0.1, runtime_params->reverse_ccw_vx_scale);
  EXPECT_DOUBLE_EQ(1.0, runtime_params->reverse_ccw_wz_gain);
  EXPECT_DOUBLE_EQ(0.0, runtime_params->reverse_ccw_vy_threshold);
  EXPECT_DOUBLE_EQ(0.0, runtime_params->reverse_ccw_steer_priority_error);
  EXPECT_GE(runtime_params->geometry.wheel_radius, K_MIN_WHEEL_RADIUS);
  EXPECT_EQ("base_link", runtime_params->command_frame_id);
}

TEST(SentryChassisControllerRuntimeParams,
     ValidateAndApplyControllerParamsRejectsInvalidModeInStrictValidation)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  SentryChassisControllerRuntimeParamsTestAccessor::SetModeText(&controller, "invalid_mode");
  EXPECT_FALSE(
      SentryChassisControllerRuntimeParamsTestAccessor::ValidateAndApply(&controller, true));
}

TEST(SentryChassisControllerRuntimeParams,
     ValidateAndApplyControllerParamsKeepsPreviousModeWhenDynamicInputIsInvalid)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  SentryChassisControllerRuntimeParamsTestAccessor::SetCommandVelocityMode(
      &controller, SentryChassisController::CommandVelocityMode::GLOBAL);
  SentryChassisControllerRuntimeParamsTestAccessor::SetModeText(&controller, "invalid_mode");
  SentryChassisControllerRuntimeParamsTestAccessor::SetBaseFrameId(&controller, "base_link");
  SentryChassisControllerRuntimeParamsTestAccessor::SetCommandFrameId(&controller, "odom");

  ASSERT_TRUE(
      SentryChassisControllerRuntimeParamsTestAccessor::ValidateAndApply(&controller, false));
  const auto* runtime_params =
      SentryChassisControllerRuntimeParamsTestAccessor::ReadRuntimeParams(&controller);
  ASSERT_NE(runtime_params, nullptr);
  EXPECT_EQ(SentryChassisController::CommandVelocityMode::GLOBAL,
            runtime_params->command_velocity_mode);
  EXPECT_EQ("global",
            SentryChassisControllerRuntimeParamsTestAccessor::GetModeText(&controller));
}

TEST(SentryChassisControllerRuntimeParams,
     ValidateAndApplyControllerParamsSanitizesEmptyFrameIds)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  SentryChassisControllerRuntimeParamsTestAccessor::SetModeText(&controller, "global");
  SentryChassisControllerRuntimeParamsTestAccessor::SetBaseFrameId(&controller, "");
  SentryChassisControllerRuntimeParamsTestAccessor::SetOdomFrameId(&controller, "");
  SentryChassisControllerRuntimeParamsTestAccessor::SetCommandFrameId(&controller, "");

  ASSERT_TRUE(
      SentryChassisControllerRuntimeParamsTestAccessor::ValidateAndApply(&controller, false));
  const auto* runtime_params =
      SentryChassisControllerRuntimeParamsTestAccessor::ReadRuntimeParams(&controller);
  ASSERT_NE(runtime_params, nullptr);
  EXPECT_EQ("base_link", runtime_params->base_frame_id);
  EXPECT_EQ("odom", runtime_params->odom_frame_id);
  EXPECT_EQ("odom", runtime_params->command_frame_id);
}

TEST(SentryChassisControllerRuntimeParams,
     ApplyRuntimeParamsInUpdateResetsOdometryWhenFrameIdsChange)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  SentryChassisControllerRuntimeParamsTestAccessor::SetOdomState(&controller, 1.5, -0.4, 0.8);
  SentryChassisControllerRuntimeParamsTestAccessor::SetAppliedFrames(
      &controller, "odom_old", "base_old");

  SentryChassisControllerRuntimeParamsTestAccessor::ApplyRuntimeParamsInUpdateWithFrames(
      &controller, "odom_new", "base_new");
  const auto& odom_state =
      SentryChassisControllerRuntimeParamsTestAccessor::GetOdomState(&controller);
  EXPECT_DOUBLE_EQ(0.0, odom_state.x);
  EXPECT_DOUBLE_EQ(0.0, odom_state.y);
  EXPECT_DOUBLE_EQ(0.0, odom_state.yaw);
  EXPECT_EQ("odom_new",
            SentryChassisControllerRuntimeParamsTestAccessor::GetAppliedOdomFrameId(
                &controller));
  EXPECT_EQ("base_new",
            SentryChassisControllerRuntimeParamsTestAccessor::GetAppliedBaseFrameId(
                &controller));
}

}  // namespace sentry_chassis_controller
