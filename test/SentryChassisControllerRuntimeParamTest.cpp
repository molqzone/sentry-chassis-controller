#include <gtest/gtest.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/time.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

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

struct UpdateJointStorage
{
  std::array<double, SentryChassisController::WHEEL_COUNT> steer_position{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, SentryChassisController::WHEEL_COUNT> steer_velocity{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, SentryChassisController::WHEEL_COUNT> steer_effort{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, SentryChassisController::WHEEL_COUNT> steer_command{{0.0, 0.0, 0.0, 0.0}};

  std::array<double, SentryChassisController::WHEEL_COUNT> wheel_position{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, SentryChassisController::WHEEL_COUNT> wheel_velocity{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, SentryChassisController::WHEEL_COUNT> wheel_effort{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, SentryChassisController::WHEEL_COUNT> wheel_command{{0.0, 0.0, 0.0, 0.0}};
};

Kinematics::Geometry MakeValidTestGeometry()
{
  Kinematics::Geometry geometry;
  geometry.wheel_base = 0.4;
  geometry.track_width = 0.3;
  geometry.wheel_radius = 0.1;
  return geometry;
}

std::array<double, SentryChassisController::WHEEL_COUNT>
MakeNonSingularSteerPositions()
{
  return {{0.35, -0.4, 2.5, -2.1}};
}

std::array<double, SentryChassisController::WHEEL_COUNT> BuildWheelVelocityFromTwist(
    const Kinematics::Geometry& geometry,
    const std::array<double, SentryChassisController::WHEEL_COUNT>& steer_positions,
    const Kinematics::ChassisTwist& twist)
{
  const double HALF_WHEEL_BASE = geometry.wheel_base * 0.5;
  const double HALF_TRACK_WIDTH = geometry.track_width * 0.5;
  const std::array<std::pair<double, double>, SentryChassisController::WHEEL_COUNT>
      MODULE_POSITIONS = {{
          {HALF_WHEEL_BASE, HALF_TRACK_WIDTH},
          {HALF_WHEEL_BASE, -HALF_TRACK_WIDTH},
          {-HALF_WHEEL_BASE, HALF_TRACK_WIDTH},
          {-HALF_WHEEL_BASE, -HALF_TRACK_WIDTH},
      }};

  std::array<double, SentryChassisController::WHEEL_COUNT> wheel_velocity{};
  for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
  {
    const double THETA = steer_positions[i];
    const double DIRECTION_X = std::cos(THETA);
    const double DIRECTION_Y = std::sin(THETA);
    const double MODULE_X = MODULE_POSITIONS[i].first;
    const double MODULE_Y = MODULE_POSITIONS[i].second;
    const double WHEEL_LINEAR_SPEED =
        DIRECTION_X * twist.vx + DIRECTION_Y * twist.vy +
        (-DIRECTION_X * MODULE_Y + DIRECTION_Y * MODULE_X) * twist.wz;
    wheel_velocity[i] = WHEEL_LINEAR_SPEED / geometry.wheel_radius;
  }
  return wheel_velocity;
}

class SentryChassisControllerRuntimeParamsTestAccessor
{
 public:
  using RuntimeParams = SentryChassisController::RuntimeParams;

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

  static void SetEnableAccelerationLimits(SentryChassisController* controller, bool enabled)
  {
    controller->enable_acceleration_limits_ = enabled;
  }

  static void SetMaxLinearAcceleration(SentryChassisController* controller,
                                       double max_linear_acceleration)
  {
    controller->max_linear_acceleration_ = max_linear_acceleration;
  }

  static void SetMaxAngularAcceleration(SentryChassisController* controller,
                                        double max_angular_acceleration)
  {
    controller->max_angular_acceleration_ = max_angular_acceleration;
  }

  static void SetEnablePowerLimit(SentryChassisController* controller, bool enabled)
  {
    controller->enable_power_limit_ = enabled;
  }

  static void SetEnablePowerLimitLogging(SentryChassisController* controller, bool enabled)
  {
    controller->enable_power_limit_logging_ = enabled;
  }

  static void SetMaxPower(SentryChassisController* controller, double max_power)
  {
    controller->max_power_ = max_power;
  }

  static void SetPowerLossK1(SentryChassisController* controller, double power_loss_k1)
  {
    controller->power_loss_k1_ = power_loss_k1;
  }

  static void SetPowerLossK2(SentryChassisController* controller, double power_loss_k2)
  {
    controller->power_loss_k2_ = power_loss_k2;
  }

  static void SetMinPowerScale(SentryChassisController* controller, double min_power_scale)
  {
    controller->min_power_scale_ = min_power_scale;
  }

  static void SetOdomIntegrateOnTimeout(SentryChassisController* controller, bool enabled)
  {
    controller->odom_integrate_on_timeout_ = enabled;
  }

  static void SetPublishTf(SentryChassisController* controller, bool enabled)
  {
    controller->publish_tf_ = enabled;
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

  static Kinematics::ChassisTwist ApplyAccelerationLimits(
      SentryChassisController* controller, const Kinematics::ChassisTwist& input, double dt,
      bool enable, double max_linear_acceleration, double max_angular_acceleration)
  {
    SentryChassisController::RuntimeParams runtime_params;
    runtime_params.enable_acceleration_limits = enable;
    runtime_params.max_linear_acceleration = max_linear_acceleration;
    runtime_params.max_angular_acceleration = max_angular_acceleration;
    Kinematics::ChassisTwist output;
    controller->ApplyAccelerationLimits(input, dt, runtime_params, &output);
    return output;
  }

  static void ResetAccelerationLimiterState(SentryChassisController* controller)
  {
    controller->last_limited_command_ = Kinematics::ChassisTwist();
    controller->has_last_limited_command_ = false;
  }

  static void SetAccelerationLimiterState(
      SentryChassisController* controller,
      const Kinematics::ChassisTwist& last_limited_command)
  {
    controller->last_limited_command_ = last_limited_command;
    controller->has_last_limited_command_ = true;
  }

  static bool HasLastLimitedCommand(const SentryChassisController* controller)
  {
    return controller->has_last_limited_command_;
  }

  static void SetControllerStartTime(SentryChassisController* controller,
                                     const ros::Time& start_time)
  {
    controller->controller_start_time_ = start_time;
  }

  static bool GetLastCommandTimedOut(const SentryChassisController* controller)
  {
    return controller->last_command_timed_out_;
  }

  static void ConfigureUpdateJointHandles(SentryChassisController* controller,
                                          UpdateJointStorage* storage)
  {
    ASSERT_NE(storage, nullptr);
    for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
    {
      const std::string STEER_NAME = "test_steer_joint_" + std::to_string(i);
      const std::string WHEEL_NAME = "test_wheel_joint_" + std::to_string(i);
      hardware_interface::JointStateHandle steer_state(
          STEER_NAME, &storage->steer_position[i], &storage->steer_velocity[i],
          &storage->steer_effort[i]);
      hardware_interface::JointStateHandle wheel_state(
          WHEEL_NAME, &storage->wheel_position[i], &storage->wheel_velocity[i],
          &storage->wheel_effort[i]);
      controller->steer_joints_[i] =
          hardware_interface::JointHandle(steer_state, &storage->steer_command[i]);
      controller->wheel_joints_[i] =
          hardware_interface::JointHandle(wheel_state, &storage->wheel_command[i]);
    }
  }

  static void ConfigureUpdateLoopDependencies(SentryChassisController* controller)
  {
    ASSERT_NE(controller, nullptr);
    controller->direction_signs_ = Kinematics::DirectionSigns();
    controller->wheel_rolling_signs_ = {{1, 1, 1, 1}};
    controller->steer_zero_offsets_ = {{0.0, 0.0, 0.0, 0.0}};
    controller->command_compensation_matrix_ = {
        {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
    controller->kinematics_.SetDirectionSigns(controller->direction_signs_);
    controller->applied_geometry_ = Kinematics::Geometry();
    controller->applied_odom_frame_id_ = "odom";
    controller->applied_base_frame_id_ = "base_link";
    for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
    {
      controller->steer_pids_[i].initPid(1.0, 0.0, 0.0, 1e9, -1e9, false);
      controller->wheel_pids_[i].initPid(1.0, 0.0, 0.0, 1e9, -1e9, false);
    }
  }

  static void WriteRuntimeParamsSnapshot(
      SentryChassisController* controller,
      const SentryChassisController::RuntimeParams& runtime_params)
  {
    controller->runtime_params_shadow_ = runtime_params;
    controller->runtime_params_buffer_.initRT(runtime_params);
    controller->runtime_params_buffer_.writeFromNonRT(runtime_params);
  }

  static void WriteCommandSnapshot(SentryChassisController* controller, double vx, double vy,
                                   double wz, const ros::Time& stamp, bool valid)
  {
    SentryChassisController::CommandData command;
    command.vx = vx;
    command.vy = vy;
    command.wz = wz;
    command.stamp = stamp;
    command.valid = valid;
    controller->command_buffer_.initRT(command);
    controller->command_buffer_.writeFromNonRT(command);
  }

  static void RunUpdate(SentryChassisController* controller, const ros::Time& time, double dt)
  {
    controller->update(time, ros::Duration(dt));
  }

  static void SetCommandTransformCache(
      SentryChassisController* controller, const std::array<double, 9>& rotation_matrix,
      bool valid)
  {
    SentryChassisController::CommandTransformCache cache;
    cache.rotation_matrix_row_major = rotation_matrix;
    cache.valid = valid;
    controller->command_transform_buffer_.writeFromNonRT(cache);
  }

  static bool ResolveCommandInBaseFrame(
      SentryChassisController* controller, SentryChassisController::CommandVelocityMode mode,
      const std::string& command_frame_id, const std::string& base_frame_id,
      const ros::Time& time, double vx, double vy, double wz,
      Kinematics::ChassisTwist* output)
  {
    SentryChassisController::CommandData command;
    command.vx = vx;
    command.vy = vy;
    command.wz = wz;
    command.stamp = time;
    command.valid = true;

    SentryChassisController::RuntimeParams runtime_params;
    runtime_params.command_velocity_mode = mode;
    runtime_params.command_frame_id = command_frame_id;
    runtime_params.base_frame_id = base_frame_id;
    return controller->ResolveCommandInBaseFrame(command, time, runtime_params, output);
  }

  static bool PrepareCommandForControl(
      SentryChassisController* controller,
      const SentryChassisController::RuntimeParams& runtime_params, const ros::Time& now,
      double dt, bool command_valid, const ros::Time& command_stamp, double vx, double vy,
      double wz, Kinematics::ChassisTwist* output, bool* timeout)
  {
    SentryChassisController::CommandData command;
    command.vx = vx;
    command.vy = vy;
    command.wz = wz;
    command.valid = command_valid;
    command.stamp = command_stamp;
    return controller->PrepareCommandForControl(command, now, dt, runtime_params, output,
                                                timeout);
  }

  static bool PrepareCommandForControlBaseLinkMode(
      SentryChassisController* controller, const ros::Time& now, double dt,
      bool command_valid, const ros::Time& command_stamp, double vx, double vy, double wz,
      Kinematics::ChassisTwist* output, bool* timeout)
  {
    SentryChassisController::RuntimeParams runtime_params;
    runtime_params.command_velocity_mode =
        SentryChassisController::CommandVelocityMode::BASE_LINK;
    return PrepareCommandForControl(controller, runtime_params, now, dt, command_valid,
                                    command_stamp, vx, vy, wz, output, timeout);
  }

  static std::array<double, 4> ApplyPowerLimiting(
      SentryChassisController* controller, const std::array<double, 4>& signed_wheel_velocities,
      const std::array<double, 4>& input_signed_wheel_efforts, bool enable_power_limit,
      double max_power, double power_loss_k1, double power_loss_k2, double min_power_scale)
  {
    SentryChassisController::RuntimeParams runtime_params;
    runtime_params.enable_power_limit = enable_power_limit;
    runtime_params.max_power = max_power;
    runtime_params.power_loss_k1 = power_loss_k1;
    runtime_params.power_loss_k2 = power_loss_k2;
    runtime_params.min_power_scale = min_power_scale;
    std::array<double, 4> efforts = input_signed_wheel_efforts;
    controller->ApplyPowerLimiting(runtime_params, signed_wheel_velocities, &efforts);
    return efforts;
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
  SentryChassisControllerRuntimeParamsTestAccessor::SetEnableAccelerationLimits(
      &controller, true);
  SentryChassisControllerRuntimeParamsTestAccessor::SetMaxLinearAcceleration(&controller,
                                                                              0.0);
  SentryChassisControllerRuntimeParamsTestAccessor::SetMaxAngularAcceleration(&controller,
                                                                               -5.0);
  SentryChassisControllerRuntimeParamsTestAccessor::SetEnablePowerLimit(&controller, true);
  SentryChassisControllerRuntimeParamsTestAccessor::SetEnablePowerLimitLogging(&controller,
                                                                                true);
  SentryChassisControllerRuntimeParamsTestAccessor::SetMaxPower(&controller, 0.0);
  SentryChassisControllerRuntimeParamsTestAccessor::SetPowerLossK1(&controller, -0.1);
  SentryChassisControllerRuntimeParamsTestAccessor::SetPowerLossK2(&controller, -0.2);
  SentryChassisControllerRuntimeParamsTestAccessor::SetMinPowerScale(&controller, 1.2);
  SentryChassisControllerRuntimeParamsTestAccessor::SetOdomIntegrateOnTimeout(&controller,
                                                                               true);
  SentryChassisControllerRuntimeParamsTestAccessor::SetPublishTf(&controller, false);
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
  EXPECT_TRUE(runtime_params->odom_integrate_on_timeout);
  EXPECT_FALSE(runtime_params->publish_tf);
  EXPECT_DOUBLE_EQ(12.0, runtime_params->wheel_effort_limit);
  EXPECT_DOUBLE_EQ(0.1, runtime_params->reverse_ccw_vx_scale);
  EXPECT_DOUBLE_EQ(1.0, runtime_params->reverse_ccw_wz_gain);
  EXPECT_DOUBLE_EQ(0.0, runtime_params->reverse_ccw_vy_threshold);
  EXPECT_DOUBLE_EQ(0.0, runtime_params->reverse_ccw_steer_priority_error);
  EXPECT_TRUE(runtime_params->enable_acceleration_limits);
  EXPECT_DOUBLE_EQ(3.0, runtime_params->max_linear_acceleration);
  EXPECT_DOUBLE_EQ(5.0, runtime_params->max_angular_acceleration);
  EXPECT_TRUE(runtime_params->enable_power_limit);
  EXPECT_TRUE(runtime_params->enable_power_limit_logging);
  EXPECT_DOUBLE_EQ(360.0, runtime_params->max_power);
  EXPECT_DOUBLE_EQ(0.001, runtime_params->power_loss_k1);
  EXPECT_DOUBLE_EQ(0.0001, runtime_params->power_loss_k2);
  EXPECT_DOUBLE_EQ(1.0, runtime_params->min_power_scale);
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

TEST(SentryChassisControllerRuntimeParams, ApplyAccelerationLimitsBoundsDeltaPerCycle)
{
  SentryChassisController controller;
  SentryChassisControllerRuntimeParamsTestAccessor::ResetAccelerationLimiterState(
      &controller);

  Kinematics::ChassisTwist first_target;
  first_target.vx = 1.0;
  first_target.vy = 0.0;
  first_target.wz = 1.0;
  const auto first_output =
      SentryChassisControllerRuntimeParamsTestAccessor::ApplyAccelerationLimits(
          &controller, first_target, 0.1, true, 1.0, 2.0);
  EXPECT_DOUBLE_EQ(1.0, first_output.vx);
  EXPECT_DOUBLE_EQ(0.0, first_output.vy);
  EXPECT_DOUBLE_EQ(1.0, first_output.wz);

  Kinematics::ChassisTwist second_target;
  second_target.vx = 2.0;
  second_target.vy = 0.0;
  second_target.wz = 2.0;
  const auto second_output =
      SentryChassisControllerRuntimeParamsTestAccessor::ApplyAccelerationLimits(
          &controller, second_target, 0.1, true, 1.0, 2.0);
  EXPECT_NEAR(1.1, second_output.vx, 1e-9);
  EXPECT_NEAR(0.0, second_output.vy, 1e-9);
  EXPECT_NEAR(1.2, second_output.wz, 1e-9);
}

TEST(SentryChassisControllerRuntimeParams,
     ResolveCommandInBaseFrameUsesInputInBaseLinkMode)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  Kinematics::ChassisTwist resolved;
  ASSERT_TRUE(SentryChassisControllerRuntimeParamsTestAccessor::ResolveCommandInBaseFrame(
      &controller, SentryChassisController::CommandVelocityMode::BASE_LINK, "base_link",
      "base_link", ros::Time(1.0), 1.2, -0.4, 0.7, &resolved));
  EXPECT_DOUBLE_EQ(1.2, resolved.vx);
  EXPECT_DOUBLE_EQ(-0.4, resolved.vy);
  EXPECT_DOUBLE_EQ(0.7, resolved.wz);
}

TEST(SentryChassisControllerRuntimeParams,
     ResolveCommandInBaseFrameGlobalModeFailsWithoutTransformCache)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  Kinematics::ChassisTwist resolved;
  EXPECT_FALSE(SentryChassisControllerRuntimeParamsTestAccessor::ResolveCommandInBaseFrame(
      &controller, SentryChassisController::CommandVelocityMode::GLOBAL, "odom",
      "base_link", ros::Time(1.0), 0.5, 0.1, -0.2, &resolved));
}

TEST(SentryChassisControllerRuntimeParams,
     ResolveCommandInBaseFrameGlobalModeUsesCachedTransform)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  const std::array<double, 9> quarter_turn_rotation = {
      {0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
  SentryChassisControllerRuntimeParamsTestAccessor::SetCommandTransformCache(
      &controller, quarter_turn_rotation, true);

  Kinematics::ChassisTwist resolved;
  ASSERT_TRUE(SentryChassisControllerRuntimeParamsTestAccessor::ResolveCommandInBaseFrame(
      &controller, SentryChassisController::CommandVelocityMode::GLOBAL, "odom",
      "base_link", ros::Time(1.0), 1.0, 0.0, 0.3, &resolved));
  EXPECT_NEAR(0.0, resolved.vx, 1e-9);
  EXPECT_NEAR(1.0, resolved.vy, 1e-9);
  EXPECT_NEAR(0.3, resolved.wz, 1e-9);
}

TEST(SentryChassisControllerRuntimeParams,
     PrepareCommandForControlTimeoutClearsAccelerationLimiterState)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  SentryChassisControllerRuntimeParamsTestAccessor::SetControllerStartTime(
      &controller, ros::Time(10.0));

  Kinematics::ChassisTwist last_limited;
  last_limited.vx = 0.8;
  last_limited.vy = 0.1;
  last_limited.wz = 0.2;
  SentryChassisControllerRuntimeParamsTestAccessor::SetAccelerationLimiterState(
      &controller, last_limited);

  Kinematics::ChassisTwist output;
  bool timeout = false;
  ASSERT_TRUE(
      SentryChassisControllerRuntimeParamsTestAccessor::PrepareCommandForControlBaseLinkMode(
          &controller, ros::Time(10.1), 0.01, true, ros::Time(9.9), 1.0, 0.0, 0.0, &output,
          &timeout));
  EXPECT_TRUE(timeout);
  EXPECT_FALSE(
      SentryChassisControllerRuntimeParamsTestAccessor::HasLastLimitedCommand(&controller));
  EXPECT_DOUBLE_EQ(0.0, output.vx);
  EXPECT_DOUBLE_EQ(0.0, output.vy);
  EXPECT_DOUBLE_EQ(0.0, output.wz);
}

TEST(SentryChassisControllerRuntimeParams,
     UpdateMainPathWritesWheelCommandsForFreshBaseLinkCommand)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  UpdateJointStorage joint_storage;
  joint_storage.steer_position = MakeNonSingularSteerPositions();
  SentryChassisControllerRuntimeParamsTestAccessor::ConfigureUpdateJointHandles(
      &controller, &joint_storage);
  SentryChassisControllerRuntimeParamsTestAccessor::ConfigureUpdateLoopDependencies(
      &controller);

  SentryChassisControllerRuntimeParamsTestAccessor::RuntimeParams runtime_params;
  runtime_params.command_velocity_mode =
      SentryChassisController::CommandVelocityMode::BASE_LINK;
  runtime_params.command_frame_id = "base_link";
  runtime_params.base_frame_id = "base_link";
  runtime_params.odom_frame_id = "odom";
  runtime_params.cmd_vel_timeout = 0.5;
  runtime_params.publish_tf = false;
  runtime_params.odom_integrate_on_timeout = false;
  runtime_params.enable_acceleration_limits = false;
  runtime_params.enable_power_limit = false;
  runtime_params.geometry = MakeValidTestGeometry();
  SentryChassisControllerRuntimeParamsTestAccessor::WriteRuntimeParamsSnapshot(
      &controller, runtime_params);
  SentryChassisControllerRuntimeParamsTestAccessor::WriteCommandSnapshot(
      &controller, 0.6, 0.0, 0.0, ros::Time(10.1), true);
  SentryChassisControllerRuntimeParamsTestAccessor::SetControllerStartTime(
      &controller, ros::Time(10.0));

  SentryChassisControllerRuntimeParamsTestAccessor::RunUpdate(&controller, ros::Time(10.12),
                                                              0.02);
  EXPECT_FALSE(
      SentryChassisControllerRuntimeParamsTestAccessor::GetLastCommandTimedOut(&controller));
  const bool HAS_NONZERO_WHEEL_COMMAND = std::any_of(
      joint_storage.wheel_command.begin(), joint_storage.wheel_command.end(),
      [](double command) { return std::fabs(command) > 1e-6; });
  EXPECT_TRUE(HAS_NONZERO_WHEEL_COMMAND);
}

TEST(SentryChassisControllerRuntimeParams,
     UpdateMainPathTimeoutZerosWheelCommandsAndSkipsOdomIntegration)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  UpdateJointStorage joint_storage;
  joint_storage.steer_position = MakeNonSingularSteerPositions();
  for (auto& velocity : joint_storage.wheel_velocity)
  {
    velocity = 5.0;
  }
  for (auto& command : joint_storage.wheel_command)
  {
    command = 1.0;
  }
  SentryChassisControllerRuntimeParamsTestAccessor::ConfigureUpdateJointHandles(
      &controller, &joint_storage);
  SentryChassisControllerRuntimeParamsTestAccessor::ConfigureUpdateLoopDependencies(
      &controller);

  SentryChassisControllerRuntimeParamsTestAccessor::RuntimeParams runtime_params;
  runtime_params.command_velocity_mode =
      SentryChassisController::CommandVelocityMode::BASE_LINK;
  runtime_params.command_frame_id = "base_link";
  runtime_params.base_frame_id = "base_link";
  runtime_params.odom_frame_id = "odom";
  runtime_params.cmd_vel_timeout = 0.1;
  runtime_params.publish_tf = false;
  runtime_params.odom_integrate_on_timeout = false;
  runtime_params.enable_acceleration_limits = false;
  runtime_params.enable_power_limit = false;
  runtime_params.geometry = MakeValidTestGeometry();
  SentryChassisControllerRuntimeParamsTestAccessor::WriteRuntimeParamsSnapshot(
      &controller, runtime_params);
  SentryChassisControllerRuntimeParamsTestAccessor::WriteCommandSnapshot(
      &controller, 0.5, 0.0, 0.0, ros::Time(9.0), true);
  SentryChassisControllerRuntimeParamsTestAccessor::SetControllerStartTime(
      &controller, ros::Time(10.0));

  SentryChassisControllerRuntimeParamsTestAccessor::RunUpdate(&controller, ros::Time(10.2),
                                                              0.02);
  EXPECT_TRUE(
      SentryChassisControllerRuntimeParamsTestAccessor::GetLastCommandTimedOut(&controller));
  for (const auto command : joint_storage.wheel_command)
  {
    EXPECT_DOUBLE_EQ(0.0, command);
  }
  const auto& odom_state =
      SentryChassisControllerRuntimeParamsTestAccessor::GetOdomState(&controller);
  EXPECT_DOUBLE_EQ(0.0, odom_state.x);
  EXPECT_DOUBLE_EQ(0.0, odom_state.y);
  EXPECT_DOUBLE_EQ(0.0, odom_state.yaw);
}

TEST(SentryChassisControllerRuntimeParams,
     UpdateMainPathIntegratesOdomFromWheelFeedbackWhenCommandIsFresh)
{
  EnsureRosTimeInitialized();
  SentryChassisController controller;
  UpdateJointStorage joint_storage;
  const auto TEST_GEOMETRY = MakeValidTestGeometry();
  const auto STEER_POSITIONS = MakeNonSingularSteerPositions();
  joint_storage.steer_position = STEER_POSITIONS;
  Kinematics::ChassisTwist expected_twist;
  expected_twist.vx = 0.8;
  expected_twist.vy = -0.3;
  expected_twist.wz = 1.2;
  joint_storage.wheel_velocity =
      BuildWheelVelocityFromTwist(TEST_GEOMETRY, STEER_POSITIONS, expected_twist);
  SentryChassisControllerRuntimeParamsTestAccessor::ConfigureUpdateJointHandles(
      &controller, &joint_storage);
  SentryChassisControllerRuntimeParamsTestAccessor::ConfigureUpdateLoopDependencies(
      &controller);

  SentryChassisControllerRuntimeParamsTestAccessor::RuntimeParams runtime_params;
  runtime_params.command_velocity_mode =
      SentryChassisController::CommandVelocityMode::BASE_LINK;
  runtime_params.command_frame_id = "base_link";
  runtime_params.base_frame_id = "base_link";
  runtime_params.odom_frame_id = "odom";
  runtime_params.cmd_vel_timeout = 0.5;
  runtime_params.publish_tf = false;
  runtime_params.odom_integrate_on_timeout = false;
  runtime_params.enable_acceleration_limits = false;
  runtime_params.enable_power_limit = false;
  runtime_params.geometry = TEST_GEOMETRY;
  SentryChassisControllerRuntimeParamsTestAccessor::WriteRuntimeParamsSnapshot(
      &controller, runtime_params);
  SentryChassisControllerRuntimeParamsTestAccessor::WriteCommandSnapshot(
      &controller, 0.0, 0.0, 0.0, ros::Time(10.1), true);
  SentryChassisControllerRuntimeParamsTestAccessor::SetControllerStartTime(
      &controller, ros::Time(10.0));

  SentryChassisControllerRuntimeParamsTestAccessor::RunUpdate(&controller, ros::Time(10.12),
                                                              0.05);
  EXPECT_FALSE(
      SentryChassisControllerRuntimeParamsTestAccessor::GetLastCommandTimedOut(&controller));
  const auto& odom_state =
      SentryChassisControllerRuntimeParamsTestAccessor::GetOdomState(&controller);
  EXPECT_GT(odom_state.x, 0.0);
}

TEST(SentryChassisControllerRuntimeParams, ApplyPowerLimitingScalesEffortsWhenOverBudget)
{
  SentryChassisController controller;
  const std::array<double, 4> wheel_velocities = {{10.0, 10.0, 10.0, 10.0}};
  const std::array<double, 4> wheel_efforts = {{2.0, 2.0, 2.0, 2.0}};
  const auto limited_efforts =
      SentryChassisControllerRuntimeParamsTestAccessor::ApplyPowerLimiting(
          &controller, wheel_velocities, wheel_efforts, true,
          40.0, 0.0, 0.0, 0.3);
  EXPECT_NEAR(1.0, limited_efforts[0], 1e-9);
  EXPECT_NEAR(1.0, limited_efforts[1], 1e-9);
  EXPECT_NEAR(1.0, limited_efforts[2], 1e-9);
  EXPECT_NEAR(1.0, limited_efforts[3], 1e-9);
}

}  // namespace sentry_chassis_controller
