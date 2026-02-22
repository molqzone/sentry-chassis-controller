#include "sentry_chassis_controller/sentry_chassis_controller.h"

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <pluginlib/class_list_macros.h>

#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace sentry_chassis_controller
{
namespace
{
// 运行时最小轮半径保护值，避免除零。
// Runtime lower bound for wheel radius to avoid division by zero.
constexpr double kMinWheelRadius = 1e-9;
// 轮序后缀固定为前左、前右、后左、后右，需与配置文件和运动学保持一致。
// Wheel order suffix must stay aligned with config and kinematics.
const std::array<std::string, SentryChassisController::WHEEL_COUNT> kWheelNameSuffix = {
    "front_left", "front_right", "rear_left", "rear_right"};
}  // namespace

bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw,
                                   ros::NodeHandle& nh)
{
  std::vector<std::string> steer_joint_names;
  std::vector<std::string> wheel_joint_names;
  std::vector<double> steer_zero_offsets;

  if (!nh.getParam("steer_joints", steer_joint_names) ||
      steer_joint_names.size() != WHEEL_COUNT)
  {
    ROS_ERROR("Parameter 'steer_joints' must exist and contain exactly %zu items.",
              WHEEL_COUNT);
    return false;
  }

  if (!nh.getParam("wheel_joints", wheel_joint_names) ||
      wheel_joint_names.size() != WHEEL_COUNT)
  {
    ROS_ERROR("Parameter 'wheel_joints' must exist and contain exactly %zu items.",
              WHEEL_COUNT);
    return false;
  }

  if (!nh.getParam("steer_zero_offsets", steer_zero_offsets))
  {
    steer_zero_offsets.assign(WHEEL_COUNT, 0.0);
  }
  if (steer_zero_offsets.size() != WHEEL_COUNT)
  {
    ROS_ERROR("Parameter 'steer_zero_offsets' must contain exactly %zu items.",
              WHEEL_COUNT);
    return false;
  }
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    steer_zero_offsets_[i] = steer_zero_offsets[i];
  }

  Kinematics::Geometry geometry;
  nh.param("geometry/wheel_base", geometry.wheel_base, 0.50);
  nh.param("geometry/track_width", geometry.track_width, 0.40);
  nh.param("geometry/wheel_radius", geometry.wheel_radius, 0.076);
  if (geometry.wheel_radius <= kMinWheelRadius)
  {
    ROS_WARN("Parameter 'geometry/wheel_radius' must be positive. Clamping to %.9f.",
             kMinWheelRadius);
    geometry.wheel_radius = kMinWheelRadius;
  }
  kinematics_.SetGeometry(geometry);

  Kinematics::DirectionSigns direction_signs;
  if (!LoadDirectionSigns(nh, &direction_signs))
  {
    return false;
  }
  kinematics_.SetDirectionSigns(direction_signs);

  nh.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
  nh.param("command_frame_id", command_frame_id_, std::string("base_link"));
  nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.25);
  nh.param("enable_dynamic_reconfigure", enable_dynamic_reconfigure_, true);
  if (command_frame_id_ != "base_link")
  {
    ROS_WARN(
        "Parameter 'command_frame_id' is '%s', but only 'base_link' is supported in this "
        "stage. Falling back to 'base_link'.",
        command_frame_id_.c_str());
    command_frame_id_ = "base_link";
  }
  if (cmd_vel_timeout_ < 0.0)
  {
    ROS_WARN("Parameter 'cmd_vel_timeout' is negative. Clamping to 0.0.");
    cmd_vel_timeout_ = 0.0;
  }

  steer_joints_.clear();
  wheel_joints_.clear();
  steer_joints_.reserve(WHEEL_COUNT);
  wheel_joints_.reserve(WHEEL_COUNT);

  try
  {
    for (const auto& name : steer_joint_names)
    {
      steer_joints_.push_back(hw->getHandle(name));
    }
    for (const auto& name : wheel_joint_names)
    {
      wheel_joints_.push_back(hw->getHandle(name));
    }
  }
  catch (const hardware_interface::HardwareInterfaceException& exception)
  {
    ROS_ERROR("Failed to get joint handle: %s", exception.what());
    return false;
  }

  if (!InitPidGroup(nh, "steer", &steer_pids_))
  {
    return false;
  }
  if (!InitPidGroup(nh, "wheel", &wheel_pids_))
  {
    return false;
  }

  CommandData command;
  command.stamp = ros::Time(0);
  command.valid = false;
  command_buffer_.writeFromNonRT(command);

  cmd_vel_subscriber_ =
      nh.subscribe(cmd_vel_topic_, 1, &SentryChassisController::CmdVelCallback, this);

  ROS_INFO(
      "SentryChassisController initialized with cmd_vel_topic='%s', "
      "command_frame_id='%s', "
      "timeout=%.3fs.",
      cmd_vel_topic_.c_str(), command_frame_id_.c_str(), cmd_vel_timeout_);
  ROS_INFO(
      "wheel_direction_signs loaded: vx=[%d,%d,%d,%d], vy=[%d,%d,%d,%d], "
      "wz=[%d,%d,%d,%d].",
      direction_signs.vx[0], direction_signs.vx[1], direction_signs.vx[2],
      direction_signs.vx[3], direction_signs.vy[0], direction_signs.vy[1],
      direction_signs.vy[2], direction_signs.vy[3], direction_signs.wz[0],
      direction_signs.wz[1], direction_signs.wz[2], direction_signs.wz[3]);
  return true;
}

bool SentryChassisController::IsCommandTimedOut(bool command_valid,
                                                const ros::Time& command_stamp,
                                                const ros::Time& now, double timeout_sec)
{
  if (!command_valid)
  {
    return true;
  }
  if (timeout_sec < 0.0)
  {
    return true;
  }
  const double age = (now - command_stamp).toSec();
  return age < 0.0 || age > timeout_sec;
}

void SentryChassisController::starting(const ros::Time& time)
{
  SetAllCommands(&steer_joints_, 0.0);
  SetAllCommands(&wheel_joints_, 0.0);
  for (auto& pid : steer_pids_)
  {
    pid.reset();
  }
  for (auto& pid : wheel_pids_)
  {
    pid.reset();
  }

  CommandData command;
  command.stamp = time;
  command.valid = false;
  command_buffer_.writeFromNonRT(command);
}

void SentryChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  if (period.toSec() <= 0.0)
  {
    ROS_WARN_THROTTLE(1.0, "Skip update due to non-positive period.");
    return;
  }

  CommandData command = *command_buffer_.readFromRT();
  const bool timeout =
      IsCommandTimedOut(command.valid, command.stamp, time, cmd_vel_timeout_);

  // cmd_vel is interpreted in base_link: +x forward, +y left, +z yaw CCW.
  const double vx = timeout ? 0.0 : command.vx;
  const double vy = timeout ? 0.0 : command.vy;
  const double wz = timeout ? 0.0 : command.wz;
  const Kinematics::WheelTargets wheel_targets =
      kinematics_.ComputeWheelAngularVelocity(vx, vy, wz);
  const std::array<double, WHEEL_COUNT> wheel_target_values = {
      wheel_targets.front_left, wheel_targets.front_right, wheel_targets.rear_left,
      wheel_targets.rear_right};

  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    // 舵向按位置误差闭环，目标值来自 steer_zero_offsets_。
    // Steering loop closes on position error using steer_zero_offsets_ as target.
    const double steer_error = steer_zero_offsets_[i] - steer_joints_[i].getPosition();
    const double steer_effort = steer_pids_[i].computeCommand(steer_error, period);
    steer_joints_[i].setCommand(steer_effort);
  }

  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    // 轮速按速度误差闭环，目标值由逆运动学解算得到。
    // Wheel loop closes on velocity error using IK-computed targets.
    const double wheel_error = wheel_target_values[i] - wheel_joints_[i].getVelocity();
    const double wheel_effort = wheel_pids_[i].computeCommand(wheel_error, period);
    wheel_joints_[i].setCommand(wheel_effort);
  }
}

void SentryChassisController::stopping(const ros::Time& time)
{
  (void)time;
  SetAllCommands(&steer_joints_, 0.0);
  SetAllCommands(&wheel_joints_, 0.0);
  for (auto& pid : steer_pids_)
  {
    pid.reset();
  }
  for (auto& pid : wheel_pids_)
  {
    pid.reset();
  }
}

void SentryChassisController::CmdVelCallback(const geometry_msgs::TwistConstPtr& message)
{
  CommandData command;
  command.vx = message->linear.x;
  command.vy = message->linear.y;
  command.wz = message->angular.z;
  command.stamp = ros::Time::now();
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
}

bool SentryChassisController::InitPidGroup(
    ros::NodeHandle& nh, const std::string& pid_group,
    std::array<control_toolbox::Pid, WHEEL_COUNT>* pid_array)
{
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    const std::string ns = "pid/" + pid_group + "/" + kWheelNameSuffix[i];
    ros::NodeHandle pid_nh(nh, ns);
    if (enable_dynamic_reconfigure_)
    {
      // 动态调参模式：由 control_toolbox 自动挂载 dynamic_reconfigure 服务。
      // Dynamic mode: control_toolbox internally provides dynamic_reconfigure services.
      if (!pid_array->at(i).init(pid_nh, false))
      {
        ROS_ERROR("Failed to initialize PID at namespace '%s'.",
                  pid_nh.getNamespace().c_str());
        return false;
      }
    }
    else
    {
      // 固定参数模式：直接读取静态 YAML 参数，不注册动态调参接口。
      // Fixed mode: read static YAML parameters without dynamic reconfigure endpoint.
      double p = 0.0;
      double i_gain = 0.0;
      double d = 0.0;
      double i_clamp_min = 0.0;
      double i_clamp_max = 0.0;
      bool antiwindup = false;

      if (!pid_nh.getParam("p", p) || !pid_nh.getParam("i", i_gain) ||
          !pid_nh.getParam("d", d) || !pid_nh.getParam("i_clamp_min", i_clamp_min) ||
          !pid_nh.getParam("i_clamp_max", i_clamp_max))
      {
        ROS_ERROR("Missing PID parameters at namespace '%s'.",
                  pid_nh.getNamespace().c_str());
        return false;
      }
      pid_nh.param("antiwindup", antiwindup, false);
      pid_array->at(i).initPid(p, i_gain, d, i_clamp_max, i_clamp_min, antiwindup);
    }
  }
  return true;
}

bool SentryChassisController::LoadDirectionSigns(
    ros::NodeHandle& nh, Kinematics::DirectionSigns* direction_signs)
{
  std::vector<int> vx_values;
  std::vector<int> vy_values;
  std::vector<int> wz_values;
  const bool has_vx = nh.getParam("wheel_direction_signs/vx", vx_values);
  const bool has_vy = nh.getParam("wheel_direction_signs/vy", vy_values);
  const bool has_wz = nh.getParam("wheel_direction_signs/wz", wz_values);

  if (!has_vx && !has_vy && !has_wz)
  {
    // 三轴都未配置时使用全 1 默认值，保持历史行为。
    // Keep legacy behavior when all three axes are omitted.
    *direction_signs = Kinematics::DirectionSigns();
    return true;
  }
  if (!(has_vx && has_vy && has_wz))
  {
    ROS_ERROR(
        "Parameters 'wheel_direction_signs/vx', 'wheel_direction_signs/vy' and "
        "'wheel_direction_signs/wz' must all be set when any one is provided.");
    return false;
  }

  if (!ParseDirectionAxis(vx_values, "wheel_direction_signs/vx", &direction_signs->vx))
  {
    return false;
  }
  if (!ParseDirectionAxis(vy_values, "wheel_direction_signs/vy", &direction_signs->vy))
  {
    return false;
  }
  if (!ParseDirectionAxis(wz_values, "wheel_direction_signs/wz", &direction_signs->wz))
  {
    return false;
  }
  return true;
}

bool SentryChassisController::ParseDirectionAxis(const std::vector<int>& axis_values,
                                                 const std::string& param_name,
                                                 std::array<int, WHEEL_COUNT>* output)
{
  // 参数必须完整覆盖四个轮子，且每项仅允许 {-1, 1}。
  // Each axis must provide exactly four entries and each value must be {-1, 1}.
  if (axis_values.size() != WHEEL_COUNT)
  {
    ROS_ERROR("Parameter '%s' must contain exactly %zu items.", param_name.c_str(),
              WHEEL_COUNT);
    return false;
  }

  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    if (axis_values[i] != -1 && axis_values[i] != 1)
    {
      ROS_ERROR("Parameter '%s[%zu]' must be -1 or 1, got %d.", param_name.c_str(), i,
                axis_values[i]);
      return false;
    }
    output->at(i) = axis_values[i];
  }
  return true;
}

void SentryChassisController::SetAllCommands(
    std::vector<hardware_interface::JointHandle>* joints, double command)
{
  for (auto& joint : *joints)
  {
    joint.setCommand(command);
  }
}

}  // namespace sentry_chassis_controller

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController,
                       controller_interface::ControllerBase)
