#include "sentry_chassis_controller/sentry_chassis_controller.h"

#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <pluginlib/class_list_macros.h>

namespace sentry_chassis_controller {
namespace {
constexpr double kMinWheelRadius = 1e-9;
const std::array<std::string, SentryChassisController::kWheelCount> kWheelNameSuffix = {
    "front_left", "front_right", "rear_left", "rear_right"};
}  // namespace

bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw,
                                   ros::NodeHandle& nh) {
  std::vector<std::string> steer_joint_names;
  std::vector<std::string> wheel_joint_names;
  std::vector<double> steer_zero_offsets;

  if (!nh.getParam("steer_joints", steer_joint_names) || steer_joint_names.size() != kWheelCount) {
    ROS_ERROR("Parameter 'steer_joints' must exist and contain exactly %zu items.", kWheelCount);
    return false;
  }

  if (!nh.getParam("wheel_joints", wheel_joint_names) || wheel_joint_names.size() != kWheelCount) {
    ROS_ERROR("Parameter 'wheel_joints' must exist and contain exactly %zu items.", kWheelCount);
    return false;
  }

  if (!nh.getParam("steer_zero_offsets", steer_zero_offsets)) {
    steer_zero_offsets.assign(kWheelCount, 0.0);
  }
  if (steer_zero_offsets.size() != kWheelCount) {
    ROS_ERROR("Parameter 'steer_zero_offsets' must contain exactly %zu items.", kWheelCount);
    return false;
  }
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    steer_zero_offsets_[i] = steer_zero_offsets[i];
  }

  Kinematics::Geometry geometry;
  nh.param("geometry/wheel_base", geometry.wheel_base, 0.50);
  nh.param("geometry/track_width", geometry.track_width, 0.40);
  nh.param("geometry/wheel_radius", geometry.wheel_radius, 0.076);
  if (geometry.wheel_radius <= kMinWheelRadius) {
    ROS_WARN("Parameter 'geometry/wheel_radius' must be positive. Clamping to %.9f.",
             kMinWheelRadius);
    geometry.wheel_radius = kMinWheelRadius;
  }
  kinematics_.SetGeometry(geometry);

  nh.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
  nh.param("command_frame_id", command_frame_id_, std::string("base_link"));
  nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.25);
  nh.param("enable_dynamic_reconfigure", enable_dynamic_reconfigure_, true);
  if (command_frame_id_ != "base_link") {
    ROS_WARN("Parameter 'command_frame_id' is '%s', but only 'base_link' is supported in this "
             "stage. Falling back to 'base_link'.",
             command_frame_id_.c_str());
    command_frame_id_ = "base_link";
  }
  if (cmd_vel_timeout_ < 0.0) {
    ROS_WARN("Parameter 'cmd_vel_timeout' is negative. Clamping to 0.0.");
    cmd_vel_timeout_ = 0.0;
  }

  steer_joints_.clear();
  wheel_joints_.clear();
  steer_joints_.reserve(kWheelCount);
  wheel_joints_.reserve(kWheelCount);

  try {
    for (const auto& name : steer_joint_names) {
      steer_joints_.push_back(hw->getHandle(name));
    }
    for (const auto& name : wheel_joint_names) {
      wheel_joints_.push_back(hw->getHandle(name));
    }
  } catch (const hardware_interface::HardwareInterfaceException& exception) {
    ROS_ERROR("Failed to get joint handle: %s", exception.what());
    return false;
  }

  if (!InitPidGroup(nh, "steer", &steer_pids_)) {
    return false;
  }
  if (!InitPidGroup(nh, "wheel", &wheel_pids_)) {
    return false;
  }

  CommandData command;
  command.stamp = ros::Time(0);
  command.valid = false;
  command_buffer_.writeFromNonRT(command);

  cmd_vel_subscriber_ =
      nh.subscribe(cmd_vel_topic_, 1, &SentryChassisController::CmdVelCallback, this);

  ROS_INFO("SentryChassisController initialized with cmd_vel_topic='%s', command_frame_id='%s', "
           "timeout=%.3fs.",
           cmd_vel_topic_.c_str(), command_frame_id_.c_str(), cmd_vel_timeout_);
  return true;
}

bool SentryChassisController::IsCommandTimedOut(bool command_valid, const ros::Time& command_stamp,
                                                const ros::Time& now, double timeout_sec) {
  if (!command_valid) {
    return true;
  }
  if (timeout_sec < 0.0) {
    return true;
  }
  const double age = (now - command_stamp).toSec();
  return age < 0.0 || age > timeout_sec;
}

void SentryChassisController::starting(const ros::Time& time) {
  SetAllCommands(&steer_joints_, 0.0);
  SetAllCommands(&wheel_joints_, 0.0);
  for (auto& pid : steer_pids_) {
    pid.reset();
  }
  for (auto& pid : wheel_pids_) {
    pid.reset();
  }

  CommandData command;
  command.stamp = time;
  command.valid = false;
  command_buffer_.writeFromNonRT(command);
}

void SentryChassisController::update(const ros::Time& time, const ros::Duration& period) {
  if (period.toSec() <= 0.0) {
    ROS_WARN_THROTTLE(1.0, "Skip update due to non-positive period.");
    return;
  }

  CommandData command = *command_buffer_.readFromRT();
  const bool timeout = IsCommandTimedOut(command.valid, command.stamp, time, cmd_vel_timeout_);

  // cmd_vel is interpreted in base_link: +x forward, +y left, +z yaw CCW.
  const double vx = timeout ? 0.0 : command.vx;
  const double vy = timeout ? 0.0 : command.vy;
  const double wz = timeout ? 0.0 : command.wz;
  const Kinematics::WheelTargets wheel_targets = kinematics_.ComputeWheelAngularVelocity(vx, vy, wz);
  const std::array<double, kWheelCount> wheel_target_values = {wheel_targets.front_left,
                                                                wheel_targets.front_right,
                                                                wheel_targets.rear_left,
                                                                wheel_targets.rear_right};

  for (std::size_t i = 0; i < kWheelCount; ++i) {
    const double steer_error = steer_zero_offsets_[i] - steer_joints_[i].getPosition();
    const double steer_effort = steer_pids_[i].computeCommand(steer_error, period);
    steer_joints_[i].setCommand(steer_effort);
  }

  for (std::size_t i = 0; i < kWheelCount; ++i) {
    const double wheel_error = wheel_target_values[i] - wheel_joints_[i].getVelocity();
    const double wheel_effort = wheel_pids_[i].computeCommand(wheel_error, period);
    wheel_joints_[i].setCommand(wheel_effort);
  }
}

void SentryChassisController::stopping(const ros::Time& time) {
  (void)time;
  SetAllCommands(&steer_joints_, 0.0);
  SetAllCommands(&wheel_joints_, 0.0);
  for (auto& pid : steer_pids_) {
    pid.reset();
  }
  for (auto& pid : wheel_pids_) {
    pid.reset();
  }
}

void SentryChassisController::CmdVelCallback(const geometry_msgs::TwistConstPtr& message) {
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
    std::array<control_toolbox::Pid, kWheelCount>* pid_array) {
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    const std::string ns = "pid/" + pid_group + "/" + kWheelNameSuffix[i];
    ros::NodeHandle pid_nh(nh, ns);
    if (enable_dynamic_reconfigure_) {
      if (!pid_array->at(i).init(pid_nh, false)) {
        ROS_ERROR("Failed to initialize PID at namespace '%s'.", pid_nh.getNamespace().c_str());
        return false;
      }
    } else {
      double p = 0.0;
      double i_gain = 0.0;
      double d = 0.0;
      double i_clamp_min = 0.0;
      double i_clamp_max = 0.0;
      bool antiwindup = false;

      if (!pid_nh.getParam("p", p) || !pid_nh.getParam("i", i_gain) || !pid_nh.getParam("d", d) ||
          !pid_nh.getParam("i_clamp_min", i_clamp_min) ||
          !pid_nh.getParam("i_clamp_max", i_clamp_max)) {
        ROS_ERROR("Missing PID parameters at namespace '%s'.", pid_nh.getNamespace().c_str());
        return false;
      }
      pid_nh.param("antiwindup", antiwindup, false);
      pid_array->at(i).initPid(p, i_gain, d, i_clamp_max, i_clamp_min, antiwindup);
    }
  }
  return true;
}

void SentryChassisController::SetAllCommands(
    std::vector<hardware_interface::JointHandle>* joints, double command) {
  for (auto& joint : *joints) {
    joint.setCommand(command);
  }
}

}  // namespace sentry_chassis_controller

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController,
                       controller_interface::ControllerBase)
