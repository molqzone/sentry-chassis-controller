#include "sentry_chassis_controller/sentry_chassis_controller.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace sentry_chassis_controller
{
namespace
{
constexpr std::size_t kWheelCount = 4;
constexpr double kZeroVelocityEpsilon = 1e-9;

bool LoadSignArray(ros::NodeHandle& controller_nh, const std::string& param_name,
                   std::array<int, kWheelCount>* values)
{
  values->fill(1);
  XmlRpc::XmlRpcValue raw_values;
  if (!controller_nh.getParam(param_name, raw_values))
  {
    return true;
  }
  if (raw_values.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      raw_values.size() != static_cast<int>(kWheelCount))
  {
    ROS_ERROR("Parameter '%s' must be an array of 4 integers.",
              param_name.c_str());
    return false;
  }
  for (int index = 0; index < raw_values.size(); ++index)
  {
    int value = 0;
    if (raw_values[index].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      value = static_cast<int>(raw_values[index]);
    }
    else if (raw_values[index].getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
      value = static_cast<int>(static_cast<double>(raw_values[index]));
    }
    else
    {
      ROS_ERROR("Parameter '%s[%d]' must be -1 or 1.", param_name.c_str(),
                index);
      return false;
    }
    if (value != -1 && value != 1)
    {
      ROS_ERROR("Parameter '%s[%d]' must be -1 or 1.", param_name.c_str(),
                index);
      return false;
    }
    (*values)[static_cast<std::size_t>(index)] = value;
  }
  return true;
}

std::size_t ModuleIndexFromName(const std::string& name, std::size_t fallback)
{
  if (name == "left_front")
  {
    return 0;
  }
  if (name == "right_front")
  {
    return 1;
  }
  if (name == "left_back")
  {
    return 2;
  }
  if (name == "right_back")
  {
    return 3;
  }
  return fallback;
}
}  // namespace

bool SentryChassisController::init(hardware_interface::RobotHW* robot_hw,
                                   ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  if (!ChassisBase::init(robot_hw, root_nh, controller_nh))
  {
    return false;
  }

  std::array<int, kWheelCount> wheel_rolling_signs{};
  std::array<int, kWheelCount> wheel_direction_vx{};
  std::array<int, kWheelCount> wheel_direction_vy{};
  std::array<int, kWheelCount> wheel_direction_wz{};
  if (!LoadSignArray(controller_nh, "wheel_rolling_signs",
                     &wheel_rolling_signs) ||
      !LoadSignArray(controller_nh, "wheel_direction_signs/vx",
                     &wheel_direction_vx) ||
      !LoadSignArray(controller_nh, "wheel_direction_signs/vy",
                     &wheel_direction_vy) ||
      !LoadSignArray(controller_nh, "wheel_direction_signs/wz",
                     &wheel_direction_wz))
  {
    return false;
  }

  XmlRpc::XmlRpcValue modules;
  if (!controller_nh.getParam("modules", modules) ||
      modules.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("Parameter 'modules' must be a struct.");
    return false;
  }

  std::size_t fallback_module_index = 0;
  for (const auto& module_param : modules)
  {
    const std::string module_name = module_param.first;
    const XmlRpc::XmlRpcValue& config = module_param.second;
    if (!config.hasMember("position") ||
        config["position"].getType() != XmlRpc::XmlRpcValue::TypeArray ||
        config["position"].size() != 2 || !config.hasMember("pivot") ||
        config["pivot"].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !config.hasMember("wheel") ||
        config["wheel"].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !config["wheel"].hasMember("radius"))
    {
      ROS_ERROR("Module '%s' is missing rm-style swerve configuration.",
                module_name.c_str());
      return false;
    }

    const std::size_t module_index =
        ModuleIndexFromName(module_name, fallback_module_index++);
    Module module;
    module.position = Eigen::Vector2d(static_cast<double>(config["position"][0]),
                                      static_cast<double>(config["position"][1]));
    module.pivot_offset = static_cast<double>(config["pivot"]["offset"]);
    module.wheel_radius = static_cast<double>(config["wheel"]["radius"]);
    module.wheel_rolling_sign = wheel_rolling_signs[module_index];
    module.vx_sign = wheel_direction_vx[module_index];
    module.vy_sign = wheel_direction_vy[module_index];
    module.wz_sign = wheel_direction_wz[module_index];
    module.pivot.reset(new effort_controllers::JointPositionController());
    module.wheel.reset(new effort_controllers::JointVelocityController());

    ros::NodeHandle pivot_nh(controller_nh, "modules/" + module_name + "/pivot");
    ros::NodeHandle wheel_nh(controller_nh, "modules/" + module_name + "/wheel");
    if (!module.pivot->init(effort_joint_interface_, pivot_nh) ||
        !module.wheel->init(effort_joint_interface_, wheel_nh))
    {
      return false;
    }

    joint_handles_.push_back(module.pivot->joint_);
    joint_handles_.push_back(module.wheel->joint_);
    modules_.push_back(std::move(module));
  }

  return !modules_.empty();
}

void SentryChassisController::moveJoint(const ros::Time& time,
                                        const ros::Duration& period)
{
  for (auto& module : modules_)
  {
    const Eigen::Vector2d module_velocity(
        static_cast<double>(module.vx_sign) * vel_cmd_.x -
            static_cast<double>(module.wz_sign) * module.position.y() *
                vel_cmd_.z,
        static_cast<double>(module.vy_sign) * vel_cmd_.y +
            static_cast<double>(module.wz_sign) * module.position.x() *
                vel_cmd_.z);

    const double module_speed = module_velocity.norm();
    if (module_speed <= kZeroVelocityEpsilon)
    {
      module.pivot->setCommand(module.pivot->joint_.getPosition());
      module.wheel->setCommand(0.0);
      module.pivot->update(time, period);
      module.wheel->update(time, period);
      continue;
    }

    const double velocity_angle =
        std::atan2(module_velocity.y(), module_velocity.x()) +
        module.pivot_offset;
    const double direct_delta = angles::shortest_angular_distance(
        module.pivot->joint_.getPosition(), velocity_angle);
    const double flipped_delta = angles::shortest_angular_distance(
        module.pivot->joint_.getPosition(), velocity_angle + M_PI);
    const bool use_direct_target =
        std::abs(direct_delta) < std::abs(flipped_delta);
    module.pivot->setCommand(use_direct_target ? velocity_angle
                                               : velocity_angle + M_PI);
    const double wheel_velocity =
        static_cast<double>(module.wheel_rolling_sign) * module_speed /
        module.wheel_radius * std::cos(direct_delta);
    module.wheel->setCommand(wheel_velocity);
    module.pivot->update(time, period);
    module.wheel->update(time, period);
  }
}

geometry_msgs::Twist SentryChassisController::odometry()
{
  geometry_msgs::Twist twist;
  if (modules_.size() < 3)
  {
    return twist;
  }

  Eigen::Matrix<double, Eigen::Dynamic, 3> a(modules_.size(), 3);
  Eigen::VectorXd b(modules_.size());
  for (std::size_t index = 0; index < modules_.size(); ++index)
  {
    const Module& module = modules_[index];
    const double heading = module.pivot->joint_.getPosition() - module.pivot_offset;
    const double direction_x = std::cos(heading);
    const double direction_y = std::sin(heading);
    const double wheel_linear_speed =
        static_cast<double>(module.wheel_rolling_sign) *
        module.wheel->joint_.getVelocity() * module.wheel_radius;

    a(static_cast<Eigen::Index>(index), 0) = direction_x;
    a(static_cast<Eigen::Index>(index), 1) = direction_y;
    a(static_cast<Eigen::Index>(index), 2) =
        -direction_x * module.position.y() + direction_y * module.position.x();
    b(static_cast<Eigen::Index>(index)) = wheel_linear_speed;
  }

  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, 3>> qr(a);
  qr.setThreshold(1e-9);
  if (qr.rank() < 3)
  {
    return twist;
  }

  const Eigen::Vector3d solved = qr.solve(b);
  twist.linear.x = solved.x();
  twist.linear.y = solved.y();
  twist.angular.z = solved.z();
  return twist;
}

}  // namespace sentry_chassis_controller

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController,
                       controller_interface::ControllerBase)
