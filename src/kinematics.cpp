#include "sentry_chassis_controller/kinematics.hpp"

#include "controller_internal.hpp"

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <utility>

namespace sentry_chassis_controller
{
namespace
{
constexpr double kRankTolerance = 1e-9;

double NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::array<std::pair<double, double>, Kinematics::WHEEL_COUNT> BuildModulePositions(
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

Eigen::Vector2d ComputeModuleVelocity(const Kinematics::DirectionSigns& direction_signs,
                                      std::size_t module_index, double module_x,
                                      double module_y,
                                      const Kinematics::ChassisTwist& twist)
{
  const double vx_sign = static_cast<double>(direction_signs.vx[module_index]);
  const double vy_sign = static_cast<double>(direction_signs.vy[module_index]);
  const double wz_sign = static_cast<double>(direction_signs.wz[module_index]);
  return Eigen::Vector2d(vx_sign * twist.vx - wz_sign * module_y * twist.wz,
                         vy_sign * twist.vy + wz_sign * module_x * twist.wz);
}
}  // namespace

Kinematics::Kinematics() = default;

Kinematics::Kinematics(const Geometry& geometry) : geometry_(geometry) {}

void Kinematics::SetGeometry(const Geometry& geometry) { geometry_ = geometry; }

void Kinematics::SetDirectionSigns(const DirectionSigns& direction_signs)
{
  direction_signs_ = direction_signs;
}

Kinematics::WheelTargetSolution Kinematics::ComputeWheelTargets(
    const ChassisTwist& twist,
    const std::array<double, WHEEL_COUNT>& steer_zero_offsets,
    const std::array<double, WHEEL_COUNT>& steer_positions) const
{
  WheelTargetSolution solution;
  const double wheel_radius =
      geometry_.wheel_radius > MIN_WHEEL_RADIUS ? geometry_.wheel_radius : MIN_WHEEL_RADIUS;
  const auto module_positions = BuildModulePositions(geometry_);

  for (std::size_t index = 0; index < WHEEL_COUNT; ++index)
  {
    WheelTarget& target = solution.modules[index];
    target.steer_error = NormalizeAngle(steer_zero_offsets[index] - steer_positions[index]);

    const Eigen::Vector2d module_velocity =
        ComputeModuleVelocity(direction_signs_, index, module_positions[index].first,
                              module_positions[index].second, twist);
    const double module_speed = std::hypot(module_velocity.x(), module_velocity.y());
    if (module_speed <= ZERO_CMD_EPS)
    {
      continue;
    }

    double steer_error = NormalizeAngle(
        std::atan2(module_velocity.y(), module_velocity.x()) + steer_zero_offsets[index] -
        steer_positions[index]);
    double wheel_target = module_speed / wheel_radius;
    if (steer_error > HALF_PI + STEER_FLIP_EPS)
    {
      steer_error -= PI;
      wheel_target = -wheel_target;
    }
    else if (steer_error < -HALF_PI - STEER_FLIP_EPS)
    {
      steer_error += PI;
      wheel_target = -wheel_target;
    }

    target.steer_error = steer_error;
    target.wheel_angular_velocity = wheel_target;
    target.active = true;
  }
  return solution;
}

bool Kinematics::ComputeChassisTwistFromWheelFeedback(
    const WheelFeedback& feedback,
    const std::array<double, WHEEL_COUNT>& steer_zero_offsets,
    const std::array<int, WHEEL_COUNT>& wheel_rolling_signs, ChassisTwist* twist) const
{
  if (twist == nullptr)
  {
    return false;
  }

  const double wheel_radius =
      geometry_.wheel_radius > MIN_WHEEL_RADIUS ? geometry_.wheel_radius : MIN_WHEEL_RADIUS;
  const auto module_positions = BuildModulePositions(geometry_);

  Eigen::Matrix<double, 4, 3> a = Eigen::Matrix<double, 4, 3>::Zero();
  Eigen::Matrix<double, 4, 1> b = Eigen::Matrix<double, 4, 1>::Zero();

  for (std::size_t index = 0; index < WHEEL_COUNT; ++index)
  {
    if (wheel_rolling_signs[index] != -1 && wheel_rolling_signs[index] != 1)
    {
      return false;
    }

    const double steer_angle = feedback.steer_position[index] - steer_zero_offsets[index];
    const double direction_x = std::cos(steer_angle);
    const double direction_y = std::sin(steer_angle);
    const double module_x = module_positions[index].first;
    const double module_y = module_positions[index].second;
    const double wheel_linear_speed = static_cast<double>(wheel_rolling_signs[index]) *
                                      wheel_radius *
                                      feedback.wheel_angular_velocity[index];

    a(index, 0) = direction_x;
    a(index, 1) = direction_y;
    a(index, 2) = -direction_x * module_y + direction_y * module_x;
    b(index) = wheel_linear_speed;
  }

  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 4, 3>> qr(a);
  qr.setThreshold(kRankTolerance);
  if (qr.rank() < 3)
  {
    return false;
  }
  const Eigen::Vector3d solved = qr.solve(b);

  twist->vx = solved.x();
  twist->vy = solved.y();
  twist->wz = solved.z();
  return true;
}

}  // namespace sentry_chassis_controller
