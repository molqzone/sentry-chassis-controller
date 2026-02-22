#include "sentry_chassis_controller/kinematics.hpp"

namespace sentry_chassis_controller
{

Kinematics::Kinematics() = default;

Kinematics::Kinematics(const Geometry& geometry) : geometry_(geometry) {}

void Kinematics::SetGeometry(const Geometry& geometry) { geometry_ = geometry; }

void Kinematics::SetDirectionSigns(const DirectionSigns& direction_signs)
{
  direction_signs_ = direction_signs;
}

Kinematics::WheelTargets Kinematics::ComputeWheelAngularVelocity(double vx, double vy,
                                                                 double wz) const
{
  // Mecanum inverse kinematics in base_link.
  // Wheel direction signs are applied per axis to align model-side joint direction.
  // Wheel order is fixed: front_left, front_right, rear_left, rear_right.
  // fl = (sx_fl*vx - sy_fl*vy - sz_fl*((wheel_base + track_width) / 2)*wz) / wheel_radius
  // fr = (sx_fr*vx + sy_fr*vy + sz_fr*((wheel_base + track_width) / 2)*wz) / wheel_radius
  // rl = (sx_rl*vx + sy_rl*vy - sz_rl*((wheel_base + track_width) / 2)*wz) / wheel_radius
  // rr = (sx_rr*vx - sy_rr*vy + sz_rr*((wheel_base + track_width) / 2)*wz) / wheel_radius
  WheelTargets targets;
  const double HALF_SUM = (geometry_.wheel_base + geometry_.track_width) * 0.5;
  const double RADIUS = geometry_.wheel_radius > 1e-9 ? geometry_.wheel_radius : 1e-9;

  targets.front_left = (static_cast<double>(direction_signs_.vx[0]) * vx -
                        static_cast<double>(direction_signs_.vy[0]) * vy -
                        static_cast<double>(direction_signs_.wz[0]) * HALF_SUM * wz) /
                       RADIUS;
  targets.front_right = (static_cast<double>(direction_signs_.vx[1]) * vx +
                         static_cast<double>(direction_signs_.vy[1]) * vy +
                         static_cast<double>(direction_signs_.wz[1]) * HALF_SUM * wz) /
                        RADIUS;
  targets.rear_left = (static_cast<double>(direction_signs_.vx[2]) * vx +
                       static_cast<double>(direction_signs_.vy[2]) * vy -
                       static_cast<double>(direction_signs_.wz[2]) * HALF_SUM * wz) /
                      RADIUS;
  targets.rear_right = (static_cast<double>(direction_signs_.vx[3]) * vx -
                        static_cast<double>(direction_signs_.vy[3]) * vy +
                        static_cast<double>(direction_signs_.wz[3]) * HALF_SUM * wz) /
                       RADIUS;

  return targets;
}

}  // namespace sentry_chassis_controller
