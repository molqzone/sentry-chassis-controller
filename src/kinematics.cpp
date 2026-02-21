#include "sentry_chassis_controller/kinematics.hpp"

namespace sentry_chassis_controller
{

Kinematics::Kinematics() = default;

Kinematics::Kinematics(const Geometry& geometry) : geometry_(geometry) {}

void Kinematics::SetGeometry(const Geometry& geometry) { geometry_ = geometry; }

Kinematics::WheelTargets Kinematics::ComputeWheelAngularVelocity(double vx, double vy,
                                                                 double wz) const
{
  // Mecanum inverse kinematics in base_link.
  // fl = (vx - vy - ((wheel_base + track_width) / 2) * wz) / wheel_radius
  // fr = (vx + vy + ((wheel_base + track_width) / 2) * wz) / wheel_radius
  // rl = (vx + vy - ((wheel_base + track_width) / 2) * wz) / wheel_radius
  // rr = (vx - vy + ((wheel_base + track_width) / 2) * wz) / wheel_radius
  WheelTargets targets;
  const double HALF_SUM = (geometry_.wheel_base + geometry_.track_width) * 0.5;
  const double RADIUS = geometry_.wheel_radius > 1e-9 ? geometry_.wheel_radius : 1e-9;

  targets.front_left = (vx - vy - HALF_SUM * wz) / RADIUS;
  targets.front_right = (vx + vy + HALF_SUM * wz) / RADIUS;
  targets.rear_left = (vx + vy - HALF_SUM * wz) / RADIUS;
  targets.rear_right = (vx - vy + HALF_SUM * wz) / RADIUS;

  return targets;
}

}  // namespace sentry_chassis_controller
