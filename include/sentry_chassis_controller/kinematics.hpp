#pragma once

namespace sentry_chassis_controller {

class Kinematics {
 public:
  // Geometry uses full dimensions: wheel_base is front-rear distance, track_width is
  // left-right distance.
  struct Geometry {
    double wheel_base = 0.50;
    double track_width = 0.40;
    double wheel_radius = 0.076;
  };

  struct WheelTargets {
    double front_left = 0.0;
    double front_right = 0.0;
    double rear_left = 0.0;
    double rear_right = 0.0;
  };

  Kinematics();

  explicit Kinematics(const Geometry& geometry);

  void SetGeometry(const Geometry& geometry);

  // Input twist is interpreted in base_link: +x forward, +y left, +z yaw CCW.
  // Output wheel order is fixed as front_left, front_right, rear_left, rear_right.
  WheelTargets ComputeWheelAngularVelocity(double vx, double vy, double wz) const;

 private:
  Geometry geometry_;
};

}  // namespace sentry_chassis_controller
