#pragma once

#include <array>

/**
 * @file   kinematics.hpp
 * @brief  舵轮底盘逆运动学定义
 *         Definition of sentry chassis inverse kinematics
 */

namespace sentry_chassis_controller
{

/**
 * @brief  舵轮底盘逆运动学工具类
 *         Inverse kinematics utility for sentry chassis
 *
 * @details
 * 该类负责把 `base_link` 坐标系下的速度指令 `(vx, vy, wz)` 映射为四个轮子的目标角速度。
 * 轮序在整个项目内固定为：
 * `front_left, front_right, rear_left, rear_right`。
 *
 * This class maps `base_link` velocity commands `(vx, vy, wz)` to four wheel
 * target angular velocities. The wheel order is fixed across the project:
 * `front_left, front_right, rear_left, rear_right`.
 */
class Kinematics
{
 public:
  /**
   * @brief  底盘几何参数（使用整车尺寸）
   *         Chassis geometry parameters (full dimensions)
   *
   * @details
   * `wheel_base` 为前后轴距，`track_width` 为左右轮距，`wheel_radius` 为轮半径。
   * `wheel_base` and `track_width` are full distances, not half-lengths.
   */
  struct Geometry
  {
    double wheel_base = 0.50;     ///< 前后轴距（米） Front-rear wheelbase in meters
    double track_width = 0.40;    ///< 左右轮距（米） Left-right track width in meters
    double wheel_radius = 0.076;  ///< 轮半径（米） Wheel radius in meters
  };

  /**
   * @brief  四轮目标角速度输出结构
   *         Four-wheel target angular velocity output
   */
  struct WheelTargets
  {
    double front_left = 0.0;   ///< 前左轮目标角速度 Front-left target angular velocity
    double front_right = 0.0;  ///< 前右轮目标角速度 Front-right target angular velocity
    double rear_left = 0.0;    ///< 后左轮目标角速度 Rear-left target angular velocity
    double rear_right = 0.0;   ///< 后右轮目标角速度 Rear-right target angular velocity
  };

  /**
   * @brief  底盘速度状态结构（base_link）
   *         Chassis twist state in base_link
   */
  struct ChassisTwist
  {
    double vx = 0.0;  ///< x 向线速度 x-axis linear velocity
    double vy = 0.0;  ///< y 向线速度 y-axis linear velocity
    double wz = 0.0;  ///< z 向角速度 z-axis angular velocity
  };

  /**
   * @brief  四轮反馈状态结构
   *         Four-wheel feedback state
   */
  struct WheelFeedback
  {
    std::array<double, 4> steer_position{
        {0.0, 0.0, 0.0, 0.0}};  ///< 舵角反馈 Steering angle feedback
    std::array<double, 4> wheel_angular_velocity{
        {0.0, 0.0, 0.0, 0.0}};  ///< 轮速反馈 Wheel angular velocity feedback
  };

  /**
   * @brief  三轴轮向符号矩阵
   *         Per-axis wheel direction sign matrix
   *
   * @details
   * 每个轴均为 4 元素数组，轮序固定为：
   * `front_left, front_right, rear_left, rear_right`。
   * Each axis array contains 4 signs with fixed wheel order:
   * `front_left, front_right, rear_left, rear_right`.
   */
  struct DirectionSigns
  {
    std::array<int, 4> vx{{1, 1, 1, 1}};  ///< x 轴符号 Signs for vx term
    std::array<int, 4> vy{{1, 1, 1, 1}};  ///< y 轴符号 Signs for vy term
    std::array<int, 4> wz{{1, 1, 1, 1}};  ///< z 轴角速度符号 Signs for wz term
  };

  /**
   * @brief  默认构造函数
   *         Default constructor
   */
  Kinematics();

  /**
   * @brief  使用给定几何参数构造逆运动学对象
   *         Constructs kinematics with explicit geometry
   * @param  geometry 底盘几何参数 Chassis geometry parameters
   */
  explicit Kinematics(const Geometry& geometry);

  /**
   * @brief  更新几何参数
   *         Updates geometry parameters
   * @param  geometry 底盘几何参数 Chassis geometry parameters
   */
  void SetGeometry(const Geometry& geometry);

  /**
   * @brief  更新轮向符号矩阵
   *         Updates wheel direction sign matrix
   * @param  direction_signs 三轴符号矩阵 Per-axis sign matrix
   */
  void SetDirectionSigns(const DirectionSigns& direction_signs);

  /**
   * @brief  根据底盘速度指令计算四轮目标角速度
   *         Computes wheel target angular velocity from chassis velocity command
   * @param  vx x 轴线速度（`base_link`，前进为正）
   *            x-axis linear velocity in `base_link` (forward is positive)
   * @param  vy y 轴线速度（`base_link`，左向为正）
   *            y-axis linear velocity in `base_link` (left is positive)
   * @param  wz z 轴角速度（`base_link`，逆时针为正）
   *            z-axis angular velocity in `base_link` (CCW is positive)
   * @return 四轮目标角速度（轮序固定）
   *         Wheel target angular velocities in fixed wheel order
   */
  WheelTargets ComputeWheelAngularVelocity(double vx, double vy, double wz) const;

  /**
   * @brief  根据舵角和轮速反馈求解底盘速度
   *         Solves chassis twist from steering and wheel feedback
   * @param  feedback 四轮反馈数据 Four-wheel feedback data
   * @param  steer_zero_offsets 舵向零位偏置 Steering zero offsets
   * @param  wheel_rolling_signs 轮速方向符号 Signs for wheel rolling direction
   * @param  twist 求解输出 Solved chassis twist output
   * @return 求解成功返回 `true`；矩阵退化或参数非法返回 `false`
   *         Returns `true` on success; `false` if singular/invalid
   */
  bool ComputeChassisTwistFromWheelFeedback(
      const WheelFeedback& feedback, const std::array<double, 4>& steer_zero_offsets,
      const std::array<int, 4>& wheel_rolling_signs, ChassisTwist* twist) const;

 private:
  Geometry geometry_;               ///< 几何参数缓存 Cached geometry
  DirectionSigns direction_signs_;  ///< 轮向符号矩阵缓存 Cached direction sign matrix
};

}  // namespace sentry_chassis_controller
