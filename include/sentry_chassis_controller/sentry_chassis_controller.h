#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_

#include <array>
#include <string>
#include <vector>

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <realtime_tools/realtime_buffer.h>

#include "sentry_chassis_controller/kinematics.hpp"

namespace sentry_chassis_controller {

class SentryChassisController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  static constexpr std::size_t WHEEL_COUNT = 4U;

  static bool IsCommandTimedOut(bool command_valid, const ros::Time& command_stamp,
                                const ros::Time& now, double timeout_sec);

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

  void starting(const ros::Time& time) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void stopping(const ros::Time& time) override;

 private:
  struct CommandData {
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;
    ros::Time stamp;
    bool valid = false;
  };

  void CmdVelCallback(const geometry_msgs::TwistConstPtr& message);

  bool InitPidGroup(ros::NodeHandle& nh, const std::string& pid_group,
                    std::array<control_toolbox::Pid, WHEEL_COUNT>* pid_array);

  static void SetAllCommands(std::vector<hardware_interface::JointHandle>* joints, double command);

  std::vector<hardware_interface::JointHandle> steer_joints_;
  std::vector<hardware_interface::JointHandle> wheel_joints_;
  std::array<control_toolbox::Pid, WHEEL_COUNT> steer_pids_;
  std::array<control_toolbox::Pid, WHEEL_COUNT> wheel_pids_;
  std::array<double, WHEEL_COUNT> steer_zero_offsets_{};
  Kinematics kinematics_;
  ros::Subscriber cmd_vel_subscriber_;
  realtime_tools::RealtimeBuffer<CommandData> command_buffer_;
  std::string cmd_vel_topic_ = "/cmd_vel";
  std::string command_frame_id_ = "base_link";
  double cmd_vel_timeout_ = 0.25;
  bool enable_dynamic_reconfigure_ = true;
};

}  // namespace sentry_chassis_controller

#endif  // SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_
