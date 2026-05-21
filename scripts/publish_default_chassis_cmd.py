#!/usr/bin/env python3
"""Publish a default rm_msgs/ChassisCmd so cmd_vel teleop can drive rm_controllers."""

import rospy

from rm_msgs.msg import ChassisCmd


def main():
    rospy.init_node("publish_default_chassis_cmd")

    topic = rospy.get_param("~topic", "/cmd_chassis")
    rate_hz = float(rospy.get_param("~rate", 10.0))
    power_limit = float(rospy.get_param("~power_limit", 100000.0))
    accel_linear_x = float(rospy.get_param("~accel_linear_x", 100.0))
    accel_linear_y = float(rospy.get_param("~accel_linear_y", 100.0))
    accel_angular_z = float(rospy.get_param("~accel_angular_z", 100.0))
    command_source_frame = rospy.get_param("~command_source_frame", "base_link")
    follow_source_frame = rospy.get_param("~follow_source_frame", "")
    follow_vel_des = float(rospy.get_param("~follow_vel_des", 0.0))
    wireless_state = bool(rospy.get_param("~wireless_state", True))

    publisher = rospy.Publisher(topic, ChassisCmd, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        message = ChassisCmd()
        message.mode = ChassisCmd.RAW
        message.power_limit = power_limit
        message.follow_vel_des = follow_vel_des
        message.follow_source_frame = follow_source_frame
        message.command_source_frame = command_source_frame
        message.wireless_state = wireless_state
        message.accel.linear.x = accel_linear_x
        message.accel.linear.y = accel_linear_y
        message.accel.angular.z = accel_angular_z
        message.stamp = rospy.Time.now()
        publisher.publish(message)
        rate.sleep()


if __name__ == "__main__":
    main()
