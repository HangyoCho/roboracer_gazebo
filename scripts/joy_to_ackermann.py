#!/usr/bin/env python3
"""
Minimal joystick to AckermannDriveStamped converter.
Hold LB (button 4) as deadman switch, left stick Y = speed, right stick X = steering.
"""
import rospy
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped


class JoyToAckermann:
    def __init__(self):
        rospy.init_node('joy_to_ackermann')
        self.max_speed = rospy.get_param('~max_speed', 4.0)
        self.max_steer = rospy.get_param('~max_steer', 0.4)
        self.speed_axis = rospy.get_param('~speed_axis', 1)
        self.steer_axis = rospy.get_param('~steer_axis', 3)
        self.deadman_button = rospy.get_param('~deadman_button', 4)
        out_topic = rospy.get_param('~out_topic', '/vesc/low_level/ackermann_cmd_mux/output')

        self.pub = rospy.Publisher(out_topic, AckermannDriveStamped, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.joy_cb)
        rospy.loginfo('joy_to_ackermann ready (deadman=btn%d, out=%s)', self.deadman_button, out_topic)

    def joy_cb(self, msg):
        if len(msg.buttons) <= self.deadman_button:
            return
        if not msg.buttons[self.deadman_button]:
            return
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.drive.speed = msg.axes[self.speed_axis] * self.max_speed
        cmd.drive.steering_angle = msg.axes[self.steer_axis] * self.max_steer
        self.pub.publish(cmd)


if __name__ == '__main__':
    JoyToAckermann()
    rospy.spin()
