#!/usr/bin/env python3
"""
Converts geometry_msgs/Twist (cmd_vel) or ackermann_msgs/AckermannDriveStamped
to Ackermann joint commands.

Subscribes: /cmd_vel, /vesc/low_level/ackermann_cmd_mux/output
Publishes:  /unicorn/*/command (steering + wheel velocity)
"""

import math
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class TwistToJoints:
    def __init__(self):
        rospy.init_node('twist_to_joints')

        self.wheelbase = rospy.get_param('~wheelbase', 0.325)
        self.track_width = rospy.get_param('~track_width', 0.2)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.05)
        self.max_steer = rospy.get_param('~max_steer', 0.8)

        ns = rospy.get_param('~robot_ns', '/unicorn')

        self.left_steer_pub = rospy.Publisher(
            ns + '/left_steering_controller/command', Float64, queue_size=1)
        self.right_steer_pub = rospy.Publisher(
            ns + '/right_steering_controller/command', Float64, queue_size=1)
        self.lf_wheel_pub = rospy.Publisher(
            ns + '/left_front_wheel_controller/command', Float64, queue_size=1)
        self.rf_wheel_pub = rospy.Publisher(
            ns + '/right_front_wheel_controller/command', Float64, queue_size=1)
        self.lr_wheel_pub = rospy.Publisher(
            ns + '/left_rear_wheel_controller/command', Float64, queue_size=1)
        self.rr_wheel_pub = rospy.Publisher(
            ns + '/right_rear_wheel_controller/command', Float64, queue_size=1)

        ackermann_topic = rospy.get_param('~ackermann_topic',
            '/vesc/low_level/ackermann_cmd_mux/output')

        self.last_cmd_time = rospy.Time.now()
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.2)


        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber(ackermann_topic, AckermannDriveStamped, self.ackermann_callback)
        rospy.Timer(rospy.Duration(0.05), self.timeout_callback)
        rospy.loginfo('twist_to_joints ready (also listening on %s)', ackermann_topic)

    def ackermann_callback(self, msg):
        self.last_cmd_time = rospy.Time.now()
        self._drive(msg.drive.speed, msg.drive.steering_angle)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = rospy.Time.now()
        self._drive(msg.linear.x, msg.angular.z)

    def timeout_callback(self, event):
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
            self._drive(0.0, 0.0)

    def _drive(self, linear_x, steer_angle):

        # Clamp steering
        steer_angle = max(-self.max_steer, min(self.max_steer, steer_angle))

        # Ackermann inner/outer wheel differentiation
        if abs(steer_angle) > 0.001:
            R = self.wheelbase / math.tan(abs(steer_angle))
            inner = math.atan(self.wheelbase / (R - self.track_width / 2))
            outer = math.atan(self.wheelbase / (R + self.track_width / 2))
            if steer_angle > 0:
                left_steer = inner
                right_steer = outer
            else:
                left_steer = -outer
                right_steer = -inner
        else:
            left_steer = 0.0
            right_steer = 0.0

        # Wheel velocity
        wheel_vel = linear_x / self.wheel_radius

        self.left_steer_pub.publish(Float64(left_steer))
        self.right_steer_pub.publish(Float64(right_steer))
        self.lf_wheel_pub.publish(Float64(wheel_vel))
        self.rf_wheel_pub.publish(Float64(wheel_vel))
        self.lr_wheel_pub.publish(Float64(wheel_vel))
        self.rr_wheel_pub.publish(Float64(wheel_vel))


if __name__ == '__main__':
    try:
        node = TwistToJoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
