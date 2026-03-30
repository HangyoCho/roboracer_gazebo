#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from copy import deepcopy


class SimpleMuxNode:

    def __init__(self):
        self.name = "simple_mux"
        rospy.init_node(self.name, anonymous=True)

        self.out_topic = rospy.get_param("~out_topic", "/vesc/low_level/ackermann_cmd_mux/output")
        self.in_topic = rospy.get_param("~in_topic", "/vesc/high_level/ackermann_cmd_mux/input/nav_1")
        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.rate_hz = rospy.get_param("~rate_hz", 50.0)
        self.max_speed = rospy.get_param("~max_speed", 6.0)
        self.max_steer = rospy.get_param("~max_steer", 1.08)
        self.joy_freshness_threshold = rospy.get_param("~joy_freshness_threshold", 1.0)

        self.current_host = "autodrive"
        self.human_drive = None
        self.autodrive = None
        self.zero_msg = AckermannDriveStamped()
        self.zero_msg.header.stamp = rospy.Time.now()
        self.zero_msg.drive.steering_angle = 0
        self.zero_msg.drive.speed = 0

        rospy.Subscriber(self.in_topic, AckermannDriveStamped, self.drive_callback)
        rospy.Subscriber(self.joy_topic, Joy, self.joy_callback)

        self.drive_pub = rospy.Publisher(self.out_topic, AckermannDriveStamped, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.timer_callback)
        rospy.loginfo("simple_mux ready: joy(LB=manual, RB=auto) -> %s", self.out_topic)

    def check_uptodate(self, drive_msg):
        if drive_msg is None:
            return False
        if abs(drive_msg.header.stamp.to_sec() - rospy.Time.now().to_sec()) < self.joy_freshness_threshold:
            return True
        return False

    def timer_callback(self, event):
        if self.current_host is None:
            return
        if self.current_host == "autodrive" and self.check_uptodate(self.autodrive):
            self.drive_pub.publish(self.autodrive)
        elif self.current_host == "humandrive" and self.check_uptodate(self.human_drive):
            self.drive_pub.publish(self.human_drive)

    def joy_callback(self, msg):
        use_human_drive = msg.buttons[4]  # LB
        use_controller = msg.buttons[5]   # RB

        if use_human_drive:
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.drive.steering_angle = msg.axes[3] * self.max_steer
            drive_msg.drive.speed = msg.axes[1] * self.max_speed
            self.human_drive = drive_msg
            self.current_host = "humandrive"
        elif use_controller:
            self.current_host = "autodrive"

    def drive_callback(self, msg):
        self.autodrive = msg


if __name__ == '__main__':
    simple_mux = SimpleMuxNode()
    rospy.spin()
