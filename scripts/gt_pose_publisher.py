#!/usr/bin/env python3
"""
Publishes ground truth pose/odom from Gazebo and broadcasts map->base_link TF.
Listens to the TF broadcast and republishes as topics at the same rate.
"""
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry


class GTPosePublisher:
    def __init__(self):
        rospy.init_node('gt_pose_publisher')
        self.model_name = rospy.get_param('~model_name', 'unicorn')
        self.pose_pub = rospy.Publisher('/glim_ros/base_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.odom_pub = rospy.Publisher('/glim_ros/base_odom', Odometry, queue_size=1, tcp_nodelay=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.idx = None
        self.last_tf_stamp = rospy.Time(0)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb, queue_size=1, buff_size=2**24, tcp_nodelay=True)

    def cb(self, msg):
        if self.idx is None or self.idx >= len(msg.name) or msg.name[self.idx] != self.model_name:
            try:
                self.idx = msg.name.index(self.model_name)
            except ValueError:
                return

        stamp = rospy.Time.now()
        pose = msg.pose[self.idx]
        twist = msg.twist[self.idx]
        p = pose.position
        o = pose.orientation

        # TF: map -> base_link
        if stamp > self.last_tf_stamp:
            self.last_tf_stamp = stamp

            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = 'map'
            tf_msg.child_frame_id = 'base_link'
            tf_msg.transform.translation.x = p.x
            tf_msg.transform.translation.y = p.y
            tf_msg.transform.translation.z = p.z
            tf_msg.transform.rotation = o
            self.tf_broadcaster.sendTransform(tf_msg)

            # PoseStamped - same timing as TF
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = 'map'
            ps.pose = pose
            self.pose_pub.publish(ps)

            # Odometry - same timing as TF
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'map'
            odom.child_frame_id = 'base_link'
            odom.pose.pose = pose
            odom.twist.twist = twist
            self.odom_pub.publish(odom)


if __name__ == '__main__':
    GTPosePublisher()
    rospy.spin()
