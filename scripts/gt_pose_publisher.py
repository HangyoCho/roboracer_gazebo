#!/usr/bin/env python3
"""
Publishes ground truth poses from Gazebo:
  - map->base_link TF + base_pose/base_odom (rear axle)
  - imu_pose/imu_odom (at livox_mid360/imu frame)
"""
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Point
from nav_msgs.msg import Odometry


def quat_rotate(q, v):
    """Rotate vector v by quaternion q = (x,y,z,w)"""
    x, y, z, w = q
    vx, vy, vz = v
    # q * v * q_inv
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


class GTPosePublisher:
    def __init__(self):
        rospy.init_node('gt_pose_publisher')
        self.model_name = rospy.get_param('~model_name', 'unicorn')
        # base_link -> livox_mid360 offset
        self.lidar_offset = (
            rospy.get_param('~lidar_offset_x', 0.26),
            rospy.get_param('~lidar_offset_y', 0.0),
            rospy.get_param('~lidar_offset_z', 0.08),
        )

        # base_link publishers
        self.base_pose_pub = rospy.Publisher('/glim_ros/base_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.base_odom_pub = rospy.Publisher('/glim_ros/base_odom', Odometry, queue_size=1, tcp_nodelay=True)
        # imu/lidar frame publishers
        self.imu_pose_pub = rospy.Publisher('/glim_ros/imu_pose', PoseStamped, queue_size=1, tcp_nodelay=True)
        self.imu_odom_pub = rospy.Publisher('/glim_ros/imu_odom', Odometry, queue_size=1, tcp_nodelay=True)

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
        if stamp <= self.last_tf_stamp:
            return
        self.last_tf_stamp = stamp

        pose = msg.pose[self.idx]
        twist = msg.twist[self.idx]
        p = pose.position
        o = pose.orientation
        q = (o.x, o.y, o.z, o.w)

        # --- base_link (rear axle) ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = p.x
        tf_msg.transform.translation.y = p.y
        tf_msg.transform.translation.z = p.z
        tf_msg.transform.rotation = o
        self.tf_broadcaster.sendTransform(tf_msg)

        base_ps = PoseStamped()
        base_ps.header.stamp = stamp
        base_ps.header.frame_id = 'map'
        base_ps.pose = pose
        self.base_pose_pub.publish(base_ps)

        base_odom = Odometry()
        base_odom.header.stamp = stamp
        base_odom.header.frame_id = 'map'
        base_odom.child_frame_id = 'base_link'
        base_odom.pose.pose = pose
        base_odom.twist.twist = twist
        self.base_odom_pub.publish(base_odom)

        # --- imu/lidar frame (livox_mid360) ---
        dx, dy, dz = quat_rotate(q, self.lidar_offset)
        imu_x = p.x + dx
        imu_y = p.y + dy
        imu_z = p.z + dz

        imu_ps = PoseStamped()
        imu_ps.header.stamp = stamp
        imu_ps.header.frame_id = 'map'
        imu_ps.pose.position.x = imu_x
        imu_ps.pose.position.y = imu_y
        imu_ps.pose.position.z = imu_z
        imu_ps.pose.orientation = o
        self.imu_pose_pub.publish(imu_ps)

        imu_odom = Odometry()
        imu_odom.header.stamp = stamp
        imu_odom.header.frame_id = 'map'
        imu_odom.child_frame_id = 'imu'
        imu_odom.pose.pose = imu_ps.pose
        imu_odom.twist.twist = twist
        self.imu_odom_pub.publish(imu_odom)


if __name__ == '__main__':
    GTPosePublisher()
    rospy.spin()
