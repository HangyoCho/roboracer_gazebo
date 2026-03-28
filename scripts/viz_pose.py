#!/usr/bin/env python3
"""
Real-time 2D visualization comparing TF (map->base_link) vs base_pose topic.
"""
import rospy
import tf2_ros
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from collections import deque

MAXLEN = 500

tf_x, tf_y = deque(maxlen=MAXLEN), deque(maxlen=MAXLEN)
pose_x, pose_y = deque(maxlen=MAXLEN), deque(maxlen=MAXLEN)
odom_vx, odom_vy = deque(maxlen=MAXLEN), deque(maxlen=MAXLEN)
odom_t = deque(maxlen=MAXLEN)
err_hist = deque(maxlen=MAXLEN)
err_t = deque(maxlen=MAXLEN)
max_err = 0.0
err_count = 0

tf_buffer = None


def pose_cb(msg):
    global max_err, err_count
    pose_x.append(msg.pose.position.x)
    pose_y.append(msg.pose.position.y)
    try:
        tf = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
        tf_x.append(tf.transform.translation.x)
        tf_y.append(tf.transform.translation.y)
        dx = msg.pose.position.x - tf.transform.translation.x
        dy = msg.pose.position.y - tf.transform.translation.y
        err = (dx**2 + dy**2)**0.5
        err_hist.append(err)
        err_t.append(msg.header.stamp.to_sec())
        if err > max_err:
            max_err = err
        err_count += 1
        if err_count % 50 == 0:
            avg_err = sum(err_hist) / len(err_hist)
            print(f'[{err_count:5d}] err={err:.4f}m  max={max_err:.4f}m  avg={avg_err:.4f}m')
    except:
        pass


def odom_cb(msg):
    odom_vx.append(msg.twist.twist.linear.x)
    odom_vy.append(msg.twist.twist.linear.y)
    odom_t.append(msg.header.stamp.to_sec())


def main():
    global tf_buffer
    rospy.init_node('viz_pose', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    rospy.Subscriber('/glim_ros/base_pose', PoseStamped, pose_cb, queue_size=1)
    rospy.Subscriber('/glim_ros/base_odom', Odometry, odom_cb, queue_size=1)

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    while not rospy.is_shutdown():
        # Trajectory plot
        ax1.clear()
        if tf_x:
            ax1.plot(list(tf_x), list(tf_y), 'b-', linewidth=2, label='TF')
            ax1.plot(tf_x[-1], tf_y[-1], 'bo', markersize=8)
        if pose_x:
            ax1.plot(list(pose_x), list(pose_y), 'r--', linewidth=1.5, label='base_pose')
            ax1.plot(pose_x[-1], pose_y[-1], 'ro', markersize=8)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Trajectory: TF vs base_pose')
        ax1.legend()
        ax1.set_aspect('equal')
        ax1.grid(True)
        ax1.set_title(f'Trajectory (max_err={max_err:.4f}m)')

        # Velocity plot
        ax2.clear()
        if odom_vx and odom_t:
            n = min(len(odom_t), len(odom_vx), len(odom_vy))
            t0 = odom_t[0]
            ts = [odom_t[i] - t0 for i in range(n)]
            ax2.plot(ts, [odom_vx[i] for i in range(n)], 'g-', label='vx')
            ax2.plot(ts, [odom_vy[i] for i in range(n)], 'm-', label='vy')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('base_odom twist')
        ax2.legend()
        ax2.grid(True)

        # Error plot (replace velocity when error data exists)
        if err_hist and err_t:
            ax2.clear()
            n = min(len(err_t), len(err_hist))
            t0 = err_t[0]
            ts = [err_t[i] - t0 for i in range(n)]
            ax2.plot(ts, [err_hist[i]*100 for i in range(n)], 'r-', label='error')
            ax2.axhline(y=max_err*100, color='k', linestyle='--', label=f'max={max_err*100:.2f}cm')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Error (cm)')
            ax2.set_title('TF vs base_pose error')
            ax2.legend()
            ax2.grid(True)

        plt.tight_layout()
        plt.pause(0.1)

    plt.ioff()
    plt.show()


if __name__ == '__main__':
    main()
