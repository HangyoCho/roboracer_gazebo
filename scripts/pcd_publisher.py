#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import os


def read_pcd(filepath):
    points = []
    with open(filepath, 'rb') as f:
        header_done = False
        num_points = 0
        data_type = 'ascii'
        fields = []
        while not header_done:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('FIELDS'):
                fields = line.split()[1:]
            elif line.startswith('POINTS'):
                num_points = int(line.split()[1])
            elif line.startswith('DATA'):
                data_type = line.split()[1]
                header_done = True

        if data_type == 'ascii':
            for _ in range(num_points):
                line = f.readline().decode('ascii', errors='ignore').strip()
                if line:
                    vals = line.split()
                    x, y, z = float(vals[0]), float(vals[1]), float(vals[2])
                    intensity = float(vals[3]) if len(vals) > 3 else 0.0
                    points.append((x, y, z, intensity))
        elif data_type == 'binary':
            for _ in range(num_points):
                data = f.read(4 * len(fields))
                if len(data) < 4 * len(fields):
                    break
                vals = struct.unpack('f' * len(fields), data)
                x, y, z = vals[0], vals[1], vals[2]
                intensity = vals[3] if len(vals) > 3 else 0.0
                points.append((x, y, z, intensity))
    return points


def main():
    rospy.init_node('pcd_publisher', anonymous=True)
    pcd_file = rospy.get_param('~pcd_file', '')
    topic = rospy.get_param('~topic', '/pcd_map')
    frame_id = rospy.get_param('~frame_id', 'map')
    rate_hz = rospy.get_param('~rate', 1.0)

    if not os.path.exists(pcd_file):
        rospy.logerr("PCD file not found: %s", pcd_file)
        return

    rospy.loginfo("Loading PCD: %s", pcd_file)
    points = read_pcd(pcd_file)
    rospy.loginfo("Loaded %d points", len(points))

    pub = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)

    header = Header()
    header.frame_id = frame_id
    header.stamp = rospy.Time.now()

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]

    cloud_msg = pc2.create_cloud(header, fields, points)
    cloud_msg.header.stamp = rospy.Time.now()
    pub.publish(cloud_msg)
    rospy.loginfo("Published PCD map on %s (latched)", topic)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
