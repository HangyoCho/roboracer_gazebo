#!/usr/bin/env python3
"""
Spawn/delete static obstacles in Gazebo.
- RViz 'Publish Point' to spawn a box obstacle.
- Interactive marker 'DELETE ALL' button in RViz to delete all obstacles.
- Joystick button 7 (Start) also deletes all.
- Publishes obstacle list as /obstacles (geometry_msgs/PoseArray)
  and /obstacles/radii (std_msgs/Float64MultiArray)
"""
import math
import threading
import rospy
from geometry_msgs.msg import PointStamped, Pose, PoseArray, PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback, MarkerArray


OBSTACLE_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>1000</mass>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{sx} {sy} {sz}</size></box>
        </geometry>
        <material>
          <ambient>1 0.2 0.2 1</ambient>
          <diffuse>1 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class ObstacleSpawner:
    def __init__(self):
        rospy.init_node('obstacle_spawner')

        self.size_x = rospy.get_param('~size_x', 0.3)
        self.size_y = rospy.get_param('~size_y', 0.3)
        self.size_z = rospy.get_param('~size_z', 0.5)
        self.delete_button = rospy.get_param('~delete_button', 7)

        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10)
        rospy.wait_for_service('/gazebo/delete_model', timeout=10)
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel, persistent=True)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel, persistent=True)

        self.obstacles = []  # list of (name, x, y, radius)
        self.count = 0
        self.radius = math.sqrt(self.size_x**2 + self.size_y**2) / 2.0

        # Publishers for obstacle list
        self.obs_pose_pub = rospy.Publisher('/gazebo/static_obstacles/poses', PoseArray, queue_size=1, latch=True)
        self.obs_radii_pub = rospy.Publisher('/gazebo/static_obstacles/radii', Float64MultiArray, queue_size=1, latch=True)
        self.obs_marker_pub = rospy.Publisher('/gazebo/static_obstacles/markers', MarkerArray, queue_size=1, latch=True)

        # Interactive marker server for delete button
        self.server = InteractiveMarkerServer("obstacle_manager")
        self._make_delete_button()

        rospy.Subscriber('/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('/joy', Joy, self.joy_cb)
        rospy.loginfo("obstacle_spawner ready: Publish Point=spawn, click DELETE ALL=delete")
        rospy.loginfo("Obstacle size: %.0fcm x %.0fcm x %.0fcm, radius=%.3fm",
                      self.size_x*100, self.size_y*100, self.size_z*100, self.radius)

    def _make_delete_button(self):
        im = InteractiveMarker()
        im.header.frame_id = "map"
        im.name = "delete_all_obstacles"
        im.description = "DELETE ALL"
        im.pose.position.x = 0.0
        im.pose.position.y = 0.0
        im.pose.position.z = 3.0
        im.scale = 1.5

        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = 1.5
        marker.scale.y = 1.5
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(marker)
        im.controls.append(control)

        self.server.insert(im, self._delete_cb)
        self.server.applyChanges()

    def _delete_cb(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.delete_all()

    def goal_cb(self, msg):
        self.count += 1
        name = f"obstacle_{self.count}"
        x = msg.pose.position.x
        y = msg.pose.position.y
        ori = msg.pose.orientation
        threading.Thread(target=self._spawn_obstacle, args=(name, x, y, ori), daemon=True).start()

    def _spawn_obstacle(self, name, x, y, ori):
        sdf = OBSTACLE_SDF.format(
            name=name,
            sx=self.size_x, sy=self.size_y, sz=self.size_z
        )
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 1.0
        pose.orientation = ori
        try:
            resp = self.spawn_srv(name, sdf, "", pose, "world")
            if resp.success:
                self.obstacles.append((name, x, y, self.radius))
                rospy.loginfo("Spawned '%s' at (%.2f, %.2f) r=%.3f", name, x, y, self.radius)
                self._publish_obstacles()
            else:
                rospy.logerr("Spawn failed: %s", resp.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn service error: %s", e)

    def joy_cb(self, msg):
        if len(msg.buttons) > self.delete_button and msg.buttons[self.delete_button]:
            self.delete_all()

    def _publish_obstacles(self):
        # PoseArray: centers
        pa = PoseArray()
        pa.header.frame_id = "map"
        pa.header.stamp = rospy.Time.now()
        radii = Float64MultiArray()
        ma = MarkerArray()

        for i, (name, x, y, r) in enumerate(self.obstacles):
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.orientation.w = 1.0
            pa.poses.append(p)
            radii.data.append(r)

            # Circle marker for rviz
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "obstacles"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = r * 2
            m.scale.y = r * 2
            m.scale.z = 0.2
            m.color.r = 1.0
            m.color.g = 0.3
            m.color.b = 0.0
            m.color.a = 0.5
            ma.markers.append(m)

        self.obs_pose_pub.publish(pa)
        self.obs_radii_pub.publish(radii)
        self.obs_marker_pub.publish(ma)

    def delete_all(self):
        if not self.obstacles:
            rospy.loginfo("No obstacles to delete")
            return
        rospy.loginfo("Deleting %d obstacles", len(self.obstacles))
        threading.Thread(target=self._delete_all_thread, daemon=True).start()

    def _delete_all_thread(self):
        # Delete from Gazebo
        for name, _, _, _ in self.obstacles:
            try:
                self.delete_srv(name)
            except rospy.ServiceException:
                pass

        # Clear rviz markers - send DELETE for each, then DELETEALL
        ma = MarkerArray()
        for i in range(len(self.obstacles)):
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "obstacles"
            m.id = i
            m.action = Marker.DELETE
            ma.markers.append(m)
        m_all = Marker()
        m_all.header.frame_id = "map"
        m_all.ns = "obstacles"
        m_all.action = Marker.DELETEALL
        ma.markers.append(m_all)
        self.obs_marker_pub.publish(ma)

        n = len(self.obstacles)
        self.obstacles.clear()
        rospy.loginfo("All %d obstacles deleted", n)


if __name__ == '__main__':
    ObstacleSpawner()
    rospy.spin()
