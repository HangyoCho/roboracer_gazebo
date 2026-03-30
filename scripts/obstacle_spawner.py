#!/usr/bin/env python3
"""
Spawn/delete static obstacles in Gazebo.
- RViz '2D Nav Goal' to spawn a box obstacle at the pointed location.
- Interactive marker 'DELETE ALL' button in RViz to delete all obstacles.
- Joystick button 7 (Start) also deletes all.
- Tracks actual obstacle poses from /gazebo/model_states.
- Publishes:
    /gazebo/static_obstacles/poses   (PoseArray)
    /gazebo/static_obstacles/radii   (Float64MultiArray)
    /gazebo/static_obstacles/markers (MarkerArray) — real size + real pose
"""
import math
import time
import threading
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnModel, DeleteModel
from std_srvs.srv import Empty
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (
    InteractiveMarker, InteractiveMarkerControl, Marker,
    InteractiveMarkerFeedback, MarkerArray,
)


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

        # Wait for sim time to be valid
        while rospy.Time.now() == rospy.Time(0) and not rospy.is_shutdown():
            time.sleep(0.1)

        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=10)
        rospy.wait_for_service('/gazebo/delete_model', timeout=10)
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.pause_srv = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

        self.lock = threading.Lock()
        self.obstacles = {}  # name -> {sx, sy, sz, radius}
        self.obstacle_poses = {}  # name -> Pose (tracked from Gazebo)
        self.count = 0
        self.deleting = False

        # Publishers
        self.obs_pose_pub = rospy.Publisher('/gazebo/static_obstacles/poses', PoseArray, queue_size=1, latch=True)
        self.obs_radii_pub = rospy.Publisher('/gazebo/static_obstacles/radii', Float64MultiArray, queue_size=1, latch=True)
        self.obs_marker_pub = rospy.Publisher('/gazebo/static_obstacles/markers', MarkerArray, queue_size=1, latch=True)

        # Interactive marker server for delete button
        self.server = InteractiveMarkerServer("obstacle_manager")
        self._make_delete_button()

        # Subscribers
        rospy.Subscriber('/goal', PoseStamped, self.goal_cb, queue_size=10, tcp_nodelay=True)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb, queue_size=10, tcp_nodelay=True)
        rospy.Subscriber('/joy', Joy, self.joy_cb)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_cb,
                         queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("obstacle_spawner ready: 2D Nav Goal=spawn, DELETE ALL=delete")
        rospy.loginfo("Obstacle size: %.0fcm x %.0fcm x %.0fcm",
                      self.size_x * 100, self.size_y * 100, self.size_z * 100)

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

    # ---- Spawn ----

    def goal_cb(self, msg):
        rospy.loginfo("goal_cb called: (%.2f, %.2f)", msg.pose.position.x, msg.pose.position.y)
        with self.lock:
            if self.deleting:
                rospy.logwarn("Delete in progress, ignoring spawn request")
                return
            self.count += 1
            name = f"obstacle_{self.count}"
        x = msg.pose.position.x
        y = msg.pose.position.y
        ori = msg.pose.orientation
        threading.Thread(target=self._spawn_obstacle, args=(name, x, y, ori), daemon=True).start()

    def _spawn_obstacle(self, name, x, y, ori):
        rospy.loginfo("_spawn_obstacle: calling spawn_srv for '%s'", name)
        sdf = OBSTACLE_SDF.format(
            name=name,
            sx=self.size_x, sy=self.size_y, sz=self.size_z
        )
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 1.0  # drop from above
        pose.orientation = ori
        try:
            resp = self.spawn_srv(name, sdf, "", pose, "world")
            rospy.loginfo("_spawn_obstacle: resp.success=%s", resp.success)
            if resp.success:
                radius = math.sqrt(self.size_x**2 + self.size_y**2) / 2.0
                with self.lock:
                    self.obstacles[name] = {
                        'sx': self.size_x, 'sy': self.size_y, 'sz': self.size_z,
                        'radius': radius,
                    }
                rospy.loginfo("Spawned '%s' at (%.2f, %.2f)", name, x, y)
            else:
                rospy.logerr("Spawn failed: %s", resp.status_message)
        except rospy.ServiceException as e:
            rospy.logerr("Spawn service error: %s", e)

    # ---- Track from Gazebo ----

    def model_states_cb(self, msg):
        with self.lock:
            if self.deleting or not self.obstacles:
                return
            updated = False
            for i, name in enumerate(msg.name):
                if name in self.obstacles:
                    self.obstacle_poses[name] = msg.pose[i]
                    updated = True
            if updated:
                self._publish_obstacles()

    # ---- Publish ----

    def _publish_obstacles(self):
        """Must be called with self.lock held."""
        pa = PoseArray()
        pa.header.frame_id = "map"
        pa.header.stamp = rospy.Time.now()
        radii = Float64MultiArray()
        ma = MarkerArray()

        for i, (name, info) in enumerate(self.obstacles.items()):
            pose = self.obstacle_poses.get(name)
            if pose is None:
                continue

            pa.poses.append(pose)
            radii.data.append(info['radius'])

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = pa.header.stamp
            m.ns = "obstacles"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = pose
            m.scale.x = info['sx']
            m.scale.y = info['sy']
            m.scale.z = info['sz']
            m.color.r = 1.0
            m.color.g = 0.3
            m.color.b = 0.0
            m.color.a = 0.6
            ma.markers.append(m)

        self.obs_pose_pub.publish(pa)
        self.obs_radii_pub.publish(radii)
        self.obs_marker_pub.publish(ma)

    # ---- Delete ----

    def joy_cb(self, msg):
        if len(msg.buttons) > self.delete_button and msg.buttons[self.delete_button]:
            self.delete_all()

    def delete_all(self):
        with self.lock:
            if not self.obstacles:
                rospy.loginfo("No obstacles to delete")
                return
            names = list(self.obstacles.keys())
            self.deleting = True
            self.obstacles.clear()
            self.obstacle_poses.clear()
            self.count = 0

        rospy.loginfo("Deleting %d obstacles", len(names))
        threading.Thread(target=self._delete_all_thread, args=(names,), daemon=True).start()

    def _delete_all_thread(self, names):
        # Delete from Gazebo (retry once if needed)
        for name in names:
            for attempt in range(2):
                try:
                    resp = self.delete_srv(name)
                    if resp.success:
                        break
                    time.sleep(0.05)
                except rospy.ServiceException:
                    time.sleep(0.05)

        # Clear markers: DELETEALL then publish empty
        ma = MarkerArray()
        m_all = Marker()
        m_all.header.frame_id = "map"
        m_all.ns = "obstacles"
        m_all.action = Marker.DELETEALL
        ma.markers.append(m_all)
        self.obs_marker_pub.publish(ma)
        time.sleep(0.1)
        self.obs_marker_pub.publish(MarkerArray())

        # Publish empty topics
        pa = PoseArray()
        pa.header.frame_id = "map"
        pa.header.stamp = rospy.Time.now()
        self.obs_pose_pub.publish(pa)
        self.obs_radii_pub.publish(Float64MultiArray())

        with self.lock:
            self.deleting = False

        rospy.loginfo("All %d obstacles deleted", len(names))


if __name__ == '__main__':
    ObstacleSpawner()
    rospy.spin()
