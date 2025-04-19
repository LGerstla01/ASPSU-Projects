# visualizer.py

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Point
import random

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualizer')
        self.declare_parameter('num_robots', 1)
        self.declare_parameter('num_cameras', 2)
        self.declare_parameter('FOV', 60.0)  # Field of view in degrees
        self.declare_parameter('room_size', [40.0, 30.0, 3.0])  # [width, height]

        self.num_robots = self.get_parameter('num_robots').value
        self.num_cameras = self.get_parameter('num_cameras').value
        self.room_size = self.get_parameter('room_size').value
        self.FOV = np.radians(self.get_parameter('FOV').value)  # Convert to radians

        self.robot_sub = self.create_subscription(PoseArray, '/robots/pose', self.robot_callback, 10)

        self.robot_sub = self.create_subscription(PoseArray, '/camera_2/detections', self.robot_estimation_callback, 10)



        # Example: Add subscriptions or other ROS2 functionality here
        self.room_pub = self.create_publisher(MarkerArray, '/visualize/room', 10)
        self.robot_pub = self.create_publisher(MarkerArray, '/visualize/robots', 10)
        self.robot_estimated_pub = self.create_publisher(MarkerArray, '/visualize/robots_estimated', 10)

        self.create_timer(1.0, self.publish_room_markers)
        #self.pub_robots(rand=True)

    def robot_callback(self, msg, estimation=False):
        self.pub_robots(pos=msg)

    def robot_estimation_callback(self, msg, estimation=True):
        self.pub_estimated_robots(pos=msg)

    def publish_room_markers(self):
        marker_array = MarkerArray()
        # floor
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.ns = "floor"
        marker.id = 0
        marker.scale.x = self.room_size[0]
        marker.scale.y = self.room_size[1]
        marker.scale.z = 0.1
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = -marker.scale.z / 2
        marker_array.markers.append(marker)

        # Walls
        walls = [{"pos:": (self.room_size[0]/2 + 0.05, 0.0, self.room_size[2]/2), "dim": (0.1, self.room_size[1], self.room_size[2])},
                 {"pos:": (-self.room_size[0]/2 - 0.05, 0.0, self.room_size[2]/2), "dim": (0.1, self.room_size[1], self.room_size[2])},
                 {"pos:": (0.0, self.room_size[1]/2 + 0.05, self.room_size[2]/2), "dim": (self.room_size[0], 0.1, self.room_size[2])},
                 {"pos:": (0.0, -self.room_size[1]/2 - 0.05, self.room_size[2]/2), "dim": (self.room_size[0], 0.1, self.room_size[2])}]

        for wall in walls:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.ns = "wall"
            marker.id = len(marker_array.markers) + 1
            marker.scale.x = wall["dim"][0]
            marker.scale.y = wall["dim"][1]
            marker.scale.z = wall["dim"][2]
            marker.pose.position.x = wall["pos:"][0]
            marker.pose.position.y = wall["pos:"][1]
            marker.pose.position.z = wall["pos:"][2]
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        # Cameras
        camera_size = 0.5, 0.5, 0.5
        cameras = [{"pos:": (self.room_size[0]/2, self.room_size[1]/2, self.room_size[2]), "dim": (camera_size[0], camera_size[1], camera_size[2])},
                   {"pos:": (-self.room_size[0]/2, self.room_size[1]/2, self.room_size[2]), "dim": (camera_size[0], camera_size[1], camera_size[2])},
                   {"pos:": (-self.room_size[0]/2, -self.room_size[1]/2, self.room_size[2]), "dim": (camera_size[0], camera_size[1], camera_size[2])},
                   {"pos:": (self.room_size[0]/2, -self.room_size[1]/2, self.room_size[2]), "dim": (camera_size[0], camera_size[1], camera_size[2])},]

        for i in range(self.num_cameras):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.id = len(marker_array.markers) + 1
            marker.ns = "camera"
            marker.scale.x = cameras[i]["dim"][0]
            marker.scale.y = cameras[i]["dim"][1]
            marker.scale.z = cameras[i]["dim"][2]
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = cameras[i]["pos:"][0]
            marker.pose.position.y = cameras[i]["pos:"][1]
            marker.pose.position.z = cameras[i]["pos:"][2]
            marker_array.markers.append(marker)

            marker_triangle = Marker()
            marker_triangle.header.frame_id = "map"
            marker_triangle.ns = "FOV"
            marker_triangle.type = Marker.TRIANGLE_LIST
            marker_triangle.action = Marker.ADD
            marker_triangle.id = len(marker_array.markers) + 1
            marker_triangle.points = self.points_triangle(i, marker.pose.position)
            marker_triangle.scale.x = 1.0
            marker_triangle.scale.y = 1.0
            marker_triangle.scale.z = 1.0
            marker_triangle.color.r = 0.0
            marker_triangle.color.g = 1.0
            marker_triangle.color.b = 0.0
            marker_triangle.color.a = 0.5
            marker_array.markers.append(marker_triangle)

        self.room_pub.publish(marker_array)

    def pub_robots(self, rand=False, pos=None, estimation=False):
        marker_array = MarkerArray()
        for i in range(self.num_robots):
            marker = Marker()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id = 10 + i
            marker.scale.x = 1.0
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            if estimation:
                marker.ns = "robot_estimated"
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.ns = "robot"
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
            if rand:
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.pose.position.x = float(random.randrange(int(-self.room_size[0]//2), int(self.room_size[0]//2)))
                marker.pose.position.y = float(random.randrange(int(-self.room_size[1]//2), int(self.room_size[1]//2)))
                marker.pose.position.z = marker.scale.z / 2
            else:
                marker.header = pos.header
                marker.pose = pos.poses[i]

            marker_array.markers.append(marker)

        self.robot_pub.publish(marker_array)

    def pub_estimated_robots(self, pos=None):
        marker_array = MarkerArray()
        for i in range(len(pos.poses)):
            if pos.poses[i].position.x < 9999:
                marker = Marker()
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.id = i
                marker.scale.x = 1.0
                marker.scale.y = 0.5
                marker.scale.z = 0.5

                marker.ns = "robot_estimated"
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.header = pos.header
                marker.pose.position = pos.poses[i].position

                marker_array.markers.append(marker)

        if len(marker_array.markers) > 0:
            self.robot_estimated_pub.publish(marker_array)

        

    def points_triangle(self, i, camera_pos):
        #camera_angle = np.radians(180) + self.normalize_angle(np.arctan2(camera_pos.y, camera_pos.x) + np.pi)
        #camera_angle = np.radians(225)
        
        if i == 0:
            camera_angle = np.radians(225)
        elif i == 1:
            camera_angle = np.radians(135)
        elif i == 2:
            camera_angle = np.radians(45)
        elif i == 3:
            camera_angle = np.radians(315)

        point1 = Point()
        point1.x = camera_pos.x
        point1.y = camera_pos.y
        point1.z = camera_pos.z

        point2 = Point()
        point2.x = camera_pos.x + self.room_size[0] * np.sin(camera_angle + self.FOV / 2)
        point2.y = camera_pos.y + self.room_size[0] * np.cos(camera_angle + self.FOV / 2)
        point2.z = camera_pos.z

        point3 = Point()
        point3.x = camera_pos.x + self.room_size[0] * np.sin(camera_angle - self.FOV / 2)
        point3.y = camera_pos.y + self.room_size[0] * np.cos(camera_angle - self.FOV / 2)
        point3.z = camera_pos.z
    
        return point1, point2, point3

    def normalize_angle(self, angle):
        # Normalize the angle to be within -pi to pi
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
def main(args=None):
    rclpy.init(args=args)
    visualizer = VisualizationNode()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()