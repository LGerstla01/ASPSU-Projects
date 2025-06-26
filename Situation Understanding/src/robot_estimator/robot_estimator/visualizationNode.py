import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Point
from nav_msgs.msg import Odometry # NEU: Import für Odometrie-Nachrichten
import random

class VisualizationNode(Node):
    """
    A ROS2 node for visualizing the environment, robots, and sensor data using RViz markers.
    This node publishes markers for the room, robots, cameras, lidar sensors, and EKF tracks.

    Attributes:
        num_robots (int): Number of robots in the simulation.
        num_cameras (int): Number of cameras in the room.
        FOV (float): Field of view of the cameras in radians.
        num_lidar (int): Number of lidar sensors in the room.
        angle_lidar (float): Angular resolution of lidar rays in degrees.
        room_size (list): Dimensions of the room [width, height, height].
        ekf_data (dict): Stores the latest EKF poses for each robot.
    """
    def __init__(self):
        """
        Initializes the visualization node, sets parameters, and creates publishers and subscriptions.
        """
        super().__init__('visualizer')
        # Declare parameters for room size, robots, cameras, and lidar
        self.declare_parameter('num_robots', 1)
        self.declare_parameter('num_cameras', 2)
        self.declare_parameter('FOV', 50.0)  # Field of view in degrees
<<<<<<< HEAD
        self.declare_parameter('num_lidar', 1)
        self.declare_parameter('angle_lidar', 10)
        self.declare_parameter('room_size', [40.0, 30.0, 3.0])  # [width, height, height]
=======
        self.declare_parameter('num_lidar', 1)  # Field of view in degrees
        self.declare_parameter('angle_lidar', 10)
        self.declare_parameter('room_size', [40.0, 30.0, 3.0])  # [width, height]
>>>>>>> b1ae4f4faf8a6400a5c7459e37df1de845e88f30

        # Retrieve parameter values
        self.num_robots = self.get_parameter('num_robots').value
        self.num_cameras = self.get_parameter('num_cameras').value
        self.room_size = self.get_parameter('room_size').value
        self.num_lidar = self.get_parameter('num_lidar').value
        self.angle_lidar = self.get_parameter('angle_lidar').value
        self.FOV = np.radians(self.get_parameter('FOV').value)  # Convert FOV to radians

        # Create subscriptions for robot positions and EKF odometry
        self.robot_sub = self.create_subscription(PoseArray, '/robots/pose', self.robot_callback, 10)
        self.ekf_sub_0 = self.create_subscription(Odometry, '/ekf/robot_0/odom', self.ekf_callback, 10)
        self.ekf_sub_1 = self.create_subscription(Odometry, '/ekf/robot_1/odom', self.ekf_callback, 10)

<<<<<<< HEAD
        # Create publishers for markers
=======
        # DEAKTIVIERT: Die alte Kameradetektions-Visualisierung, die rote Punkte erzeugt hat
        # self.robot_camera_sub = self.create_subscription(PoseArray, '/detections/camera_1', self.robot_camera_callback, 10)

        # self.robot_lidar_sub = self.create_subscription(PoseArray, '/detections/lidar_1', self.robot_lidar_callback, 10)


        # Example: Add subscriptions or other ROS2 functionality here
>>>>>>> b1ae4f4faf8a6400a5c7459e37df1de845e88f30
        self.room_pub = self.create_publisher(MarkerArray, '/visualize/room', 10)
        self.robot_pub = self.create_publisher(MarkerArray, '/visualize/robots', 10)
        self.robot_estimated_pub = self.create_publisher(MarkerArray, '/visualize/robots_estimated', 10)
        self.robots_lidar_pub = self.create_publisher(MarkerArray, '/visualize/lidar', 10)
        self.ekf_pub = self.create_publisher(MarkerArray, '/visualize/ekf_tracks', 10)

<<<<<<< HEAD
        # Initialize EKF data storage
        self.ekf_data = {}  # Stores the latest EKF poses for each robot

        # Create a timer to periodically publish room markers
=======
        # NEU: Publisher für EKF-Tracks
        self.ekf_pub = self.create_publisher(MarkerArray, '/visualize/ekf_tracks', 10)
        self.ekf_data = {} # Speichert die letzten EKF-Posen für jeden Roboter

        # NEU: Abonnements für EKF-Odometrie-Topics
        # Annahme: Der EKF veröffentlicht auf /ekf/robot_0/odom, /ekf/robot_1/odom, etc.
        # Und die child_frame_id der Odometrie-Nachricht ist 'robot_0', 'robot_1', etc.
        self.ekf_sub_0 = self.create_subscription(Odometry, '/ekf/robot_0/odom', self.ekf_callback, 10)
        self.ekf_sub_1 = self.create_subscription(Odometry, '/ekf/robot_1/odom', self.ekf_callback, 10)


>>>>>>> b1ae4f4faf8a6400a5c7459e37df1de845e88f30
        self.create_timer(1.0, self.publish_room_markers)

    def robot_callback(self, msg, estimation=False):
        """
        Callback function for robot pose messages. Publishes robot markers.

        Args:
            msg (PoseArray): The pose array message containing the robots' positions.
            estimation (bool): Flag indicating if the poses are estimations (default: False).
        """
        self.pub_robots(pos=msg)

    def robot_camera_callback(self, msg, estimation=True):
        """
        Callback function for camera detection messages. Publishes estimated robot markers.

        Args:
            msg (PoseArray): The pose array message containing the estimated robot positions.
            estimation (bool): Flag indicating if the poses are estimations (default: True).
        """
        self.pub_estimated_robots(pos=msg)

    # NEU: Callback-Funktion für EKF-Odometrie-Nachrichten
    def ekf_callback(self, msg):
<<<<<<< HEAD
        """
        Callback function for EKF odometry messages. Updates the EKF data and publishes markers.

        Args:
            msg (Odometry): The odometry message containing the robot's estimated pose.
        """
        # Extract robot ID from the child_frame_id (e.g., 'robot_0/base_link' -> 0)
=======
        # Korrigiert: Versuchen, die Roboter-ID aus der child_frame_id zu extrahieren (z.B. 'robot_0/base_link' -> 0)
        # Zuerst den Teil vor dem '/' nehmen, dann am '_' splitten
>>>>>>> b1ae4f4faf8a6400a5c7459e37df1de845e88f30
        parts = msg.child_frame_id.split('/')
        if len(parts) > 0 and parts[0].startswith('robot_'):
            try:
                robot_id_str = parts[0].split('_')[1]
                robot_id = int(robot_id_str)
<<<<<<< HEAD
                self.ekf_data[robot_id] = msg.pose.pose  # Store the estimated pose
                self.publish_ekf_markers()  # Update and publish EKF markers
=======
                self.ekf_data[robot_id] = msg.pose.pose # Speichern der Pose (enthält Position und Orientierung)
                self.publish_ekf_markers() # Marker aktualisieren und veröffentlichen
>>>>>>> b1ae4f4faf8a6400a5c7459e37df1de845e88f30
            except (IndexError, ValueError):
                self.get_logger().warn(f"Could not parse robot_id from child_frame_id: {msg.child_frame_id}")
        else:
            self.get_logger().warn(f"Unexpected child_frame_id format for EKF: {msg.child_frame_id}")


    # NEU: Funktion zum Veröffentlichen der EKF-Marker
    def publish_ekf_markers(self):
<<<<<<< HEAD
        """
        Publishes markers for EKF tracks, visualizing the estimated positions of robots.
        """
=======
        marker_array = MarkerArray()
        for robot_id, pose in self.ekf_data.items():
            marker = Marker()
            marker.header.frame_id = "map"  # Annahme: EKF-Posen sind im "map"-Frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE  # Visualisiere als Kugel (Punkt)
            marker.action = Marker.ADD
            marker.ns = "ekf_tracked_robots" # Namespace für diese Marker
            marker.id = robot_id  # Eindeutige ID für jeden Roboter-Marker
            marker.scale.x = 0.5  # Größe der Kugel
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 1.0  # Rot
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Volle Deckkraft
            marker.pose = pose    # Die geschätzte Pose des Roboters

            marker_array.markers.append(marker)

        if len(marker_array.markers) > 0:
            self.ekf_pub.publish(marker_array)

    def publish_room_markers(self):
>>>>>>> b1ae4f4faf8a6400a5c7459e37df1de845e88f30
        marker_array = MarkerArray()
        for robot_id, pose in self.ekf_data.items():
            marker = Marker()
            marker.header.frame_id = "map"  # Assume EKF poses are in the "map" frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE  # Visualize as a sphere (point)
            marker.action = Marker.ADD
            marker.ns = "ekf_tracked_robots"  # Namespace for EKF markers
            marker.id = robot_id  # Unique ID for each robot marker
            marker.scale.x = 0.5  # Size of the sphere
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 1.0  # Red color
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Full opacity
            marker.pose = pose    # The geschätzte Pose des Roboters

            marker_array.markers.append(marker)

        if len(marker_array.markers) > 0:
            self.ekf_pub.publish(marker_array)

    def publish_room_markers(self):
        """
        Publishes markers for the room, including the floor, walls, cameras, and lidar sensors.
        """
        marker_array = MarkerArray()

        # Floor marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.ns = "floor"
        marker.id = 0
        marker.scale.x = self.room_size[0]
        marker.scale.y = self.room_size[1]
        marker.scale.z = 0.1  # Thickness of the floor
        marker.color.r = 0.5  # Gray color
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = -marker.scale.z / 2  # Position the floor at the bottom
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
            marker_triangle.color.a = 0.3
            marker_array.markers.append(marker_triangle)

        # Lidar
        lidar_positions = [
            (0.0, -self.room_size[1] / 2 ,1.0),
            (0.0, self.room_size[1] / 2 ,1.0),
            ]
        
        for i in range(self.num_lidar):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.id = len(marker_array.markers) + 1
            marker.ns = "lidar"
            marker.points = self.points_lidar(i, lidar_positions[i], self.angle_lidar)
            marker.scale.x = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            #marker.pose.position.x = 0
            #marker.pose.position.y = 0
            #marker.pose.position.z = 1
            marker_array.markers.append(marker)


        self.room_pub.publish(marker_array)

    def pub_robots(self, rand=False, pos=None, estimation=False):
        """
        Publishes markers for the robots in the simulation.

        Args:
            rand (bool): Flag indicating if random positions should be used (default: False).
            pos (PoseArray): The pose array message containing the robots' positions (default: None).
            estimation (bool): Flag indicating if the poses are estimations (default: False).
        """
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
        """
        Publishes markers for the estimated robot positions.

        Args:
            pos (PoseArray): The pose array message containing the estimated robot positions.
        """
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

    def robot_lidar_callback(self, msg):
        """
        Callback function for lidar detection messages. Publishes markers for detected lidar points.

        Args:
            msg (PoseArray): The pose array message containing the lidar points' positions.
        """
        marker_array = MarkerArray()
        for i in range(len(msg.poses)):
            if msg.poses[i].position.x < 9999:
                marker = Marker()
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.id = i
                marker.scale.x = 1.0
                marker.scale.y = 0.5
                marker.scale.z = 0.5

                marker.ns = "robot_lidar"
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                marker.header = msg.header
                marker.pose.position = msg.poses[i].position

                marker_array.markers.append(marker)

        if len(marker_array.markers) > 0:
            self.robots_lidar_pub.publish(marker_array)

    def points_triangle(self, i, camera_pos):
        """
        Calculates the 3D points for the triangle representing the camera's field of view.

        Args:
            i (int): The index of the camera.
            camera_pos (Point): The position of the camera.

        Returns:
            list: A list of 3D points defining the triangle.
        """
        points = []
        if i == 0:
            camera_angle = np.radians(225)
        elif i == 1:
            camera_angle = np.radians(135)
        elif i == 2:
            camera_angle = np.radians(45)
        elif i == 3:
            camera_angle = np.radians(135)

        point1 = Point()
        point1.x = camera_pos.x
        point1.y = camera_pos.y
        point1.z = camera_pos.z
        points.append(point1)

        länge1 = np.abs((self.room_size[0]) / np.sin(camera_angle + self.FOV / 2))
        länge2 = np.abs((self.room_size[1]) / np.cos(camera_angle + self.FOV / 2))
        länge = länge1 if länge1 < länge2 else länge2

        point2 = Point()
        point2.x = camera_pos.x + länge * np.sin(camera_angle + self.FOV / 2)
        point2.y = camera_pos.y + länge * np.cos(camera_angle + self.FOV / 2)
        point2.z = camera_pos.z
        points.append(point2)

        länge1 = np.abs((self.room_size[0]) / np.sin(camera_angle - self.FOV / 2))
        länge2 = np.abs((self.room_size[1]) / np.cos(camera_angle - self.FOV / 2))
        länge = länge1 if länge1 < länge2 else länge2

        point3 = Point()
        point3.x = -camera_pos.x
        point3.y = -camera_pos.y
        point3.z = camera_pos.z
        

        point4 = Point()
        point4.x = camera_pos.x + länge * np.sin(camera_angle - self.FOV / 2)
        point4.y = camera_pos.y + länge * np.cos(camera_angle - self.FOV / 2)
        point4.z = camera_pos.z
        points.append(point4)
        points.append(point2)
        points.append(point3)
        points.append(point4)
    
        return points
    
    def points_lidar(self, num, start, angle):
        """
        Calculates the 3D points for the lidar rays.

        Args:
            num (int): The index of the lidar sensor.
            start (tuple): The starting position of the lidar rays.
            angle (float): The angular resolution of the lidar rays.

        Returns:
            list: A list of 3D points defining the lidar rays.
        """
        points = []
        for i in range(int(180 / angle)-1):
            i += 1
            if num == 1:
                i = i + int(180 / angle)
            angle_rad = np.radians(angle) * i
            start_point = Point()
            start_point.x = start[0]
            start_point.y = start[1]
            start_point.z = start[2]
            points.append(start_point)

            länge = np.abs((self.room_size[0]/2) / np.cos(angle_rad))
            if länge > np.sqrt((self.room_size[0]/2)*(self.room_size[0]/2) + self.room_size[1]*self.room_size[1]):
                länge = np.abs((self.room_size[1]) / np.sin(angle_rad))

            end = Point()
            end.x = start_point.x + länge * np.cos(angle_rad)
            end.y = start_point.y + länge * np.sin(angle_rad)
            end.z = start[2]  
            points.append(end)
        return points

    def normalize_angle(self, angle):
        """
        Normalizes the angle to be within -pi to pi.

        Args:
            angle (float): The angle in radians.

        Returns:
            float: The normalized angle in radians.
        """
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