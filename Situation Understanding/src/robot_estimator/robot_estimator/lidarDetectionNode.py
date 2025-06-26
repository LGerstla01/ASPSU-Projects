import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
from robot_msgs.msg import LidarArray, Lidar

class lidarDetectionNode(Node):
    """
    A ROS2 node that simulates lidar sensors detecting robots in a room.
    The node publishes noisy lidar detections based on robot positions and lidar configurations.

    Attributes:
        room_size (list): Dimensions of the room [width, height, height].
        num_lidar (int): Number of lidar sensors in the room.
        angle_lidar (float): Angular resolution of lidar rays in degrees.
        noise_dist (float): Gaussian noise added to the distance measurements.
        noise_angle (float): Gaussian noise added to the angle measurements.
    """
    def __init__(self):
        """
        Initializes the lidar detection node, sets parameters, and creates subscriptions and publishers.
        """
        super().__init__('lidarDetectionNode')
        # Declare parameters for room size, lidar configuration, and noise
        self.declare_parameter('room_size', [40.0, 30.0, 3.0])  # [width, height, height]
        self.declare_parameter('num_lidar', 1) 
        self.declare_parameter('angle_lidar', 10)
        self.declare_parameter('noise_dist', 0.5)
        self.declare_parameter('noise_angle', 0.1)

        # Retrieve parameter values
        self.room_size = self.get_parameter('room_size').value
        self.num_lidar = self.get_parameter('num_lidar').value
        self.angle_lidar = self.get_parameter('angle_lidar').value
        self.noise_dist = self.get_parameter('noise_dist').value
        self.noise_angle = self.get_parameter('noise_angle').value

        # Subscribe to robot positions
        self.robot_sub = self.create_subscription(PoseArray, '/robots/pose', self.robot_callback, 10)
        # Initialize lidar positions and publishers
        self.lidar_positions = self.initialize_lidar_positions()
        self.lidar_publishers = self.initialize_lidar_publishers()

    def initialize_lidar_positions(self):
        """
        Defines the positions of lidar sensors in the room.

        Returns:
            list: A list of lidar positions as (x, y, z) tuples.
        """
        # Define lidar positions based on room dimensions
        positions = [
            (0.0, -self.room_size[1] / 2, 1.0),  # Bottom center
            (0.0, self.room_size[1] / 2, 1.0),   # Top center
        ]
        return positions[:self.num_lidar]  # Return positions based on the number of lidar sensors

    def initialize_lidar_publishers(self):
        """
        Creates publishers for lidar detections.

        Returns:
            list: A list of publishers for lidar detection topics.
        """
        publishers = []
        for i in range(self.num_lidar):
            topic_name = f'/detections/lidar_{i+1}'  # Topic name for each lidar
            publishers.append(self.create_publisher(LidarArray, topic_name, 10))
        return publishers

    def robot_callback(self, msg):
        """
        Callback function for robot position updates. Simulates lidar detections based on robot positions.

        Args:
            msg (PoseArray): The message containing robot positions.
        """
        # Create a LidarArray for each lidar sensor
        detections_per_lidar = [LidarArray() for _ in range(self.num_lidar)]
        for i, detections in enumerate(detections_per_lidar):
            detections.header = Header()  # Initialize header for each LidarArray
            detections.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
            detections.header.frame_id = "map"  # Set frame ID to "map"

        # Simulate lidar detections for each robot
        for i, lidar_pos in enumerate(self.lidar_positions):
            for robot_pose in msg.poses:
                pos_robot = self.is_hit_by_lidar(lidar_pos, self.angle_lidar, robot_pose, i)
                if pos_robot:
                    noisy_pose = self.add_noise(pos_robot[1], pos_robot[0])  # Add noise to detections
                    lidar_msg = Lidar()
                    lidar_msg.angle = noisy_pose[0]  # Set noisy angle
                    lidar_msg.distance = noisy_pose[1]  # Set noisy distance
                    detections_per_lidar[i].detections.append(lidar_msg)  # Add detection to LidarArray

        # Publish detections for each lidar sensor
        for i, detections in enumerate(detections_per_lidar):
            if len(detections.detections) > 0:  # Only publish if there are detections
                self.lidar_publishers[i].publish(detections)

    def is_hit_by_lidar(self, lidar_pos, angle_lidar, robot_pose, lidar_num, max_dist=0.5):
        """
        Simulates lidar rays and checks if a robot is hit by any ray.

        Args:
            lidar_pos (tuple): Position of the lidar sensor (x, y, z).
            angle_lidar (float): Angular resolution of lidar rays in degrees.
            robot_pose (Pose): Position of the robot.
            lidar_num (int): Index of the lidar sensor.
            max_dist (float): Maximum distance for a hit.

        Returns:
            list or None: [angle, distance] if hit, otherwise None.
        """
        num_rays = int(180 / angle_lidar) - 1  # Number of rays per lidar
        for j in range(num_rays):
            idx = j + 1
            if lidar_num == 1:  # Adjust ray index for second lidar
                idx = idx + num_rays
            angle_rad = np.radians(angle_lidar) * idx  # Compute ray angle in radians
            lx, ly = lidar_pos[0], lidar_pos[1]  # Lidar position
            rx = robot_pose.position.x  # Robot x position
            ry = robot_pose.position.y  # Robot y position
            # Compute distance from robot to ray
            dist = self.abstand_punkt_zu_gerade((rx, ry), (lx, ly), angle_rad)
            if dist < max_dist:  # Check if within maximum distance
                dist_lidar = np.sqrt((rx - lx) ** 2 + (ry - ly) ** 2)  # Compute distance to lidar
                return [angle_rad, dist_lidar]
        return None

    def abstand_punkt_zu_gerade(self, punkt, startpunkt, winkel):
        """
        Computes the perpendicular distance from a point to a line.

        Args:
            punkt (tuple): Coordinates of the point (x, y).
            startpunkt (tuple): Starting point of the line (x0, y0).
            winkel (float): Angle of the line in radians.

        Returns:
            float: Perpendicular distance.
        """
        x, y = punkt
        x0, y0 = startpunkt
        theta = winkel

        dx = x - x0
        dy = y - y0

        # Compute distance using the cross-product method (2D)
        abstand = abs(dx * np.sin(theta) - dy * np.cos(theta))
        return abstand

    def add_noise(self, dist, angle):
        """
        Adds Gaussian noise to distance and angle measurements.

        Args:
            dist (float): Original distance measurement.
            angle (float): Original angle measurement.

        Returns:
            list: [noisy_angle, noisy_dist] with added noise.
        """
        noisy_dist = dist + np.random.normal(0, self.noise_dist)  # Add noise to distance
        noisy_angle = angle + np.random.normal(0, self.noise_angle)  # Add noise to angle
        return [noisy_angle, noisy_dist]

def main(args=None):
    """
    Entry point for the lidar detection node. Initializes the node and starts spinning.
    """
    rclpy.init(args=args)
    node = lidarDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
