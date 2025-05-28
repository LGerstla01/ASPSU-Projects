import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
from robot_msgs.msg import LidarArray, Lidar

class lidarDetectionNode(Node):
    def __init__(self):
        super().__init__('lidarDetectionNode')
        self.declare_parameter('room_size', [40.0, 30.0, 3.0])  # [width, height, height]
        self.declare_parameter('num_lidar', 1) 
        self.declare_parameter('angle_lidar', 20)
        self.declare_parameter('noise_dist', 0.2)
        self.declare_parameter('noise_angle', 0.1)

        self.room_size = self.get_parameter('room_size').value
        self.num_lidar = self.get_parameter('num_lidar').value
        self.angle_lidar = self.get_parameter('angle_lidar').value
        self.noise_dist = self.get_parameter('noise_dist').value
        self.noise_angle = self.get_parameter('noise_angle').value

        self.robot_sub = self.create_subscription(PoseArray, '/robots/pose', self.robot_callback, 10)
        self.lidar_positions = self.initialize_lidar_positions()
        self.lidar_publishers = self.initialize_lidar_publishers()

    def initialize_lidar_positions(self):
        # (x, y, z) wie in der Visualisierung
        positions = [
            (0.0, -self.room_size[1] / 2, 1.0),
            (0.0, self.room_size[1] / 2, 1.0),
        ]
        return positions[:self.num_lidar]

    def initialize_lidar_publishers(self):
        publishers = []
        for i in range(self.num_lidar):
            topic_name = f'/detections/lidar_{i+1}'
            publishers.append(self.create_publisher(LidarArray, topic_name, 10))
        return publishers

    def robot_callback(self, msg):
        # Für jeden Lidar ein PoseArray
        detections_per_lidar = [LidarArray() for _ in range(self.num_lidar)]
        for i, detections in enumerate(detections_per_lidar):
            detections.header = Header()
            detections.header.stamp = self.get_clock().now().to_msg()
            detections.header.frame_id = "map"

        for i, lidar_pos in enumerate(self.lidar_positions):
            for robot_pose in msg.poses:
                pos_robot = self.is_hit_by_lidar(lidar_pos, self.angle_lidar, robot_pose, i)
                if pos_robot:
                    noisy_pose = self.add_noise(pos_robot[1], pos_robot[0])
                    lidar_msg = Lidar()
                    lidar_msg.angle = noisy_pose[0]
                    lidar_msg.distance = noisy_pose[1]
                    detections_per_lidar[i].detections.append(lidar_msg)
        for i, detections in enumerate(detections_per_lidar):
            if len(detections.detections) > 0:
                self.lidar_publishers[i].publish(detections)

    def is_hit_by_lidar(self, lidar_pos, angle_lidar, robot_pose, lidar_num, max_dist=0.5):
        # Simuliere Strahlen wie in points_lidar und prüfe Abstand zum Roboter
        num_rays = int(180 / angle_lidar)-1
        for j in range(num_rays):
            idx = j + 1
            if lidar_num == 1:
                idx = idx + num_rays
            angle_rad = np.radians(angle_lidar) * idx
            lx, ly = lidar_pos[0], lidar_pos[1]
            # Roboterposition
            rx = robot_pose.position.x
            ry = robot_pose.position.y
            # Abstand Punkt zu Gerade (Lidar-Strahl)
            dist = self.abstand_punkt_zu_gerade((rx, ry), (lx, ly), angle_rad)
            if dist < max_dist:
                dist_lidar = np.sqrt((rx - lx) ** 2 + (ry - ly) ** 2)
                return [angle_rad, dist_lidar]
        return None
    
    def abstand_punkt_zu_gerade(self, punkt, startpunkt, winkel):
        x, y = punkt
        x0, y0 = startpunkt
        theta = winkel
        
        dx = x - x0
        dy = y - y0

        # Abstand über Kreuzprodukt-Methode (2D)
        abstand = abs(dx * np.sin(theta) - dy * np.cos(theta))
        return abstand

    def add_noise(self, dist, angle):
        # Add Gaussian noise to the distance and angle
        noisy_dist = dist + np.random.normal(0, self.noise_dist)
        noisy_angle = angle + np.random.normal(0, self.noise_angle)
        return [noisy_angle, noisy_dist]

def main(args=None):
    rclpy.init(args=args)
    node = lidarDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
