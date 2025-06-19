# ekf_logger_node.py – ROS2-Logger für EKF-Debugging
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from robot_msgs.msg import LidarArray
import csv
import os
import math
from datetime import datetime

class EKFLogger(Node):
    def __init__(self):
        super().__init__('ekf_logger')

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.base_path = os.path.expanduser(f'~/ekf_logs_{timestamp}')
        os.makedirs(self.base_path, exist_ok=True)

        self.init_csv('ekf_robot_0.csv', ['time', 'x', 'y', 'theta', 'v', 'omega'])
        self.init_csv('ekf_robot_1.csv', ['time', 'x', 'y', 'theta', 'v', 'omega'])
        self.init_csv('camera_detections.csv', ['time', 'camera_id', 'x', 'y'])
        self.init_csv('lidar_detections.csv', ['time', 'x', 'y'])

        self.create_subscription(Odometry, '/ekf/robot_0/odom', lambda msg: self.log_odom(msg, 0), 10)
        self.create_subscription(Odometry, '/ekf/robot_1/odom', lambda msg: self.log_odom(msg, 1), 10)

        self.create_subscription(PoseArray, '/detections/camera_1', lambda msg: self.log_camera(msg, 1), 10)
        self.create_subscription(PoseArray, '/detections/camera_2', lambda msg: self.log_camera(msg, 2), 10)
        self.create_subscription(LidarArray, '/detections/lidar_1', self.log_lidar, 10)

        self.get_logger().info(f'Logger started. Writing to: {self.base_path}')

    def init_csv(self, name, header):
        with open(os.path.join(self.base_path, name), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)

    def log_odom(self, msg, robot_id):
        file = f'ekf_robot_{robot_id}.csv'
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qw = msg.pose.pose.orientation.w
        qz = msg.pose.pose.orientation.z
        theta = 2 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        row = [t, x, y, theta, v, omega]
        self.write_row(file, row)

    def log_camera(self, msg, cam_id):
        t = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().nanoseconds % 1e9 * 1e-9
        for pose in msg.poses:
            row = [t, cam_id, pose.position.x, pose.position.y]
            self.write_row('camera_detections.csv', row)

    def log_lidar(self, msg):
        t = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().nanoseconds % 1e9 * 1e-9
        for d in msg.detections:
            x = d.distance * math.cos(d.angle)
            y = d.distance * math.sin(d.angle)
            self.write_row('lidar_detections.csv', [t, x, y])

    def write_row(self, filename, row):
        with open(os.path.join(self.base_path, filename), 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)


def main(args=None):
    rclpy.init(args=args)
    node = EKFLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()