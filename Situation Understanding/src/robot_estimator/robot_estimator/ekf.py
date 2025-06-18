# ekf.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from robot_msgs.msg import LidarArray  # Custom message
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from scipy.stats import chi2
from scipy.optimize import linear_sum_assignment
import threading

TRACK_COLORS = [
    ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5),
    ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5),
    ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5),
    ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5),
    ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.5),
]

GATE_THRESHOLD = chi2.ppf(0.99, df=2)  # ≈ 9.21 für 99% Konfidenzintervall


class SingleEKF:
    def __init__(self, initial_pose: Pose, node: Node, track_id: int):
        self.node = node
        self.track_id = track_id

        quat = [initial_pose.orientation.x, initial_pose.orientation.y,
                initial_pose.orientation.z, initial_pose.orientation.w]
        initial_yaw = R.from_quat(quat).as_euler('zyx')[0]

        self.x = np.array([initial_pose.position.x, initial_pose.position.y, initial_yaw, 0.0, 0.0])
        self.P = np.eye(5) * 1.0

        self.R = np.diag([0.1, 0.1])
        self.Q = np.diag([0.05, 0.05, 0.01, 0.1, 0.1])

        self.last_time = self.node.get_clock().now()

        self.publisher = self.node.create_publisher(Odometry, f'/ekf/robot_{self.track_id}/odom', 10)
        self.marker_pub = self.node.create_publisher(Marker, f'/ekf/robot_{self.track_id}/cov_ellipse', 10)

        self.missed_updates = 0
        self.max_missed_updates = 9999  # sehr hoch, damit nie automatisch gelöscht

        self.node.get_logger().info(f'Initialized EKF {self.track_id} at ({self.x[0]:.2f}, {self.x[1]:.2f})')

    def predict(self, dt: float):
        x, y, theta, v, omega = self.x

        x_pred = x + v * np.cos(theta) * dt
        y_pred = y + v * np.sin(theta) * dt
        theta_pred = self.normalize_angle(theta + omega * dt)

        self.x = np.array([x_pred, y_pred, theta_pred, v, omega])

        F = np.eye(5)
        F[0, 2] = -v * np.sin(theta) * dt
        F[0, 3] = np.cos(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt
        F[1, 3] = np.sin(theta) * dt
        F[2, 4] = dt

        self.P = F @ self.P @ F.T + self.Q
        self.missed_updates += 1

    def update(self, measurement: Pose):
        H = np.array([[1, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0]])

        z_measured = np.array([measurement.position.x, measurement.position.y])
        z_predicted = H @ self.x

        y = z_measured - z_predicted
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P
        self.missed_updates = 0

    def get_mahalanobis_distance(self, measurement: Pose):
        H = np.array([[1, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0]])
        z_measured = np.array([measurement.position.x, measurement.position.y])
        z_predicted = H @ self.x
        y = z_measured - z_predicted
        S = H @ self.P @ H.T + self.R

        try:
            return y.T @ np.linalg.inv(S) @ y
        except np.linalg.LinAlgError:
            self.node.get_logger().warn(f"Singular matrix in Mahalanobis distance for EKF {self.track_id}")
            return float('inf')

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.node.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = f'robot_{self.track_id}/base_link'

        odom.pose.pose.position.x = self.x[0]
        odom.pose.pose.position.y = self.x[1]
        odom.pose.pose.position.z = 0.0

        quaternion = R.from_euler('z', self.x[2]).as_quat()
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.twist.twist.linear.x = self.x[3]
        odom.twist.twist.angular.z = self.x[4]

        self.publisher.publish(odom)
        self.publish_covariance_ellipse()

    def publish_covariance_ellipse(self):
        P_xy = self.P[0:2, 0:2]
        eigenvals, eigenvecs = np.linalg.eig(P_xy)

        if np.any(eigenvals < 0):
            self.node.get_logger().warn(f'Negative eigenvalues in EKF {self.track_id}')
            return

        chi2_val = 5.991
        major, minor = 2 * np.sqrt(chi2_val * eigenvals)
        angle = np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0])

        ellipse = Marker()
        ellipse.header.frame_id = "map"
        ellipse.header.stamp = self.node.get_clock().now().to_msg()
        ellipse.ns = f"cov_ellipse_{self.track_id}"
        ellipse.id = self.track_id
        ellipse.type = Marker.CYLINDER
        ellipse.action = Marker.ADD

        ellipse.pose.position.x = self.x[0]
        ellipse.pose.position.y = self.x[1]
        ellipse.pose.position.z = 0.01
        ellipse.pose.orientation.z = np.sin(angle / 2)
        ellipse.pose.orientation.w = np.cos(angle / 2)

        ellipse.scale.x = major
        ellipse.scale.y = minor
        ellipse.scale.z = 0.01

        ellipse.color = TRACK_COLORS[self.track_id % len(TRACK_COLORS)]
        self.marker_pub.publish(ellipse)

        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = self.node.get_clock().now().to_msg()
        text.ns = f"text_{self.track_id}"
        text.id = 1000 + self.track_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD

        text.pose.position.x = self.x[0]
        text.pose.position.y = self.x[1]
        text.pose.position.z = 0.5
        text.scale.z = 0.3
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = f"ID {self.track_id}"

        self.marker_pub.publish(text)

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi


class EKFTracker(Node):
    def __init__(self):
        super().__init__('ekf_tracker')

        self.declare_parameter('num_robots_to_track', 2)
        self.num_robots_to_track = self.get_parameter('num_robots_to_track').value

        self.tracked_ekfs = {i: None for i in range(self.num_robots_to_track)}
        self.detection_buffer = []
        self.buffer_lock = threading.Lock()

        self.create_subscription(PoseArray, '/detections/camera_1', self.camera_callback, 10)
        self.create_subscription(PoseArray, '/detections/camera_2', self.camera_callback, 10)
        self.create_subscription(LidarArray, '/detections/lidar_1', self.lidar_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_timer_time = self.get_clock().now()

        self.get_logger().info("EKF Tracker initialized.")

    def camera_callback(self, msg: PoseArray):
        with self.buffer_lock:
            self.detection_buffer.extend(msg.poses)

    def lidar_callback(self, msg: LidarArray):
        with self.buffer_lock:
            for d in msg.detections:
                pose = Pose()
                pose.position.x = d.distance * math.cos(d.angle)
                pose.position.y = d.distance * math.sin(d.angle)
                pose.orientation.w = 1.0
                self.detection_buffer.append(pose)

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds / 1e9
        self.last_timer_time = now

        with self.buffer_lock:
            detections = list(self.detection_buffer)
            self.detection_buffer.clear()

        initialized_ids = [i for i, ekf in self.tracked_ekfs.items() if ekf]
        cost_matrix = np.full((len(detections), len(initialized_ids)), np.inf)

        for i, det in enumerate(detections):
            for j, ekf_id in enumerate(initialized_ids):
                cost = self.tracked_ekfs[ekf_id].get_mahalanobis_distance(det)
                cost_matrix[i, j] = cost

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        used_detections = set()
        assignments = {}

        for i, j in zip(row_ind, col_ind):
            if cost_matrix[i, j] < GATE_THRESHOLD:
                ekf_id = initialized_ids[j]
                assignments[ekf_id] = detections[i]
                used_detections.add(i)

        for ekf_id, ekf in self.tracked_ekfs.items():
            if ekf:
                if ekf_id in assignments:
                    ekf.update(assignments[ekf_id])
                ekf.predict(dt)
                ekf.publish_odometry()

        # Initialisierung nur falls EKF noch nie existierte
        for i, ekf in self.tracked_ekfs.items():
            if ekf is None:
                new_detections = [detections[j] for j in range(len(detections)) if j not in used_detections]
                if new_detections:
                    self.tracked_ekfs[i] = SingleEKF(new_detections.pop(0), self, i)
                    used_detections.add(0)  # Vermeide doppelte Nutzung

def main(args=None):
    rclpy.init(args=args)
    tracker = EKFTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
