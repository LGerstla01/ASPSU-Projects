import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from robot_msgs.msg import LidarArray
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import threading
from scipy.optimize import linear_sum_assignment

class SingleEKF:
    def __init__(self, initial_pose: Pose, node: Node, track_id: int):
        self.node = node
        self.track_id = track_id

        quat = [initial_pose.orientation.x, initial_pose.orientation.y,
                initial_pose.orientation.z, initial_pose.orientation.w]
        initial_yaw = R.from_quat(quat).as_euler('zyx')[0]

        self.x = np.array([initial_pose.position.x, initial_pose.position.y, initial_yaw, 0.0, 0.0])
        self.P = np.eye(5) * 1.0

        self.Q = np.diag([0.05, 0.05, 0.02, 0.2, 0.2])

        self.last_time = self.node.get_clock().now()
        self.last_update_time = self.last_time

        self.last_meas_pos = None
        self.last_meas_theta = None
        self.last_meas_time = None

        self.publisher = self.node.create_publisher(Odometry, f'/ekf/robot_{self.track_id}/odom', 10)
        self.marker_pub = self.node.create_publisher(Marker, f'/ekf/robot_{self.track_id}/cov_ellipse', 10)

        self.x_min = -20.0
        self.x_max = 20.0
        self.y_min = -15.0
        self.y_max = 15.0

        self.node.get_logger().info(f'Initialized EKF for Robot {self.track_id} at ({initial_pose.position.x:.2f}, {initial_pose.position.y:.2f})')

    def predict(self, dt: float):
        x, y, theta, v, omega = self.x
        x_pred = x + v * np.cos(theta) * dt
        y_pred = y + v * np.sin(theta) * dt
        theta_pred = self.normalize_angle(theta + omega * dt)

        if x_pred <= self.x_min or x_pred >= self.x_max:
            theta_pred = math.pi - theta_pred
            x_pred = np.clip(x_pred, self.x_min, self.x_max)

        if y_pred <= self.y_min or y_pred >= self.y_max:
            theta_pred = -theta_pred
            y_pred = np.clip(y_pred, self.y_min, self.y_max)

        self.x = np.array([x_pred, y_pred, theta_pred, v, omega])

        F = np.eye(5)
        F[0, 2] = -v * np.sin(theta) * dt
        F[0, 3] = np.cos(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt
        F[1, 3] = np.sin(theta) * dt
        F[2, 4] = dt

        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement: Pose, sensor_type):
        # Messung vektorisieren + passende R wählen
        if sensor_type == "camera":
            quat = [measurement.orientation.x, measurement.orientation.y,
                    measurement.orientation.z, measurement.orientation.w]
            yaw = R.from_quat(quat).as_euler('zyx')[0]
            z_measured = np.array([measurement.position.x, measurement.position.y, yaw])
            R_mat = np.diag([0.005, 0.005, 0.01])
        elif sensor_type == "lidar":
            z_measured = np.array([measurement.position.x, measurement.position.y])
            R_mat = np.diag([0.05, 0.05])
        else:
            z_measured = np.array([measurement.position.x, measurement.position.y])
            R_mat = np.diag([0.1, 0.1])

        # Nichtlineare Vorhersage
        z_predicted = self.h_of_x(self.x, sensor_type)
        y = z_measured - z_predicted

        # Winkel normalisieren (nur bei Kamera notwendig)
        if sensor_type == "camera" and len(y) == 3:
            y[2] = self.normalize_angle(y[2])

        # Jacobi-Matrix berechnen
        H = self.compute_H(self.x, sensor_type)

        # EKF Update
        S = H @ self.P @ H.T + R_mat
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ H) @ self.P

        # Geschwindigkeit schätzen wie gehabt
        now = self.node.get_clock().now()
        if self.last_meas_pos is not None and self.last_meas_time is not None:
            dt = (now - self.last_meas_time).nanoseconds / 1e9
            if dt > 0.05:
                dx = measurement.position.x - self.last_meas_pos[0]
                dy = measurement.position.y - self.last_meas_pos[1]
                distance = np.sqrt(dx ** 2 + dy ** 2)
                v_est = distance / dt

                if sensor_type == "camera":
                    # Winkeländerung direkt aus Messung:
                    quat = [measurement.orientation.x, measurement.orientation.y,
                            measurement.orientation.z, measurement.orientation.w]
                    yaw = R.from_quat(quat).as_euler('zyx')[0]
                    if self.last_meas_theta is not None:
                        dtheta = self.normalize_angle(yaw - self.last_meas_theta)
                        omega_est = dtheta / dt
                    else:
                        omega_est = self.x[4]
                elif sensor_type == "lidar":
                    # Winkeländerung aus Bewegungsrichtung:
                    last_dx = self.last_meas_pos[0] - self.x[0]
                    last_dy = self.last_meas_pos[1] - self.x[1]
                    current_direction = math.atan2(dy, dx)
                    last_direction = math.atan2(last_dy, last_dx)
                    dtheta = self.normalize_angle(current_direction - last_direction)
                    omega_est = dtheta / dt
                else:
                    omega_est = self.x[4]

                self.x[3] = 0.8 * self.x[3] + 0.2 * v_est
                self.x[4] = 0.8 * self.x[4] + 0.2 * omega_est

        # Letzte Messwerte updaten ...
        self.last_meas_pos = [measurement.position.x, measurement.position.y]
        self.last_meas_time = now
        if sensor_type == "camera":
            quat = [measurement.orientation.x, measurement.orientation.y,
                    measurement.orientation.z, measurement.orientation.w]
            self.last_meas_theta = R.from_quat(quat).as_euler('zyx')[0]
        else:
            self.last_meas_theta = None

    def get_mahalanobis_distance(self, measurement: Pose, sensor_type: str):
        # Messung als Vektor
        if sensor_type == "camera":
            quat = [measurement.orientation.x, measurement.orientation.y,
                    measurement.orientation.z, measurement.orientation.w]
            yaw = R.from_quat(quat).as_euler('zyx')[0]
            z_measured = np.array([measurement.position.x, measurement.position.y, yaw])
            R_mat = np.diag([0.005, 0.005, 0.01])
        elif sensor_type == "lidar":
            z_measured = np.array([measurement.position.x, measurement.position.y])
            R_mat = np.diag([0.05, 0.05])
        else:
            z_measured = np.array([measurement.position.x, measurement.position.y])
            R_mat = np.diag([0.1, 0.1])

        z_predicted = self.h_of_x(self.x, sensor_type)
        y = z_measured - z_predicted

        # Winkel normalisieren (nur wenn yaw gemessen wird)
        if sensor_type == "camera" and len(y) == 3:
            y[2] = self.normalize_angle(y[2])

        H = self.compute_H(self.x, sensor_type)

        S = H @ self.P @ H.T + R_mat
        try:
            d2 = y.T @ np.linalg.inv(S) @ y
            return d2
        except np.linalg.LinAlgError:
            self.node.get_logger().warn(f"Singular matrix in Mahalanobis distance for EKF {self.track_id}.")
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
        self.publish_velocity_arrow()

    def publish_covariance_ellipse(self):
        P_xy = self.P[0:2, 0:2]
        eigenvals, eigenvecs = np.linalg.eig(P_xy)

        if np.any(eigenvals < 0):
            self.node.get_logger().warn(f'Negative Eigenwerte in Covariance EKF {self.track_id}')
            return

        chi2_val = 5.991
        major, minor = 2 * np.sqrt(chi2_val * eigenvals)
        angle = np.arctan2(eigenvecs[1, 0], eigenvecs[0, 0])

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = f"cov_ellipse_{self.track_id}"
        marker.id = self.track_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = self.x[0]
        marker.pose.position.y = self.x[1]
        marker.pose.position.z = 0.01

        marker.pose.orientation.z = np.sin(angle / 2)
        marker.pose.orientation.w = np.cos(angle / 2)

        marker.scale.x = major
        marker.scale.y = minor
        marker.scale.z = 0.01

        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
        self.marker_pub.publish(marker)

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def publish_velocity_arrow(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = f"velocity_arrow_{self.track_id}"
        marker.id = self.track_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        x, y, theta, v, _ = self.x
        dx = v * np.cos(theta)
        dy = v * np.sin(theta)

        marker.points = [
            self._make_point(x, y),
            self._make_point(x + dx, y + dy)
        ]

        marker.scale.x = 0.05  # Shaft diameter
        marker.scale.y = 0.1   # Head diameter
        marker.scale.z = 0.1   # Head length

        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green

        self.marker_pub.publish(marker)

    def _make_point(self, x, y):
        from geometry_msgs.msg import Point
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.05
        return p
    
    def h_of_x(self, x, sensor_type):
        if sensor_type == "camera":
            return np.array([x[0], x[1], x[2]])  # misst Position + Richtung
        elif sensor_type == "lidar":
            return np.array([x[0], x[1]])        # misst nur Position
        else:
            return np.array([x[0], x[1]])        # fallback

    def compute_H(self, x, sensor_type, eps=1e-5):
        h0 = self.h_of_x(x, sensor_type)
        H = np.zeros((len(h0), len(x)))
        for i in range(len(x)):
            x_perturbed = np.copy(x)
            x_perturbed[i] += eps
            h_perturbed = self.h_of_x(x_perturbed, sensor_type)
            H[:, i] = (h_perturbed - h0) / eps
        return H
    
class EKFTracker(Node):
    def __init__(self):
        super().__init__('ekf_tracker')
        self.declare_parameter('num_robots_to_track', 2)
        self.num_robots_to_track = self.get_parameter('num_robots_to_track').value

        self.tracked_ekfs = {i: None for i in range(self.num_robots_to_track)}
        self.GATE_THRESHOLD = 30

        self.detection_buffer = []
        self.buffer_lock = threading.Lock()

        self.create_subscription(PoseArray, '/detections/camera_1', lambda msg: self.camera_callback(msg, "camera"), 10)
        self.create_subscription(PoseArray, '/detections/camera_2', lambda msg: self.camera_callback(msg, "camera"), 10)
        self.create_subscription(LidarArray, '/detections/lidar_1', self.lidar_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_timer_time = self.get_clock().now()

        self.get_logger().info("EKF Tracker initialized.")

    def is_track_stale(self, ekf: SingleEKF, now):
        time_since_update = (now - ekf.last_update_time).nanoseconds / 1e9
        return time_since_update > 6.0

    def camera_callback(self, msg: PoseArray, sensor_type: str):
        with self.buffer_lock:
            for pose in msg.poses:
                self.detection_buffer.append((pose, sensor_type))

    def lidar_callback(self, msg: LidarArray):
        lidar_origin_x = 0.0
        lidar_origin_y = -15.0
        with self.buffer_lock:
            for detection in msg.detections:
                x = lidar_origin_x + detection.distance * math.cos(detection.angle)
                y = lidar_origin_y + detection.distance * math.sin(detection.angle)

                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 0.0
                pose.orientation.w = 1.0
                self.detection_buffer.append((pose, "lidar"))

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds / 1e9
        self.last_timer_time = now

        for robot_id, ekf in self.tracked_ekfs.items():
            if ekf:
                ekf.predict(dt)

        for robot_id, ekf in list(self.tracked_ekfs.items()):
            if ekf and self.is_track_stale(ekf, now):
                self.get_logger().info(f"Removing stale EKF {robot_id}")
                self.tracked_ekfs[robot_id] = None

        with self.buffer_lock:
            detections = list(self.detection_buffer)
            self.detection_buffer.clear()

        assignments = {i: None for i in self.tracked_ekfs}
        initialized_ids = [i for i, ekf in self.tracked_ekfs.items() if ekf]
        cost_matrix = np.full((len(detections), len(initialized_ids)), float('inf'))

        for i, (det, sensor) in enumerate(detections):
            for j, robot_id in enumerate(initialized_ids):
                cost = self.tracked_ekfs[robot_id].get_mahalanobis_distance(det, sensor)
                cost_matrix[i, j] = cost

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        used_indices = set()

        for det_idx, col_idx in zip(row_ind, col_ind):
            robot_id = initialized_ids[col_idx]
            if cost_matrix[det_idx, col_idx] < self.GATE_THRESHOLD:
                assignments[robot_id] = detections[det_idx]
                used_indices.add(det_idx)

        unassigned_detections = [detections[i] for i in range(len(detections)) if i not in used_indices]
        uninitialized_ids = [i for i, ekf in self.tracked_ekfs.items() if ekf is None]

        for robot_id in uninitialized_ids:
            if unassigned_detections:
                pose, sensor = unassigned_detections.pop(0)
                self.tracked_ekfs[robot_id] = SingleEKF(pose, self, robot_id)
            else:
                break

        for robot_id, ekf in self.tracked_ekfs.items():
            if ekf and assignments[robot_id]:
                ekf.update(assignments[robot_id][0], assignments[robot_id][1])
            if ekf:
                ekf.publish_odometry()

def main(args=None):
    rclpy.init(args=args)
    tracker = EKFTracker()
    rclpy.spin(tracker)
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()