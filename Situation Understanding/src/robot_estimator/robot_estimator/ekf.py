import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
import numpy as np

class EKFTracker(Node):
    def __init__(self):
        super().__init__('ekf_tracker')

        self.subscription = self.create_subscription(
            PoseArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.publisher = self.create_publisher(Odometry, '/ekf/odom', 10)

        # Initialer Zustand [x, y, theta, v, omega]
        self.x = np.zeros(5)
        self.P = np.eye(5) * 0.1

        # Zeitstempel
        self.last_time = self.get_clock().now()

        # R: Messrauschen x/y
        self.R = np.diag([0.1, 0.1])  # anpassen an Kamera

        # Q: Prozessrauschen
        self.Q = np.diag([0.05, 0.05, 0.01, 0.1, 0.1])  # frei wählbar

    def detection_callback(self, msg: PoseArray):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.predict(dt)

        # Auswahl der besten Detektion per Mahalanobis-Distanz
        best_pose = self.get_best_measurement(msg.poses)

        if best_pose is not None:
            z = np.array([best_pose.position.x, best_pose.position.y])
            self.update(z)

        self.publish_odometry()

    def predict(self, dt):
        x, y, theta, v, omega = self.x
        if dt == 0:
            return

        # Motion Model für Differentialantrieb
        theta_new = theta + omega * dt
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt

        self.x[0] = x_new
        self.x[1] = y_new
        self.x[2] = theta_new

        # Jacobian F (5x5)
        F = np.eye(5)
        F[0, 3] = np.cos(theta) * dt
        F[0, 4] = -v * np.sin(theta) * dt
        F[1, 3] = np.sin(theta) * dt
        F[1, 4] = v * np.cos(theta) * dt
        F[2, 4] = dt

        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        # Messmatrix H
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])

        z_hat = H @ self.x
        y = z - z_hat
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(5) - K @ H) @ self.P

    def get_best_measurement(self, poses):
        """Finde Detektion mit kleinster Mahalanobis-Distanz"""
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])
        z_hat = H @ self.x
        S = H @ self.P @ H.T + self.R

        min_d = float('inf')
        best_pose = None
        for pose in poses:
            z = np.array([pose.position.x, pose.position.y])
            y = z - z_hat
            try:
                d2 = y.T @ np.linalg.inv(S) @ y
            except np.linalg.LinAlgError:
                continue

            if d2 < 5.99 and d2 < min_d:  # 95%-Gate
                min_d = d2
                best_pose = pose
        return best_pose

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x[0]
        odom.pose.pose.position.y = self.x[1]
        odom.pose.pose.orientation.z = np.sin(self.x[2] / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.x[2] / 2.0)

        odom.twist.twist.linear.x = self.x[3]
        odom.twist.twist.angular.z = self.x[4]

        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = EKFTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
