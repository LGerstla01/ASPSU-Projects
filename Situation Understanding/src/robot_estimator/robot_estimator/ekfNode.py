import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
import math
from builtin_interfaces.msg import Time
from ekf_tracker_msgs.msg import RobotState, RobotStateArray



class EKFTracker(Node):
    def __init__(self):
        super().__init__('ekf_tracker')

        # Parameter einlesen
        self.declare_parameter('num_robots', 3)
        self.declare_parameter('num_cameras', 2)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.num_cameras = self.get_parameter('num_cameras').get_parameter_value().integer_value

        # EKF-Zustände initialisieren
        self.filters = [self.initialize_ekf() for _ in range(self.num_robots)]
        self.last_update_time = [self.get_clock().now() for _ in range(self.num_robots)]

        # Publisher für Pose-Schätzungen
        #self.estimates_pub = self.create_publisher(PoseArray, '/robot_estimates', 10)
        
        self.state_pub = self.create_publisher(RobotStateArray, '/robot_states', 10)

        # Abonnements für Kamera-Topics
        for i in range(self.num_cameras):
            topic = f'/camera_{i+1}/detections'
            self.create_subscription(PoseArray, topic, self.make_callback(i), 10)

        # Timer für Prädiktion, falls keine neue Messung vorliegt
        self.create_timer(0.05, self.predict_missing)

    def initialize_ekf(self):
        dt = 0.1
        x = np.zeros(5)  # [x, y, theta, v, omega]
        P = np.eye(5) * 0.1
        Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1])
        R = np.diag([0.04, 0.04])  # entspricht sigma=0.2
        return {'x': x, 'P': P, 'Q': Q, 'R': R, 'dt': dt}

    def make_callback(self, cam_id):
        def callback(msg):
            now = self.get_clock().now()
            for robot_idx in range(self.num_robots):
                pose = msg.poses[robot_idx]
                if pose.position.x == 999999.0:
                    continue  # Dummy ignorieren

                # Zufallsrauschen hinzufügen
                z = np.array([
                    pose.position.x + np.random.normal(0, 0.2),
                    pose.position.y + np.random.normal(0, 0.2)
                ])
                self.last_update_time[robot_idx] = now
                self.run_ekf_step(robot_idx, z, update=True)
        return callback

    def run_ekf_step(self, robot_idx, z=None, update=False):
        ekf = self.filters[robot_idx]
        x, P, Q, R, dt = ekf['x'], ekf['P'], ekf['Q'], ekf['R'], ekf['dt']

        # Prädiktion
        F = self.jacobian_F(x, dt)
        x_pred = self.f(x, dt)
        P_pred = F @ P @ F.T + Q

        if update and z is not None:
            # Update mit Messung
            H = np.array([
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0]
            ])
            y = z - H @ x_pred
            S = H @ P_pred @ H.T + R
            K = P_pred @ H.T @ np.linalg.inv(S)
            x_new = x_pred + K @ y
            P_new = (np.eye(5) - K @ H) @ P_pred
        else:
            # Nur Prädiktion
            x_new, P_new = x_pred, P_pred

        ekf['x'], ekf['P'] = x_new, P_new

        # Nach jedem Schritt veröffentlichen
        self.publish_estimates()
        self.publish_full_state()


    def f(self, x, dt):
        x_, y_, theta, v, omega = x
        theta_new = theta + omega * dt
        x_new = x_ + v * math.cos(theta) * dt
        y_new = y_ + v * math.sin(theta) * dt
        return np.array([x_new, y_new, theta_new, v, omega])

    def jacobian_F(self, x, dt):
        _, _, theta, v, _ = x
        return np.array([
            [1, 0, -v * math.sin(theta) * dt, math.cos(theta) * dt, 0],
            [0, 1,  v * math.cos(theta) * dt, math.sin(theta) * dt, 0],
            [0, 0, 1, 0, dt],
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1]
        ])

    def predict_missing(self):
        """Führe Prädiktion durch, wenn länger als 100ms keine neue Messung kam."""
        now = self.get_clock().now()
        for i in range(self.num_robots):
            dt_ns = (now.nanoseconds - self.last_update_time[i].nanoseconds)
            if dt_ns > 1e8:  # 100 ms = 1e8 ns
                self.run_ekf_step(i, update=False)

    def publish_estimates(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for ekf in self.filters:
            x, y, theta = ekf['x'][0], ekf['x'][1], ekf['x'][2]

            pose = Pose()
            pose.position.x = x
            pose.position.y = y

            # yaw → quaternion (nur z-Rotation)
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            pose.orientation.z = qz
            pose.orientation.w = qw

            msg.poses.append(pose)

        self.estimates_pub.publish(msg)

    def publish_full_state(self):
        msg = RobotStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        for ekf in self.filters:
            x_vec = ekf['x']
            state = RobotState()
            state.header.stamp = msg.header.stamp
            state.x = x_vec[0]
            state.y = x_vec[1]
            state.theta = x_vec[2]
            state.v = x_vec[3]
            state.omega = x_vec[4]
            msg.states.append(state)

        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
