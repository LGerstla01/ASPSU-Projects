import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64MultiArray

class RobotStateEstimator:
    def __init__(self):
        rclpy.init_node('robot_state_estimator')

        # Parameters
        self.num_robots = rclpy.get_param('~num_robots', 2)
        self.corridor_bounds = rclpy.get_param('~corridor_bounds', [-5.0, 5.0])  # Example bounds

        # State: [x, y, yaw, velocity] for each robot
        self.state = np.zeros((self.num_robots, 4))
        self.covariance = np.array([np.eye(4) for _ in range(self.num_robots)])  # Initial covariance

        # Noise parameters
        self.process_noise = np.diag([0.1, 0.1, 0.05, 0.1])  # Process noise covariance
        self.measurement_noise = np.diag([0.5, 0.5])  # Measurement noise covariance

        # Subscribers and Publishers
        self.pose_sub = rclpy.Subscriber('/robots/pose', PoseArray, self.pose_callback)
        self.state_pub = rclpy.Publisher('/robots/estimated_state', Float64MultiArray, queue_size=10)

    def pose_callback(self, msg):
        measurements = []
        for pose in msg.poses:
            measurements.append([pose.position.x, pose.position.y])
        measurements = np.array(measurements)

        for i in range(self.num_robots):
            if self.is_in_corridor(measurements[i]):
                self.measurement_update(i, measurements[i])
            else:
                self.prediction_update(i)

        self.publish_state()

    def is_in_corridor(self, position):
        return self.corridor_bounds[0] <= position[0] <= self.corridor_bounds[1]

    def prediction_update(self, robot_idx):
        # Prediction step of EKF
        F = np.eye(4)  # State transition model
        self.state[robot_idx] = F @ self.state[robot_idx]
        self.covariance[robot_idx] = F @ self.covariance[robot_idx] @ F.T + self.process_noise

    def measurement_update(self, robot_idx, measurement):
        # Measurement step of EKF
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  # Measurement model
        z = measurement  # Measurement
        x = self.state[robot_idx]  # Predicted state
        P = self.covariance[robot_idx]  # Predicted covariance

        # Kalman gain
        S = H @ P @ H.T + self.measurement_noise
        K = P @ H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.state[robot_idx] = x + K @ (z - H @ x)
        self.covariance[robot_idx] = (np.eye(4) - K @ H) @ P

    def publish_state(self):
        msg = Float64MultiArray()
        msg.data = self.state.flatten().tolist()
        self.state_pub.publish(msg)

if __name__ == '__main__':
    try:
        estimator = RobotStateEstimator()
        rclpy.spin()
    except rclpy.ROSInterruptException:
        pass
