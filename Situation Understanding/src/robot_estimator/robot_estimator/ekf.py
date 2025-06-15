import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import threading # Für Thread-Sicherheit beim Zugriff auf den Detektionspuffer

class SingleEKF:
    """
    Diese Klasse kapselt die Logik für einen einzelnen Extended Kalman Filter.
    """
    def __init__(self, initial_pose: Pose, node: Node, track_id: int):
        self.node = node # Referenz zum ROS-Node, um Logger und Clock zu nutzen
        self.track_id = track_id

        quat = [initial_pose.orientation.x, initial_pose.orientation.y,
                initial_pose.orientation.z, initial_pose.orientation.w]
        initial_yaw = R.from_quat(quat).as_euler('zyx')[0] 

        self.x = np.array([initial_pose.position.x, initial_pose.position.y, initial_yaw, 0.0, 0.0])
        self.P = np.eye(5) * 0.1 

        self.R = np.diag([0.1, 0.1])  # Messrauschen für x/y Detektionen (Kamera und Lidar)

        self.Q = np.diag([0.05, 0.05, 0.01, 0.1, 0.1])  # Prozessrauschen

        self.last_time = self.node.get_clock().now()
        self.missed_detections = 0 

        self.publisher = self.node.create_publisher(Odometry, f'/ekf/robot_{self.track_id}/odom', 10)
        self.node.get_logger().info(f'Initialized new EKF for Robot {self.track_id} at ({initial_pose.position.x:.2f}, {initial_pose.position.y:.2f})')


    def predict(self, dt: float):
        x, y, theta, v, omega = self.x

        x_pred = x + v * np.cos(theta) * dt
        y_pred = y + v * np.sin(theta) * dt
        theta_pred = self.normalize_angle(theta + omega * dt)
        v_pred = v
        omega_pred = omega
        self.x = np.array([x_pred, y_pred, theta_pred, v_pred, omega_pred])

        F = np.eye(5)
        F[0, 2] = -v * np.sin(theta) * dt
        F[0, 3] = np.cos(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt
        F[1, 3] = np.sin(theta) * dt
        F[2, 4] = dt

        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement: Pose):
        # H ist weiterhin eine 2x5 Matrix, da jede Detektion (x,y) liefert
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])

        z_measured = np.array([measurement.position.x, measurement.position.y])
        z_predicted = H @ self.x 

        y = z_measured - z_predicted

        S = H @ self.P @ H.T + self.R

        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y

        self.P = (np.eye(5) - K @ H) @ self.P
        
        self.missed_detections = 0 

    def get_mahalanobis_distance(self, measurement: Pose):
        H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])
        z_measured = np.array([measurement.position.x, measurement.position.y])
        z_predicted = H @ self.x 
        y = z_measured - z_predicted
        S = H @ self.P @ H.T + self.R
        
        try:
            d2 = y.T @ np.linalg.inv(S) @ y
            return d2
        except np.linalg.LinAlgError:
            self.node.get_logger().warn(f"Singular matrix in Mahalanobis distance for EKF {self.track_id}. Skipping.")
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

        pose_covariance_matrix = np.zeros((6, 6))
        pose_covariance_matrix[0:2, 0:2] = self.P[0:2, 0:2]
        pose_covariance_matrix[5, 5] = self.P[2, 2]
        pose_covariance_matrix[0, 5] = self.P[0, 2]
        pose_covariance_matrix[5, 0] = self.P[2, 0]
        pose_covariance_matrix[1, 5] = self.P[1, 2]
        pose_covariance_matrix[5, 1] = self.P[2, 1]

        odom.pose.covariance = pose_covariance_matrix.flatten().tolist()

        odom.twist.twist.linear.x = self.x[3]
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.x[4]

        twist_covariance_matrix = np.zeros((6, 6))
        twist_covariance_matrix[0, 0] = self.P[3, 3]
        twist_covariance_matrix[5, 5] = self.P[4, 4]
        twist_covariance_matrix[0, 5] = self.P[3, 4]
        twist_covariance_matrix[5, 0] = self.P[4, 3]

        odom.twist.covariance = twist_covariance_matrix.flatten().tolist()

        self.publisher.publish(odom)

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle


class EKFTracker(Node):
    def __init__(self):
        super().__init__('ekf_tracker')

        self.declare_parameter('max_tracks', 2) 
        self.max_tracks = self.get_parameter('max_tracks').value

        # NEU: Abonnieren aller Detektions-Topics
        self.camera2_sub = self.create_subscription(
            PoseArray,
            '/detections/camera_2',
            self.camera2_detection_callback,
            10
        )
        self.camera1_sub = self.create_subscription(
            PoseArray,
            '/detections/camera_1',
            self.camera1_detection_callback,
            10
        )
        self.lidar_sub = self.create_subscription(
            PoseArray,
            '/detections/lidar_1', # Annahme für Lidar-Topic
            self.lidar_detection_callback,
            10
        )

        self.get_logger().info('EKF Tracker node started, subscribing to multiple detection topics.')

        self.tracked_ekfs = {} 
        self.next_track_id = 0 

        self.GATE_THRESHOLD = 5.99 
        self.MAX_MISSED_DETECTIONS = 10 

        # NEU: Puffer für eingehende Detektionen
        self.detection_buffer = []
        self.buffer_lock = threading.Lock() # Für Thread-sicheren Zugriff auf den Puffer

        self.timer = self.create_timer(0.1, self.timer_callback) 
        self.last_timer_time = self.get_clock().now()

    # NEU: Separater Callback für jede Sensorquelle, die in den Puffer schreibt
    def camera1_detection_callback(self, msg: PoseArray):
        with self.buffer_lock:
            self.detection_buffer.extend(msg.poses)
            # self.get_logger().info(f"Buffered {len(msg.poses)} detections from Camera 0. Total buffered: {len(self.detection_buffer)}")

    def camera2_detection_callback(self, msg: PoseArray):
        with self.buffer_lock:
            self.detection_buffer.extend(msg.poses)
            # self.get_logger().info(f"Buffered {len(msg.poses)} detections from Camera 1. Total buffered: {len(self.detection_buffer)}")

    def lidar_detection_callback(self, msg: PoseArray):
        with self.buffer_lock:
            self.detection_buffer.extend(msg.poses)
            # self.get_logger().info(f"Buffered {len(msg.poses)} detections from Lidar. Total buffered: {len(self.detection_buffer)}")

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_timer_time).nanoseconds / 1e9
        self.last_timer_time = now

        # Prädiktionsschritt für alle bestehenden Tracks
        # Wichtig: Zuerst alle prädizieren, dann alle Detektionen verarbeiten
        for track_id, ekf in list(self.tracked_ekfs.items()):
            ekf.predict(dt)
            # `missed_detections` wird erst nach dem Assoziationsschritt inkrementiert

        # --- Detektionen aus dem Puffer entnehmen und verarbeiten ---
        current_detections = []
        with self.buffer_lock:
            current_detections = list(self.detection_buffer) # Kopie erstellen
            self.detection_buffer.clear() # Puffer leeren

        # Debug-Ausgabe: Wieviele Detektionen wurden in dieser Runde verarbeitet
        # if len(current_detections) > 0:
        #    self.get_logger().info(f"Processing {len(current_detections)} detections this cycle.")
        
        # --- Assoziationsschritt: Ordne Detektionen bestehenden Tracks zu ---
        # Die Logik bleibt gleich, da 'current_detections' nun ALLE Detektionen enthält.
        
        # Initialisiere die Detektionen, die zugeordnet werden können
        unassigned_detections_indices = set(range(len(current_detections)))
        
        # Halte eine Liste von (Track-ID, Detektion) für die Updates
        updates_to_perform = []

        # Für jeden bestehenden Track, finde die am besten passende Detektion
        for track_id, ekf in list(self.tracked_ekfs.items()): # Loop durch EKFs
            best_det_idx = -1
            min_cost = float('inf')

            # Suche die beste Detektion für diesen EKF unter den noch unzugeordneten
            for det_idx in list(unassigned_detections_indices): # Nur die noch nicht zugeordneten prüfen
                detection = current_detections[det_idx]
                cost = ekf.get_mahalanobis_distance(detection)
                
                if cost < min_cost:
                    min_cost = cost
                    best_det_idx = det_idx
            
            # Wenn eine geeignete Detektion gefunden wurde (innerhalb des Gates)
            if best_det_idx != -1 and min_cost < self.GATE_THRESHOLD:
                updates_to_perform.append((track_id, current_detections[best_det_idx]))
                unassigned_detections_indices.remove(best_det_idx) # Detektion als zugeordnet markieren
                self.get_logger().info(f"Proposed assignment: EKF {track_id} with detection {best_det_idx} (Cost: {min_cost:.2f})")
            else:
                # Wenn keine Detektion gefunden oder zu teuer, inkrementiere missed_detections
                ekf.missed_detections += 1


        # --- Aktualisiere EKFs mit den zugeordneten Detektionen ---
        # Dies ist der eigentliche Update-Schritt
        for track_id, detection in updates_to_perform:
            ekf = self.tracked_ekfs[track_id]
            ekf.update(detection)
            ekf.missed_detections = 0 # Zurücksetzen, da erfolgreich aktualisiert
            self.get_logger().info(f"Updated EKF {track_id} with detection (x:{detection.position.x:.2f}, y:{detection.position.y:.2f}). Missed: {ekf.missed_detections}")

        # --- Track Management: Löschen alter Tracks ---
        tracks_to_remove = []
        for track_id, ekf in list(self.tracked_ekfs.items()):
            if ekf.missed_detections > self.MAX_MISSED_DETECTIONS:
                self.get_logger().info(f'EKF for Robot {track_id} removed due to too many missed detections ({ekf.missed_detections}).')
                tracks_to_remove.append(track_id)
            else:
                ekf.publish_odometry() # Odometrie für aktive Tracks veröffentlichen
        
        for track_id in tracks_to_remove:
            del self.tracked_ekfs[track_id]

        # --- Initialisierung neuer Tracks für unzugeordnete Detektionen ---
        for det_idx in unassigned_detections_indices:
            detection = current_detections[det_idx]
            if len(self.tracked_ekfs) < self.max_tracks:
                # Finde die kleinste freie track_id
                used_ids = set(self.tracked_ekfs.keys())
                new_id = 0
                while new_id in used_ids:
                    new_id += 1
                
                new_ekf = SingleEKF(detection, self, new_id)
                self.tracked_ekfs[new_id] = new_ekf
                self.get_logger().info(f"Initialized new EKF for Robot {new_id} at ({detection.position.x:.2f}, {detection.position.y:.2f})")
            else:
                self.get_logger().warn(f"Max number of EKFs ({self.max_tracks}) reached. Cannot initialize new EKF for detection at ({detection.position.x:.2f}, {detection.position.y:.2f}).")
        
def main(args=None):
    rclpy.init(args=args)
    ekf_tracker = EKFTracker()
    rclpy.spin(ekf_tracker)
    ekf_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()