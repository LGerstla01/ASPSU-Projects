import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
import tf_transformations

class CameraDetectionNode(Node):
    def __init__(self):
        super().__init__('camera_detection_node')
        self.declare_parameter('num_cameras', 2)
        self.declare_parameter('room_size', [40.0, 30.0, 3.0])  # [width, height, height]
        self.declare_parameter('camera_fov', 40.0)  # Field of view in degrees
        self.declare_parameter('noise', 0.3)

        self.num_cameras = self.get_parameter('num_cameras').value
        self.room_size = self.get_parameter('room_size').value
        self.camera_fov = np.radians(self.get_parameter('camera_fov').value)  # Convert to radians
        self.noise = self.get_parameter('noise').value

        self.robot_sub = self.create_subscription(PoseArray, '/robots/pose', self.pos_callback, 10)
        self.camera_positions = self.initialize_camera_positions()
        self.camera_publishers = self.initialize_camera_publishers()

        self.create_timer(0.1, self.robot_callback)  # Timer to publish dummy data
        self.msg = PoseArray()

    def initialize_camera_positions(self):
        # Define camera positions in the corners of the room
        positions = [
            (self.room_size[0] / 2, self.room_size[1] / 2),
            (-self.room_size[0] / 2, self.room_size[1] / 2),
            (self.room_size[0] / 2, -self.room_size[1] / 2),
            (-self.room_size[0] / 2, -self.room_size[1] / 2)
        ]
        return positions[:self.num_cameras]

    def initialize_camera_publishers(self):
        publishers = []
        for i in range(self.num_cameras):
            topic_name = f'/detections/camera_{i+1}'
            publishers.append(self.create_publisher(PoseArray, topic_name, 10))
        return publishers
    
    def pos_callback(self, msg):
        self.msg = msg

    def robot_callback(self):
        detections_per_camera = [PoseArray() for _ in range(self.num_cameras)]
        for i, detections in enumerate(detections_per_camera):
            detections.header = Header()
            detections.header.stamp = self.get_clock().now().to_msg()
            detections.header.frame_id = "map"

        for robot_pose in self.msg.poses:
            for i, camera_pos in enumerate(self.camera_positions):
                if self.is_in_fov(camera_pos, i, robot_pose):
                    noisy_pose = self.add_noise(robot_pose)
                    detections_per_camera[i].poses.append(noisy_pose)
                
        for i, detections in enumerate(detections_per_camera):
            if detections.poses:
                self.camera_publishers[i].publish(detections)

    def is_in_fov(self, camera_pos, camera_num, robot_pose):
        # Calculate relative position of the robot to the camera
        dx = robot_pose.position.x - camera_pos[0]
        dy = robot_pose.position.y - camera_pos[1]
        distance = np.sqrt(dx**2 + dy**2)

        # Check if the robot is within the camera's range and field of view
        #if distance > self.room_size[0]:  # Camera range is the room length
        #    return False

        angle = self.normalize_angle(np.arctan2(dy, dx))
        camera_angle = self.normalize_angle(np.arctan2(camera_pos[1], camera_pos[0]) + np.pi)
        if camera_num == 0:
            camera_angle = self.normalize_angle(np.radians(225))
        elif camera_num == 1:
            camera_angle = self.normalize_angle(np.radians(315))
        elif camera_num == 2:
            camera_angle = self.normalize_angle(np.radians(45))
        elif camera_num == 3:
            camera_angle = self.normalize_angle(np.radians(135))

        angle_diff = abs(angle - camera_angle)

        return angle_diff <= self.camera_fov / 2
    
    def normalize_angle(self, angle):
        # Normalize the angle to be within -pi to pi
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def add_noise(self, pose):
        noisy_pose = Pose()
        noisy_pose.position.x = pose.position.x + np.random.normal(0, self.noise)  # Add Gaussian noise
        noisy_pose.position.y = pose.position.y + np.random.normal(0, self.noise)
        noisy_pose.position.z = pose.position.z  
        
        # Add noise to yaw angle
        # Convert quaternion to euler
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat)
        yaw += np.random.normal(0, self.noise/3)  # Add Gaussian noise to yaw
        # Convert back to quaternion
        noisy_quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        noisy_pose.orientation.x = noisy_quat[0]
        noisy_pose.orientation.y = noisy_quat[1]
        noisy_pose.orientation.z = noisy_quat[2]
        noisy_pose.orientation.w = noisy_quat[3]
        return noisy_pose

def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
