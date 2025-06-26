import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation as R

# Patch numpy for compatibility with tf_transformations
if not hasattr(np, 'float'):
    np.float = float  # Add alias for compatibility

class CameraDetectionNode(Node):
    """
    CameraDetectionNode is a ROS2 node responsible for simulating camera-based robot detection in a predefined room environment.
    This node initializes a set of cameras positioned at the corners of a rectangular room and simulates their ability to detect robots within their field of view (FOV). The detections are published as PoseArray messages to specific topics for each camera.
    Attributes:
        num_cameras (int): Number of cameras to simulate. Default is 2.
        room_size (list): Dimensions of the room [width, height, height]. Default is [40.0, 30.0, 3.0].
        camera_fov (float): Field of view of the cameras in radians. Default is 40 degrees converted to radians.
        noise (float): Gaussian noise level added to robot detections. Default is 0.3.
        robot_sub (Subscription): ROS2 subscription to the topic '/robots/pose' for receiving robot poses.
        camera_positions (list): List of tuples representing the positions of the cameras in the room.
        camera_publishers (list): List of ROS2 publishers for publishing detections from each camera.
        msg (PoseArray): Stores the latest robot poses received from the '/robots/pose' topic.
    Methods:
        __init__(): Initializes the node, declares parameters, sets up subscriptions, publishers, and timers.
        initialize_camera_positions(): Defines the positions of the cameras based on the room size and number of cameras.
        initialize_camera_publishers(): Creates publishers for each camera to publish detection messages.
        pos_callback(msg): Callback function for the '/robots/pose' subscription. Updates the latest robot poses.
        robot_callback(): Timer callback function that processes robot poses, checks for detections, and publishes them.
        is_in_fov(camera_pos, camera_num, robot_pose): Determines whether a robot is within the field of view of a specified camera.
        normalize_angle(angle): Normalizes an angle to be within the range [-pi, pi].
        add_noise(pose): Adds Gaussian noise to the position and orientation of a given pose.
    Usage:
        This node is intended for simulating camera-based robot detection in a controlled environment. It can be used for testing algorithms that rely on robot pose detection and localization.
    """
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
        # Create PoseArray objects for each camera to store detections
        detections_per_camera = [PoseArray() for _ in range(self.num_cameras)]
        for i, detections in enumerate(detections_per_camera):
            detections.header = Header()  # Initialize header for each PoseArray
            detections.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
            detections.header.frame_id = "map"  # Set frame ID to "map"

        # Iterate through all robot poses received
        for robot_pose in self.msg.poses:
            for i, camera_pos in enumerate(self.camera_positions):
                # Check if the robot is within the field of view of the camera
                if self.is_in_fov(camera_pos, i, robot_pose):
                    noisy_pose = self.add_noise(robot_pose)  # Add noise to the robot pose
                    detections_per_camera[i].poses.append(noisy_pose)  # Add noisy pose to detections

        # Publish detections for each camera
        for i, detections in enumerate(detections_per_camera):
            if detections.poses:  # Only publish if there are detections
                self.camera_publishers[i].publish(detections)

    def is_in_fov(self, camera_pos, camera_num, robot_pose):
        """
        Determines whether a robot is within the field of view (FOV) of a specified camera.

        Args:
            camera_pos (tuple): A tuple (x, y) representing the position of the camera in the world frame.
            camera_num (int): The identifier of the camera (0, 1, 2, or 3), used to determine its predefined orientation.
            robot_pose (object): An object representing the robot's pose, which includes a `position` attribute with `x` and `y` coordinates.

        Returns:
            bool: True if the robot is within the camera's field of view, False otherwise.

        Notes:
            - The camera's FOV is determined by the `self.camera_fov` attribute.
            - The function calculates the relative position and angle between the robot and the camera.
            - Predefined camera angles are set based on the camera number:
            - Camera 0: 225 degrees
            - Camera 1: 315 degrees
            - Camera 2: 45 degrees
            - Camera 3: 135 degrees
            - The angular difference between the robot's position and the camera's orientation is checked against half of the camera's FOV.
        """
        # Calculate relative position of the robot to the camera
        dx = robot_pose.position.x - camera_pos[0]
        dy = robot_pose.position.y - camera_pos[1]
        distance = np.sqrt(dx**2 + dy**2)  # Compute distance between robot and camera

        # Calculate angle between robot and camera
        angle = self.normalize_angle(np.arctan2(dy, dx))
        camera_angle = self.normalize_angle(np.arctan2(camera_pos[1], camera_pos[0]) + np.pi)

        # Set predefined camera angles based on camera number
        if camera_num == 0:
            camera_angle = self.normalize_angle(np.radians(225))
        elif camera_num == 1:
            camera_angle = self.normalize_angle(np.radians(315))
        elif camera_num == 2:
            camera_angle = self.normalize_angle(np.radians(45))
        elif camera_num == 3:
            camera_angle = self.normalize_angle(np.radians(135))

        # Compute the angular difference and check if it is within the field of view
        angle_diff = abs(angle - camera_angle)

        return angle_diff <= self.camera_fov / 2  # Return True if within field of view
    
    def normalize_angle(self, angle):
        # Normalize the angle to be within -pi to pi
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def add_noise(self, pose):
        """
        Adds Gaussian noise to the position and orientation of a given pose.

        This method perturbs the position coordinates (x, y) and the yaw angle of the orientation
        of the input pose using Gaussian noise. The noise applied to the yaw angle is scaled down
        by a factor of 3 compared to the position noise.

        Args:
            pose (Pose): The input pose to which noise will be added. The pose should have position
                         attributes (x, y, z) and orientation attributes (x, y, z, w) representing
                         a quaternion.

        Returns:
            Pose: A new pose object with added noise in position and orientation.
        """
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
        rotation = R.from_quat(quat)
        roll, pitch, yaw = rotation.as_euler('xyz')
        yaw += np.random.normal(0, self.noise / 3)  # Add Gaussian noise to yaw
        # Convert back to quaternion
        noisy_quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
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
