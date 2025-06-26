import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import random
import math
from scipy.spatial.transform import Rotation as R

class RobotControllerNode(Node):
    """
    A ROS2 node that simulates robots moving within a bounded room. 
    The node publishes the positions and orientations of robots at regular intervals.

    Attributes:
        num_robots (int): Number of robots to simulate.
        room_size (list): Dimensions of the room [width, height].
        robots (list): List of dictionaries containing robot positions and directions.
    """
    def __init__(self):
        """
        Initializes the robot controller node, sets parameters, and creates publishers and timers.
        """
        super().__init__('robot_controller')
        # Declare parameters for the number of robots and room size
        self.declare_parameter('num_robots', 1)
        self.declare_parameter('room_size', [40.0, 30.0])  # [width, height]

        # Retrieve parameter values
        self.num_robots = self.get_parameter('num_robots').value
        self.room_size = self.get_parameter('room_size').value

        # Create publisher for robot positions
        self.robot_pub = self.create_publisher(PoseArray, '/robots/pose', 10)
<<<<<<< HEAD
        # Create timers for updating positions and changing directions
        self.create_timer(0.1, self.update_positions)  # Update positions every 0.1 seconds
        self.create_timer(10.0, self.change_directions)  # Change directions every 10 seconds
=======
        self.create_timer(0.1, self.update_positions)
        self.create_timer(10.0, self.change_directions)  # Timer to change directions every 3 seconds
>>>>>>> b1ae4f4faf8a6400a5c7459e37df1de845e88f30

        # Initialize robot positions and directions
        self.robots = []
        for _ in range(self.num_robots):
            x = random.uniform(-self.room_size[0] / 2, self.room_size[0] / 2)  # Random x position
            y = random.uniform(-self.room_size[1] / 2, self.room_size[1] / 2)  # Random y position
            angle = random.uniform(0, 2 * math.pi)  # Random initial direction
            self.robots.append({'x': x, 'y': y, 'angle': angle})

    def update_positions(self):
        """
        Updates the positions of robots based on their current direction and publishes the updated positions.
        """
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"  # Set frame ID to "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()  # Set timestamp

        for robot in self.robots:
            # Apply a small random change to the robot's direction
            robot['angle'] += random.uniform(-0.2, 0.2)  # Random angular adjustment

<<<<<<< HEAD
            # Update the robot's position based on its direction
            robot['x'] += math.cos(robot['angle']) * 0.3  # Move in the x direction
            robot['y'] += math.sin(robot['angle']) * 0.3  # Move in the y direction
=======
            # Update position
            robot['x'] += math.cos(robot['angle']) * 0.3 
            robot['y'] += math.sin(robot['angle']) * 0.3
>>>>>>> b1ae4f4faf8a6400a5c7459e37df1de845e88f30

            # Check for collisions with walls and bounce back
            if robot['x'] <= -self.room_size[0] / 2 or robot['x'] >= self.room_size[0] / 2:
                robot['angle'] = math.pi - robot['angle']  # Reverse x direction
                robot['x'] = max(min(robot['x'], self.room_size[0] / 2), -self.room_size[0] / 2)  # Keep within bounds

            if robot['y'] <= -self.room_size[1] / 2 or robot['y'] >= self.room_size[1] / 2:
                robot['angle'] = -robot['angle']  # Reverse y direction
                robot['y'] = max(min(robot['y'], self.room_size[1] / 2), -self.room_size[1] / 2)  # Keep within bounds

            # Create Pose message for the robot
            pose = Pose()
            pose.position.x = robot['x']  # Set x position
            pose.position.y = robot['y']  # Set y position
            pose.position.z = 0.0  # Z-coordinate is fixed

            # Calculate orientation from the robot's angle
            quaternion = R.from_euler('z', robot['angle']).as_quat()  # Convert angle to quaternion
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            pose_array.poses.append(pose)  # Add pose to PoseArray

        # Publish updated positions
        self.robot_pub.publish(pose_array)

    def change_directions(self):
        """
        Assigns new random directions to all robots.
        """
        for robot in self.robots:
            robot['angle'] = random.uniform(0, 2 * math.pi)  # Assign a new random direction
        self.get_logger().info("Robot directions updated.")  # Log direction update

def main(args=None):
    """
    Entry point for the robot controller node. Initializes the node and starts spinning.
    """
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
