import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import random
import math
from tf_transformations import quaternion_from_euler

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.declare_parameter('num_robots', 1)
        self.declare_parameter('room_size', [40.0, 30.0])  # [width, height]

        self.num_robots = self.get_parameter('num_robots').value
        self.room_size = self.get_parameter('room_size').value

        self.robot_pub = self.create_publisher(PoseArray, '/robots/pose', 10)
        self.create_timer(0.1, self.update_positions)
        self.create_timer(3.0, self.change_directions)  # Timer to change directions every 3 seconds

        # Initialize robot positions and directions
        self.robots = []
        for _ in range(self.num_robots):
            x = random.uniform(-self.room_size[0] / 2, self.room_size[0] / 2)
            y = random.uniform(-self.room_size[1] / 2, self.room_size[1] / 2)
            angle = random.uniform(0, 2 * math.pi)  # Random initial direction
            self.robots.append({'x': x, 'y': y, 'angle': angle})

    def update_positions(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for robot in self.robots:
            # Update position
            robot['x'] += math.cos(robot['angle']) * 0.5  # Move forward (0.5 units per step)
            robot['y'] += math.sin(robot['angle']) * 0.5

            # Check for collisions with walls and bounce
            if robot['x'] <= -self.room_size[0] / 2 or robot['x'] >= self.room_size[0] / 2:
                robot['angle'] = math.pi - robot['angle']  # Reverse x direction
                robot['x'] = max(min(robot['x'], self.room_size[0] / 2), -self.room_size[0] / 2)  # Keep within bounds

            if robot['y'] <= -self.room_size[1] / 2 or robot['y'] >= self.room_size[1] / 2:
                robot['angle'] = -robot['angle']  # Reverse y direction
                robot['y'] = max(min(robot['y'], self.room_size[1] / 2), -self.room_size[1] / 2)  # Keep within bounds

            # Create Pose message
            pose = Pose()
            pose.position.x = robot['x']
            pose.position.y = robot['y']
            pose.position.z = 0.0

            # Calculate orientation from angle
            quaternion = quaternion_from_euler(0, 0, robot['angle'])
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            pose_array.poses.append(pose)

        # Publish updated positions
        self.robot_pub.publish(pose_array)

    def change_directions(self):
        for robot in self.robots:
            robot['angle'] = random.uniform(0, 2 * math.pi)  # Assign a new random direction
        self.get_logger().info("Robot directions updated.")

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
