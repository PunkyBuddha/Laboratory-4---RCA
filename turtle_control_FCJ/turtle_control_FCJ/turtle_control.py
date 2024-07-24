import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import math

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')
        self.init_variables()
        self.init_publisher()
        self.init_subscribers()

    def init_publisher(self):
        self.publisher = self.create_publisher(Pose2D, '/SeuNome/goal', 10)
        self.get_logger().info("Publisher initialized")

    def init_subscribers(self):
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)
        self.get_logger().info("Subscribers initialized")

    def init_variables(self):
        self.x = 0.0
        self.y = 0.0
        self.positions = [(2.0, 2.0), (4.0, 4.0), (6.0, 6.0)]
        self.goal_index = 0
        self.threshold = 0.1

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.check_goal()

    def check_goal(self):
        goal_x, goal_y = self.positions[self.goal_index]
        distance = math.sqrt((self.x - goal_x)**2 + (self.y - goal_y)**2)
        if distance < self.threshold:
            self.goal_index = (self.goal_index + 1) % len(self.positions)
            self.publish_goal()

    def publish_goal(self):
        goal_x, goal_y = self.positions[self.goal_index]
        msg = Pose2D()
        msg.x = goal_x
        msg.y = goal_y
        self.publisher.publish(msg)
        self.get_logger().info(f"New goal published: {goal_x}, {goal_y}")

def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
