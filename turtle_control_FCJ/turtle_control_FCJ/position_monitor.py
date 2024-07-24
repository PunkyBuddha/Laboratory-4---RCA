import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from turtlesim.msg import Pose
import math

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        self.goal_index = 0
        self.goals = [
            Pose2D(x=5.0, y=5.0, theta=0.0),
            Pose2D(x=8.0, y=8.0, theta=0.0),
            Pose2D(x=3.0, y=6.0, theta=0.0)
        ]
        self.current_pose = None

        # Publisher para o novo objetivo
        self.publisher_ = self.create_publisher(Pose2D, '/FCJ/goal', 10)

        # Subscriber para a pose atual
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Timer para verificar a posição a cada 1 segundo
        self.timer = self.create_timer(1.0, self.check_position)

    def pose_callback(self, msg):
        self.current_pose = msg

    def check_position(self):
        if self.current_pose is None:
            return

        goal = self.goals[self.goal_index]
        distance = self.calculate_distance(self.current_pose, goal)

        # Verifica se chegou na posição desejada
        if distance < 0.5:  # Tolerância de 0.5 unidades
            self.goal_index = (self.goal_index + 1) % len(self.goals)
            new_goal = self.goals[self.goal_index]
            self.publisher_.publish(new_goal)
            self.get_logger().info(f'Novo objetivo publicado: x={new_goal.x}, y={new_goal.y}, theta={new_goal.theta}')

    def calculate_distance(self, pose, goal):
        return math.sqrt((pose.x - goal.x)**2 + (pose.y - goal.y)**2)

def main(args=None):
    rclpy.init(args=args)
    position_monitor = PositionMonitor()
    rclpy.spin(position_monitor)
    position_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
