import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from custom_interfaces.action import MoveTurtle
import math

class MoveTurtleActionServer(Node):
    def __init__(self):
        super().__init__('move_turtle_action_server')
        self._action_server = ActionServer(
            self,
            MoveTurtle,
            'move_turtle',
            self.execute_callback)
        self._publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self._pose = None

    def pose_callback(self, msg):
        self._pose = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = MoveTurtle.Feedback()
        result = MoveTurtle.Result()

        target_x = goal_handle.request.x
        target_y = goal_handle.request.y

        while not self.is_close_to_target(target_x, target_y):
            if self._pose is None:
                continue

            twist = self.compute_twist(target_x, target_y)
            self._publisher.publish(twist)

            feedback_msg.current_x = self._pose.x
            feedback_msg.current_y = self._pose.y
            goal_handle.publish_feedback(feedback_msg)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.success = False
                return result

        goal_handle.succeed()
        result.success = True
        self.get_logger().info('Goal succeeded')
        return result

    def is_close_to_target(self, x, y):
        if self._pose is None:
            return False
        return math.sqrt((self._pose.x - x) ** 2 + (self._pose.y - y) ** 2) < 0.5

    def compute_twist(self, x, y):
        twist = Twist()
        angle_to_goal = math.atan2(y - self._pose.y, x - self._pose.x)
        twist.linear.x = 2.0 * math.sqrt((x - self._pose.x) ** 2 + (y - self._pose.y) ** 2)
        twist.angular.z = 6.0 * (angle_to_goal - self._pose.theta)
        return twist

def main(args=None):
    rclpy.init(args=args)
    move_turtle_action_server = MoveTurtleActionServer()
    rclpy.spin(move_turtle_action_server)
    move_turtle_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
