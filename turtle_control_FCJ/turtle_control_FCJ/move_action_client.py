import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from custom_interfaces.action import MoveTurtle

class MoveTurtleActionClient(Node):
    def __init__(self):
        super().__init__('move_turtle_action_client')
        self._action_client = ActionClient(self, MoveTurtle, 'move_turtle')

    def send_goal(self, x, y):
        goal_msg = MoveTurtle.Goal()
        goal_msg.x = x
        goal_msg.y = y

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal result received')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: current_x = {feedback_msg.current_x}, current_y = {feedback_msg.current_y}')

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveTurtleActionClient()

    # Enviar a meta de movimento
    action_client.send_goal(5.0, 5.0)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
