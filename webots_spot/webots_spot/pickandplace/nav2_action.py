import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose



class NavigateToPoseActionClient(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._action_client.wait_for_server()

    def send_goal(self, p, o):
        self.reached = False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.pose.position.x = p[0]
        goal_msg.pose.pose.position.y = p[1]
        goal_msg.pose.pose.position.z = p[2]

        goal_msg.pose.pose.orientation.x = o[0]
        goal_msg.pose.pose.orientation.y = o[1]
        goal_msg.pose.pose.orientation.z = o[2]
        goal_msg.pose.pose.orientation.w = o[3]

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # self.get_logger().info(str(future))
        self.reached = True

    def feedback_callback(self, feedback_msg):
        # self.get_logger().info(str(feedback_msg))
        pass

    def get_nav2_status(self):
        return self.reached


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateToPoseActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()