import rclpy
from webots_spot_msgs.srv import SpotMotion
from rclpy.node import Node


class GripperClient(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.close_cli = self.create_client(SpotMotion, '/Spot/close_gripper')
        self.open_cli = self.create_client(SpotMotion, '/Spot/open_gripper')
        self.req = SpotMotion.Request()

    def open_callback(self, future_result):
        response = future_result.result()
        self.get_logger().info(f"result：{response.answer}")

    def close_callback(self, future_result):
        response = future_result.result()
        self.get_logger().info(f"result：{response.answer}")

    def open_request(self):
        self.result = self.open_cli.call_async(self.req).add_done_callback(self.open_callback)

    def close_request(self):
        self.result = self.close_cli.call_async(self.req).add_done_callback(self.close_callback)


def main(args=None):
    rclpy.init(args=args)
    pick_place = GripperClient('pick_place')
    rclpy.spin(pick_place)
    pick_place.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
