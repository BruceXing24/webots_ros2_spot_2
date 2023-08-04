import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK


class MoveitIKClientAsync(Node):
    def __init__(self):
        super().__init__('moveit_ik')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.on_timer)
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)

        self.cli = self.create_client(GetPositionIK, '/compute_ik')

        self.target = 'Image1'
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            rclpy.spin_once(self)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                self.target,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1))

            self.tf2_bl_target = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,
                                  t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to self.target: {ex}')
            return

    def joint_states_cb(self, joint_state):
        self.joint_state = joint_state

    def moveit_ik(self):
        self.req.ik_request.group_name = 'spot_arm'
        self.req.ik_request.robot_state.joint_state = self.joint_state
        self.req.ik_request.avoid_collisions = True

        self.req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.req.ik_request.pose_stamped.header.frame_id = 'base_link'

        self.req.ik_request.pose_stamped.pose.position.x = self.tf2_bl_target[0]
        self.req.ik_request.pose_stamped.pose.position.y = self.tf2_bl_target[1]
        self.req.ik_request.pose_stamped.pose.position.z = self.tf2_bl_target[2] + 0.03

        if self.target == 'PlaceBox':
            self.req.ik_request.pose_stamped.pose.position.z+=0.58

        # self.req.ik_request.pose_stamped.pose.orientation.x = self.tf2_bl_target[3]
        # self.req.ik_request.pose_stamped.pose.orientation.y = self.tf2_bl_target[4]
        # self.req.ik_request.pose_stamped.pose.orientation.z = self.tf2_bl_target[5]
        # self.req.ik_request.pose_stamped.pose.orientation.w = self.tf2_bl_target[6]

        self.req.ik_request.timeout.sec = 10

        return self.req.ik_request

    def send_request(self, target):
        self.req = GetPositionIK.Request()
        self.target = target
        self.joint_state = None
        self.tf2_bl_target = None

        while self.joint_state is None or self.tf2_bl_target is None:
            rclpy.spin_once(self)

        self.req.ik_request = self.moveit_ik()

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        target_angles = list(self.future.result().solution.joint_state.position)[:7]

        if len(target_angles) > 0:
            print('solution available')
        else:
            print('no solution')




        return target_angles if len(target_angles) > 0 else [0., 1.55, 1.74, 0., 0.075, 1.03, 0.]
