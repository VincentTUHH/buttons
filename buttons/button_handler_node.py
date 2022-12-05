import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from buttons_msgs.msg import Button
from std_srvs.srv import SetBool


class ButtonHandlerNode(Node):
    ARM_BUTTON = 0
    DISARM_BUTTON = 0

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._ok = False

        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.message_cb_group = MutuallyExclusiveCallbackGroup()

        self.arming_client = self.create_client(
            SetBool, '/arm', callback_group=self.service_cb_group)

        self.button_sub = self.create_subscription(
            Button,
            'button',
            self.on_button,
            10,
            callback_group=self.message_cb_group)

        self._ok = True

    def arm_vehicle(self, state: bool):
        request = SetBool.Request()
        request.data = state
        if not self.arming_client.wait_for_service(timeout_sec=0.5):
            srv_name = self.resolve_service_name(self.arming_client.srv_name)
            self.get_logger().error(f'Service [{srv_name}] not available.')
        response: SetBool.Response = self.arming_client.call(request)
        if response.success:
            self.get_logger().info(
                f'Successfully called arming service: {response.message}')
        else:
            self.get_logger().warn(
                f'Failed to call arming service: {response.message}')

    def on_button(self, msg: Button):
        if msg.button == self.ARM_BUTTON:
            self.arm_vehicle(True)
        elif msg.button == self.DISARM_BUTTON:
            self.arm_vehicle(False)
        else:
            self.get_logger().info(f'Unhandled button pressed: {msg.button}')

    def is_okay(self):
        return self._ok


def main(args=None):
    rclpy.init(args=args)
    node = ButtonHandlerNode('button_handler')
    if not node.is_okay():
        node.destroy_node()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
