import rclpy
import rcl_interfaces.msg
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.node import Node
from buttons_msgs.msg import Button, BatteryState

from buttons.buzzer import Buzzer


class BuzzerInterfaceNode(Node):

    DEFAULT_PIN = 18

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._ok = False

        # parameters
        self.buzzer_pin: int

        if not self.init_params():
            return

        self.buzzer = Buzzer(self.buzzer_pin)

        self.buzzer.happy(0.2)
        self.battery_low = False

        self.battery_sub = self.create_subscription(BatteryState,
                                                    'battery_state',
                                                    self.on_battery_state, 10)
        self.button_sub = self.create_subscription(Button, 'button',
                                                   self.on_button, 10)

        self.update_timer = self.create_timer(0.1, self.update)
        self.t_last = self.get_clock().now().nanoseconds * 1e-9
        self._ok = True

    def is_okay(self):
        return self._ok

    def init_params(self):
        descriptor = rcl_interfaces.msg.ParameterDescriptor()
        descriptor.name = 'buzzer_gpio'
        descriptor.description = ('GPIO used for controlling the buzzer.')
        descriptor.read_only = True
        descriptor.type = rcl_interfaces.msg.ParameterType.PARAMETER_INTEGER
        try:
            param = self.declare_parameter(descriptor.name,
                                           value=self.DEFAULT_PIN,
                                           descriptor=descriptor)
        except InvalidParameterTypeException as e:
            self.get_logger().fatal(f'{e}')
            return False
        if param.type_ == rclpy.Parameter.Type.NOT_SET:
            self.get_logger().fatal(f'Required parameter {param.name} not set.')
            return False

        self.buzzer_pin = param.value
        self.get_logger().info(f'{param.name}={param.value}')
        return True

    def on_button(self, _):
        # avoid interrupting battery-low-alert
        if not self.battery_low:
            self.buzzer.blip()

    def on_battery_state(self, msg: BatteryState):
        self.battery_low = msg.state == BatteryState.BAD

    def update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.t_last > 2.0:
            self.t_last = now
            self.buzzer.low_pitch(0.5)

    def on_shutdown(self):
        self.buzzer.sad(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = BuzzerInterfaceNode('sound')
    if not node.is_okay():
        node.destroy_node()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
