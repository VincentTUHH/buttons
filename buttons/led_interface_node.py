import time

import rcl_interfaces.msg
import rclpy
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.node import Node
from std_msgs.msg import Bool

from buttons.common import SingleData
from buttons.led import Strip
from buttons_msgs.msg import BatteryState


class LedInterfaceNode(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._ok = False

        self.battery_timeout = False
        self.arming_timeout = False

        self.battery_state = SingleData(BatteryState.UNAVAILABLE)
        self.armed_state = SingleData(value=False)

        self.battery_timeout = 5.0
        self.arming_timeout = 5.0

        self.strip = Strip()

        # self.vehicle_namespace: str = ''
        # if not self.init_params():
        #     return

        self.battery_sub = self.create_subscription(BatteryState,
                                                    'battery_state',
                                                    self.on_battery_state, 1)
        self.arming_state_sub = self.create_subscription(
            Bool, '/arming_state', self.on_arming_state, 1)

        update_frequency = 5.0
        self.update_timer = self.create_timer(1 / update_frequency, self.update)
        self._ok = True

    def init_params(self):
        descriptor = rcl_interfaces.msg.ParameterDescriptor()
        descriptor.name = 'vehicle_namespace'
        descriptor.description = (
            'Name of the vehicle\'s namespace this node is associated with.')
        descriptor.read_only = True
        descriptor.type = rcl_interfaces.msg.ParameterType.PARAMETER_STRING
        try:
            param = self.declare_parameter(descriptor.name,
                                           descriptor=descriptor)
        except InvalidParameterTypeException as e:
            self.get_logger().fatal(f'{e}')
            return False
        if param.type_ == rclpy.Parameter.Type.NOT_SET:
            self.get_logger().fatal(f'Required parameter {param.name} not set.')
            return False

        self.vehicle_namespace = param.value
        self.get_logger().info(f'{param.name}={param.value}')
        return True

    def is_okay(self):
        return self._ok

    def on_battery_state(self, msg: BatteryState):
        self.battery_state.value = msg.state
        self.battery_state.stamp = self.get_clock().now().nanoseconds * 1e-9
        self.battery_state.updated = True

    def on_arming_state(self, msg: Bool):
        self.armed_state.value = msg.data
        self.armed_state.stamp = self.get_clock().now().nanoseconds * 1e-9
        self.armed_state.updated = True

    def update_battery(self):
        if self.battery_state.value == BatteryState.GOOD:
            self.strip.set_battery_good()
        elif self.battery_state.value == BatteryState.MEDIUM:
            self.strip.set_battery_medium()
        elif self.battery_state.value == BatteryState.BAD:
            self.strip.set_battery_low()
        elif self.battery_state.value == BatteryState.UNSET:
            self.strip.set_battery_undefined()
        elif self.battery_state.value == BatteryState.UNAVAILABLE:
            self.strip.set_battery_undefined()
            self.get_logger().warning('BatteryState is UNAVAILABLE.')
        else:
            self.get_logger().error(
                f'Unhandled BatteryState: {self.battery_state.value}')

    def update(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        any_timed_out = False

        if self.battery_state.updated:
            if self.battery_timeout:
                self.get_logger().info('BatteryState is not timed out anymore.')
            self.battery_timeout = False
            self.battery_state.updated = False
            self.update_battery()
        else:
            if now - self.battery_state.stamp > self.battery_timeout:
                any_timed_out = True
                if not self.battery_timeout:
                    self.get_logger().warning('BatteryState timed out.',
                                              throttle_duration_sec=2.0)
                self.strip.set_battery_undefined()
                self.battery_timeout = True

        if self.armed_state.updated:
            if self.arming_timeout:
                self.get_logger().info('Arming State is not timed out anymore')
            self.arming_timeout = False
            self.armed_state.updated = False
            self.strip.set_arming(now, self.armed_state.value)
        elif now - self.armed_state.stamp > self.arming_timeout:
            any_timed_out = True
            if not self.arming_timeout:
                self.get_logger().warning('Arming state timed out.',
                                          throttle_duration_sec=2.0)
            self.strip.set_arming_state_undefined()
            self.arming_timeout = True

        self.strip.set_status(not any_timed_out)

    def on_shutdown(self):
        self.strip.switch_off()
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = LedInterfaceNode('led')
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
