from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace


def declare_launch_args(launch_description: LaunchDescription):
    action = DeclareLaunchArgument(name='vehicle_name')
    launch_description.add_action(action)

    action = DeclareLaunchArgument(name='button_config_file')
    launch_description.add_action(action)

    action = DeclareLaunchArgument(name='buzzer_config_file')
    launch_description.add_action(action)

    action = DeclareLaunchArgument(name='battery_config_file')
    launch_description.add_action(action)


def arming_service_config():
    return ['/', LaunchConfiguration('vehicle_name'), '/arm']


def battery_voltage_topic():
    return ['/', LaunchConfiguration('vehicle_name'), '/battery_voltage']


def arming_state_topic():
    return ['/', LaunchConfiguration('vehicle_name'), '/arming_state']


def add_nodes(launch_description: LaunchDescription):
    package_name = 'buttons'
    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        Node(package=package_name,
             executable='buzzer_interface_node',
             parameters=[LaunchConfiguration('buzzer_config_file')],
             output='screen'),
        Node(package=package_name,
             executable='button_interface_node',
             parameters=[LaunchConfiguration('button_config_file')],
             output='screen'),
        Node(package=package_name,
             executable='led_interface_node',
             remappings=[
                 ('/arming_state', arming_state_topic()),
                 ('/battery_voltage', battery_voltage_topic()),
             ],
             output='screen'),
        Node(package=package_name,
             executable='button_handler_node',
             remappings=[
                 ('/arm', arming_service_config()),
             ],
             output='screen'),
        Node(package=package_name,
             executable='battery_watcher_node',
             parameters=[LaunchConfiguration('battery_config_file')],
             remappings=[
                 ('/battery_voltage', battery_voltage_topic()),
             ],
             output='screen')
    ])
    launch_description.add_action(group)


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    add_nodes(launch_description=launch_description)
    return launch_description
