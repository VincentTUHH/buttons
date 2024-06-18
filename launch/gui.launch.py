from hippo_common.launch_helper import config_file_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def declare_launch_args(launch_description: LaunchDescription):
    config_file = config_file_path('buttons', 'gui_config.yaml')
    action = DeclareLaunchArgument(
        name='config_file', default_value=config_file
    )
    launch_description.add_action(action)


def add_gui_node():
    return Node(
        package='buttons',
        executable='gui_node',
        # parameters=[LaunchConfiguration('config_file')],
        output='screen',
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description)
    actions = [
        add_gui_node(),
    ]

    for action in actions:
        launch_description.add_action(action)
    return launch_description
