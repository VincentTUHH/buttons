# Copyright (C) 2024 Thies Lennart Alff

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def declare_launch_args(launch_description: LaunchDescription):
    pkg = 'buttons'
    config_file = str(get_package_share_path(pkg) / 'config/buzzer.yaml')
    action = DeclareLaunchArgument(
        name='config_file', default_value=config_file
    )
    launch_description.add_action(action)

    action = DeclareLaunchArgument(name='vehicle_name')
    launch_description.add_action(action)


def create_buzzer_node():
    return Node(
        package='buttons',
        executable='buzzer_interface_node',
        namespace=LaunchConfiguration('vehicle_name'),
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description)
    actions = [
        create_buzzer_node(),
    ]

    for action in actions:
        launch_description.add_action(action)
    return launch_description
