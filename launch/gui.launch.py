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

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def declare_launch_args(launch_description: LaunchDescription):
    launch_description.add_action(
        DeclareLaunchArgument(name='vehicle_names',
                              default_value='[bluerov01, uuv02]'))


def add_gui_node():
    return Node(
        package='buttons',
        executable='gui_node',
        parameters=[
            {
                'vehicle_names': LaunchConfiguration('vehicle_names')
            },
        ],
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
