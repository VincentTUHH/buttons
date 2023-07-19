from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path


def declare_launch_args(launch_description: LaunchDescription):
    action = DeclareLaunchArgument(name='vehicle_name')
    launch_description.add_action(action)


def generate_launch_description():
    package_name = 'buttons'
    package_path = get_package_share_path(package_name)
    button_config_file = str(package_path / 'config/button_0.yaml')
    buzzer_config_file = str(package_path / 'config/buzzer.yaml')
    battery_config_file = str(package_path / 'config/battery.yaml')

    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    path = str(package_path / 'launch/generic_button.launch.py')
    source = PythonLaunchDescriptionSource(path)
    action = IncludeLaunchDescription(source,
                                      launch_arguments={
                                          'namespace':
                                          'button_0',
                                          'vehicle_name':
                                          LaunchConfiguration('vehicle_name'),
                                          'button_config_file':
                                          button_config_file,
                                          'buzzer_config_file':
                                          buzzer_config_file,
                                          'battery_config_file':
                                          battery_config_file,
                                      }.items())
    launch_description.add_action(action)

    return launch_description
