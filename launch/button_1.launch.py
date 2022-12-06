import launch
import launch_ros
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    package_name = 'buttons'
    package_path = get_package_share_path(package_name)
    default_vehicle_name = 'uuv00'
    default_button_config_file = package_path / 'config/button_1.yaml'
    default_buzzer_config_file = package_path / 'config/buzzer.yaml'
    default_battery_config_file = package_path / 'config/battery.yaml'

    vehicle_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='vehicle_name',
        default_value=default_vehicle_name,
        description='Vehicle name used as namespace.')

    button_config_launch_arg = launch.actions.DeclareLaunchArgument(
        name='button_config_file',
        default_value=str(default_button_config_file),
        description='Path to the button config file that defines used GPIOs.')
    buzzer_config_launch_arg = launch.actions.DeclareLaunchArgument(
        name='buzzer_config_file',
        default_value=str(default_buzzer_config_file),
        description='Path to the buzzer config file that defines the used GPIO.'
    )
    battery_config_launch_arg = launch.actions.DeclareLaunchArgument(
        name='battery_config_file',
        default_value=str(default_battery_config_file),
        description=('Path to the battery config file that defines the number '
                     'of cells and voltage level thresholds.'))

    arming_service = [
        '/',
        launch.substitutions.LaunchConfiguration('vehicle_name'),
        '/arm',
    ]

    battery_voltage_topic = [
        '/',
        launch.substitutions.LaunchConfiguration('vehicle_name'),
        '/battery_voltage',
    ]

    namespace_group = launch.actions.GroupAction([
        launch_ros.actions.PushRosNamespace('button_1'),
        launch_ros.actions.Node(
            package=package_name,
            executable='button_interface',
            parameters=[
                launch.substitutions.LaunchConfiguration('button_config_file')
            ],
            output='screen'),
        launch_ros.actions.Node(
            package=package_name,
            executable='buzzer_interface',
            parameters=[
                launch.substitutions.LaunchConfiguration('buzzer_config_file')
            ],
            output='screen'),
        launch_ros.actions.Node(package=package_name,
                                executable='led_interface',
                                output='screen'),
        launch_ros.actions.Node(package=package_name,
                                executable='button_handler',
                                remappings=[
                                    ('/arm', arming_service),
                                ],
                                output='screen',
                                emulate_tty=True),
        launch_ros.actions.Node(
            package=package_name,
            executable='battery_watcher',
            parameters=[
                launch.substitutions.LaunchConfiguration('battery_config_file'),
            ],
            remappings=[
                ('/battery_voltage', battery_voltage_topic),
            ],
            output='screen',
            emulate_tty=True),
    ])

    launch_description = [
        vehicle_name_launch_arg,
        button_config_launch_arg,
        buzzer_config_launch_arg,
        battery_config_launch_arg,
        namespace_group,
    ]

    return launch.LaunchDescription(launch_description)
