import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    servos = launch_ros.actions.Node(
        package='beagle_io', node_executable='bleagle_io_servos', output='screen')

    imu = launch_ros.actions.Node(
        package='beagle_io', node_executable='beagle_io_imu', output='screen')

    return launch.LaunchDescription([
        imu,
        servos,
    ])


