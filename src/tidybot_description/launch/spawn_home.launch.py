"""
Launch TidyBot in the home world with diff-drive control active.

Uses the Gazebo-native libgazebo_ros_diff_drive plugin (declared in
urdf/ros2_control.xacro) for wheel control + odometry. No external
controller_manager or spawner nodes required.

Sequence:
  1. gzserver + gzclient   (home.world, factory + init plugins)
  2. robot_state_publisher (TF from URDF)
  3. joint_state_publisher (wheel joint state; Gazebo publishes its
                            own joint_states for simulated joints,
                            jsp here covers fixed/unsimulated ones)
  4. spawn_entity.py       (insert robot, delayed 8 s for /spawn_entity
                            service registration on WSL)

Once spawned, /cmd_vel drives the wheels and /odom is continuously
published by the diff-drive plugin at 50 Hz.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('tidybot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'tidybot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'home.world')

    robot_description = xacro.process_file(xacro_file).toxml()

    plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    plugin_path = '/opt/ros/humble/lib' + (os.pathsep + plugin_path if plugin_path else '')

    gz_env = {
        'ALSOFT_DRIVERS': 'null',
        'SDL_AUDIODRIVER': 'dummy',
        'GAZEBO_PLUGIN_PATH': plugin_path,
    }

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file,
        ],
        output='screen',
        additional_env=gz_env,
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        additional_env=gz_env,
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'tidybot',
            '-x', '-3.0', '-y', '0.0', '-z', '0.10',
            '-Y', '0.0',
        ],
        output='screen',
    )
    delayed_spawn = TimerAction(period=8.0, actions=[spawn])

    # EKF fusing /odom (wheel) + /imu -> /odometry/filtered + tf.
    # Started with a small delay so /odom and /imu are already flowing
    # by the time the filter subscribes (keeps the covariance estimate
    # from initializing on stale/empty data).
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ],
    )
    delayed_ekf = TimerAction(period=12.0, actions=[ekf])

    return LaunchDescription([
        gzserver, gzclient,
        rsp, jsp,
        delayed_spawn,
        delayed_ekf,
    ])
