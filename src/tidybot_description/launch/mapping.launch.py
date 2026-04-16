"""
Mapping run: brings up Gazebo + home world + robot + EKF + slam_toolbox.

Workflow:
  1. Launch this file:
       ros2 launch tidybot_description mapping.launch.py
  2. In a second terminal, start teleop:
       ros2 run teleop_twist_keyboard teleop_twist_keyboard
     and drive the robot slowly through BOTH rooms. Try to visit every
     corner so slam_toolbox sees all the walls. Stop briefly at each
     new area so scans settle.
  3. When the map in RViz/Gazebo looks complete, save it (third
     terminal):
       ros2 run nav2_map_server map_saver_cli -f ~/DriftAI/maps/home
     That writes maps/home.pgm (image) + maps/home.yaml (metadata).
  4. Kill this launch. Next run will load the saved map via AMCL
     (step 4) instead of mapping from scratch.
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
    ekf_cfg   = os.path.join(pkg, 'config', 'ekf.yaml')
    slam_cfg  = os.path.join(pkg, 'config', 'slam_toolbox.yaml')

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

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_cfg, {'use_sim_time': True}],
    )
    delayed_ekf = TimerAction(period=12.0, actions=[ekf])

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_cfg, {'use_sim_time': True}],
    )
    # Delay slam a bit longer so EKF is already publishing
    # map -> odom ordering: slam_toolbox owns map->odom; EKF owns
    # odom->base_footprint. Both need to be alive before slam starts.
    delayed_slam = TimerAction(period=15.0, actions=[slam])

    return LaunchDescription([
        gzserver, gzclient,
        rsp, jsp,
        delayed_spawn,
        delayed_ekf,
        delayed_slam,
    ])
