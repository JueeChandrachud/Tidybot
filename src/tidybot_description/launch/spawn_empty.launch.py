"""
Launch TidyBot in an empty Gazebo world. For testing the robot build

WSL-specific workarounds:
  - ALSOFT_DRIVERS=null disables OpenAL's ALSA probe (which hangs for
    ~30 s on WSL with no audio device — long enough that spawn_entity.py
    times out before gzserver finishes booting).
  - spawn_entity is delayed 8 s via TimerAction to give gzserver time to
    register /spawn_entity even on a cold first-run Gazebo startup.

Plugins:
  - -s libgazebo_ros_init.so    -> /clock, sim time
  - -s libgazebo_ros_factory.so -> /spawn_entity service
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
    world_file = os.path.join(pkg, 'worlds', 'empty.world')

    robot_description = xacro.process_file(xacro_file).toxml()

    gz_env = {
        'ALSOFT_DRIVERS': 'null',   # skip ALSA probe on WSL
        'SDL_AUDIODRIVER': 'dummy',
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
            '-x', '0', '-y', '0', '-z', '0.05',
        ],
        output='screen',
    )

    # Wait for gzserver to finish booting and register /spawn_entity.
    delayed_spawn = TimerAction(period=8.0, actions=[spawn])

    return LaunchDescription([gzserver, gzclient, rsp, jsp, delayed_spawn])
