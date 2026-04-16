"""
Full navigation stack: Gazebo + home world + robot + EKF + AMCL + Nav2.

Brings up everything needed for autonomous navigation with the
pre-built map in maps/home.yaml.

Terminal 1:
  ros2 launch tidybot_description navigation.launch.py
Terminal 2 (send a goal from RViz):
  rviz2
    Fixed Frame = map, add Map (/map), LaserScan (/scan), RobotModel,
    Path, Particle Cloud (/particlecloud).
    Click "2D Goal Pose" in toolbar, click+drag on map -> robot drives.
Or via CLI:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
    '{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: 1.0},
      orientation: {w: 1.0}}}}'
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('tidybot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'tidybot.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'home.world')
    ekf_cfg   = os.path.join(pkg, 'config', 'ekf.yaml')
    nav2_cfg  = os.path.join(pkg, 'config', 'nav2.yaml')
    rviz_cfg  = os.path.join(pkg, 'config', 'nav.rviz')

    # Map file path lives outside the install share dir, in the workspace
    # maps/ directory. Nav2's map_server takes an absolute path.
    map_file = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..', '..', 'maps', 'home.yaml'))
    # Fallback: if the relative compute above is off in some install
    # layouts, point explicitly at the source-tree maps/.
    if not os.path.exists(map_file):
        map_file = '/home/jueec/DriftAI/maps/home.yaml'

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

    # Nav2 bringup: map_server + AMCL + planner + controller +
    # behavior_server + bt_navigator + velocity_smoother, all wired
    # together. Delayed so EKF is already publishing odom tf.
    nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_cfg,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )
    delayed_nav2 = TimerAction(period=15.0, actions=[nav2])

    # Force-publish /initialpose after Nav2 is active. Humble AMCL's
    # set_initial_pose yaml param is unreliable (doesn't always fire
    # during lifecycle configure), so we seed the filter ourselves
    # from the same spawn pose. A single --once pub is enough; AMCL
    # persists the particles from there.
    seed_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once', '/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            '{header: {frame_id: map}, '
            'pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, '
            'orientation: {w: 1.0}}, '
            'covariance: [0.25, 0, 0, 0, 0, 0, '
            '0, 0.25, 0, 0, 0, 0, '
            '0, 0, 0, 0, 0, 0, '
            '0, 0, 0, 0, 0, 0, '
            '0, 0, 0, 0, 0, 0, '
            '0, 0, 0, 0, 0, 0.06]}}',
        ],
        output='screen',
    )
    delayed_seed = TimerAction(period=22.0, actions=[seed_pose])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    # Start RViz after Nav2 is up so all its subscribed topics exist on
    # launch (avoids the "waiting for..." spinner on every display).
    delayed_rviz = TimerAction(period=18.0, actions=[rviz])

    return LaunchDescription([
        gzserver, gzclient,
        rsp, jsp,
        delayed_spawn,
        delayed_ekf,
        delayed_nav2,
        delayed_seed,
        delayed_rviz,
    ])
