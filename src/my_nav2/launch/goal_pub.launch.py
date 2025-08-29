# ~/iki_workspace/stage_project/src/my_nav2/launch/goal_pub.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # --- Hard paths (inside container) ---
    gazebo_launch = '/opt/ros/humble/share/turtlebot3_gazebo/launch/turtlebot3_dqn_stage4.launch.py'
    nav2_launch   = '/opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py'
    map_yaml      = '/root/iki_workspace/stage_project/maps/myfirstmap.yaml'
    params_yaml   = '/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml'
    rviz_config   = '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'

    # --- Avoid DDS conflicts ---
    env_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '7')
    env_rmw    = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    # --- Gazebo (Stage 4 world) ---
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch))

    # --- Nav2 bringup (no RViz here; we start RViz explicitly below) ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_yaml,
            'params_file': params_yaml,
            'autostart': 'true',
            'use_rviz': 'false'
        }.items()
    )

    # --- RViz explicitly ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # --- Auto initial pose (publishes /initialpose) ---
    init_pose = Node(
        package='my_nav2',
        executable='init_node',   # your file is init_node.py
        name='init_node',
        output='screen'
    )
    init_pose_delayed = TimerAction(period=5.0, actions=[init_pose])  # wait for Nav2 to be ready

    # --- Goal publisher ---
    goal_pub = Node(
        package='my_nav2',
        executable='goal_pub',
        name='goal_pub',
        output='screen'
    )

    return LaunchDescription([
        env_domain, env_rmw,
        gazebo,
        nav2,
        rviz,
        init_pose_delayed,
        goal_pub
    ])

