#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # パッケージとディレクトリ
    lightrover_description_dir = get_package_share_directory('lightrover_description')
    launch_dir = os.path.join(lightrover_description_dir, 'launch')

    # Launch引数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RVIZ')

    # Gazebo起動
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': os.path.join(
            lightrover_description_dir, 'worlds', 'lightrover_sample.world')}.items()
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))
    )

    # robot_description（xacroやurdfを読み込んで文字列化）
    urdf_path = os.path.join(lightrover_description_dir, 'urdf', 'lightrover_urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}
    use_sim_time_param = {'use_sim_time': use_sim_time}

    # robot_state_publisher 起動
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, use_sim_time_param],
        output='screen'
    )

    # ros2_control_node 起動（URDF + YAML）
    # control_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         robot_description,
    #         os.path.join(lightrover_description_dir, 'config', 'lightrover_controllers.yaml')
    #     ],
    #     output='screen'
    # )

    # ロボットのスポーン
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'lightrover',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # コントローラーのスポーン
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # RViz起動（オプション）
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(lightrover_description_dir, 'rviz', 'lightrover.rviz')],
        output='screen')

    # LaunchDescription構築
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(control_node)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(spawn_diff_drive_controller)
    ld.add_action(start_rviz_cmd)

    return ld