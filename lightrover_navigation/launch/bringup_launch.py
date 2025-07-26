# Copyright (c) 2018 Intel Corporation
# (and others)
#
# このLaunchファイルは、ROS2ナビゲーションスタック(Nav2)を起動するためのものです。
# SLAMモードと純粋なナビゲーションモードの両方をサポートしており、
# パラメータによって動作を切り替えることができます。
# LightRover用に設定が調整されています。

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    """Nav2スタックを起動するためのLaunchDescriptionを生成します。"""
    # このパッケージの共有ディレクトリを取得
    bringup_dir = get_package_share_directory('lightrover_navigation')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # --- Launch設定変数の定義 ---
    # LaunchConfigurationは、launchコマンドの引数や他のLaunchファイルからの値を参照します。
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # TFトピックのリマッピング設定
    # 名前空間を使用する場合に、/tf -> <namespace>/tf のようにリマップします。
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # パラメータファイル(YAML)内の一部の値を、Launch引数の値で動的に書き換える設定
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    # RewrittenYamlを使って、上記の設定でパラメータファイルを書き換える
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # --- Launch引数の宣言 ---
    # ここで、このLaunchファイルが受け付ける引数をデフォルト値と共に定義します。

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='false', description='Whether to apply a namespace')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Run SLAM to build a map'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value='', description='Full path to map file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file for Nav2')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the Nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True', description='Use composed bringup for nodes')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False', description='Respawn nodes if they crash')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Log level')

    # --- ノードと他のLaunchファイルの起動設定 ---

    bringup_cmd_group = GroupAction([
        # use_namespaceがtrueの場合、以降のノードをここで指定した名前空間内で起動する
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        # use_compositionがtrueの場合、Nav2の各ノードをコンポーネントとして
        # １つのプロセス（コンテナ）で起動する。効率が良い。
        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        # SLAMモードの場合 (slam:=True)
        # lightrover_slam.launch.py をインクルードしてSLAMノードを起動する
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_sync.launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': params_file}.items()),

        # ナビゲーションモードの場合 (slam:=False)
        # localization_launch.py をインクルードして、地図ベースの自己位置推定(AMCL)を起動する
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        # SLAM/ナビゲーションモード共通
        # navigation_launch.py をインクルードして、経路計画や障害物回避などの
        # ナビゲーション本体のノードを起動する
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])

    # LaunchDescriptionオブジェクトを作成
    ld = LaunchDescription()

    # 宣言した引数をLaunchDescriptionに追加
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 定義したノード群をLaunchDescriptionに追加
    ld.add_action(bringup_cmd_group)

    return ld