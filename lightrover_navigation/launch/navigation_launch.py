# Copyright (c) 2018 Intel Corporation
# (and others)
#
# このLaunchファイルは、Nav2のナビゲーション機能の中核を担うノード群を起動します。
# これには、経路計画、経路追従（ローカルプランニング）、リカバリー動作、
# それらを統括するビヘイビアツリーなどが含まれます。

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    """LaunchDescriptionを生成します。"""
    bringup_dir = get_package_share_directory('lightrover_navigation')

    # --- Launch設定変数の定義 ---
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # ライフサイクル管理の対象となるノードのリスト
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # トピックのリマッピング設定
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  # Nav2が出力する速度指令'cmd_vel'を、LightRoverのモーター制御ノードが
                  # 購読する'/rover_twist'にリマップする。これがロボットを動かすための重要な接続点。
                  ('/cmd_vel', '/rover_twist')]

    # パラメータファイル内の値を動的に書き換える設定
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # --- Launch引数の宣言 ---
    # (引数の宣言部分は、他のLaunchファイルと同様のためコメントは簡略化)
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', ...)
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='false', ...)
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'), ...)
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', ...)
    declare_use_composition_cmd = DeclareLaunchArgument('use_composition', default_value='False', ...)
    declare_container_name_cmd = DeclareLaunchArgument('container_name', default_value='nav2_container', ...)
    declare_use_respawn_cmd = DeclareLaunchArgument('use_respawn', default_value='False', ...)
    declare_log_level_cmd = DeclareLaunchArgument('log_level', default_value='info', ...)

    # --- ノードの起動設定 ---

    # use_composition が false の場合: 各ノードを個別のプロセスとして起動
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            # Controller Server: ローカルパスプランナー。大域経路に沿って、障害物を回避しながら実際にロボットが従うべき速度指令を生成する。
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn, ...
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            # Smoother Server: 生成された経路を滑らかにする。
            Node(
                package='nav2_smoother', ...
            ),
            # Planner Server: グローバルパスプランナー。地図全体を見て、スタートからゴールまでの大まかな経路を生成する。
            Node(
                package='nav2_planner', ...
            ),
            # Behavior Server: ナビゲーション中の特定の状況（リカバリーなど）で実行される個別の動作（スピン、バックアップなど）を提供する。
            Node(
                package='nav2_behaviors', ...
            ),
            # BT Navigator: ビヘイビアツリーを使って、上記のPlanner, Controller, Behaviorを適切に組み合わせ、ナビゲーションタスク全体を管理する。
            Node(
                package='nav2_bt_navigator', ...
            ),
            # Waypoint Follower: 複数の経由点を順番にたどるナビゲーション機能を提供する。
            Node(
                package='nav2_waypoint_follower', ...
            ),
            # Velocity Smoother: Controllerから出た速度指令を滑らかにし、急な加減速を防ぐ。
            Node(
                package='nav2_velocity_smoother', ...
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            # Lifecycle Manager: 上記の全ノードのライフサイクルを管理する。
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation', ...
                parameters=[..., {'node_names': lifecycle_nodes}]),
        ]
    )

    # use_composition が true の場合: 各ノードをコンポーネントとして単一のコンテナにロード
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            # 上記のNode定義と同様のノードを、プラグインとしてロードする
            ComposableNode(package='nav2_controller', plugin='nav2_controller::ControllerServer', ...),
            ComposableNode(package='nav2_smoother', plugin='nav2_smoother::SmootherServer', ...),
            ComposableNode(package='nav2_planner', plugin='nav2_planner::PlannerServer', ...),
            ComposableNode(package='nav2_behaviors', plugin='behavior_server::BehaviorServer', ...),
            ComposableNode(package='nav2_bt_navigator', plugin='nav2_bt_navigator::BtNavigator', ...),
            ComposableNode(package='nav2_waypoint_follower', plugin='nav2_waypoint_follower::WaypointFollower', ...),
            ComposableNode(package='nav2_velocity_smoother', plugin='nav2_velocity_smoother::VelocitySmoother', ...),
            ComposableNode(package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager', ...),
        ],
    )

    ld = LaunchDescription()

    # 引数とアクションをLaunchDescriptionに追加
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld