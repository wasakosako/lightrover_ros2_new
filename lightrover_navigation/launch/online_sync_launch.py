#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このLaunchファイルは、`slam_toolbox`パッケージのSLAMノードを
# 同期モード(sync)で起動します。
# 同期モードは、すべてのセンサーデータが処理されるのを待ってから
# 地図を更新するため、処理は重くなりますが、より正確な地図が期待できます。

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """LaunchDescriptionを生成します。"""

    # --- Launch設定変数の定義 ---
    # `use_sim_time`: シミュレーション時間を使うかどうか (Gazeboなど)
    use_sim_time = LaunchConfiguration('use_sim_time')
    # `slam_params_file`: SLAMノード用のパラメータファイルのパス
    slam_params_file = LaunchConfiguration('slam_params_file')

    # --- Launch引数の宣言 ---

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        # デフォルトのパラメータファイルのパスを指定
        default_value=os.path.join(get_package_share_directory("lightrover_navigation"),
                                   'config', 'mapper_params_online_sync.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )

    # --- ノードの定義 ---

    # `slam_toolbox`の同期SLAMノードを起動
    start_sync_slam_toolbox_node = Node(
        parameters=[
          slam_params_file, # SLAMのアルゴリズムに関するパラメータ
          {'use_sim_time': use_sim_time} # シミュレーション時間を使用するかどうかの設定
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node', # 同期モードの実行ファイル
        name='slam_toolbox',
        output='screen'
    )

    # LaunchDescriptionオブジェクトを作成
    ld = LaunchDescription()

    # 宣言した引数とノードをLaunchDescriptionに追加
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_sync_slam_toolbox_node)

    return ld