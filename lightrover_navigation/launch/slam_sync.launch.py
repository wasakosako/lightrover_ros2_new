#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このLaunchファイルは、同期SLAMプロセスを開始するためのものです。
# RVizをSLAM用の設定で起動し、`slam_toolbox`を同期モードで実行する
# 別のLaunchファイル(`online_sync_launch.py`)をインクルードします。

from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """LaunchDescriptionを生成します。""" 

    # --- RVizの起動設定 ---

    # SLAM表示用にカスタマイズされたRViz設定ファイルのパスを取得
    rviz_config_path = get_package_share_path('lightrover_navigation') / 'rviz/slam.rviz'

    # RViz設定ファイルのパスをLaunch引数として宣言
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # RViz2ノードの定義
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')], # 上で定義した設定ファイルを使用
    )

    # --- SLAMノードの起動設定 ---

    # `online_sync_launch.py`をインクルードして、SLAMの本体を起動します。
    # `IncludeLaunchDescription` を使うことで、他のLaunchファイルを再利用できます。
    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lightrover_navigation'), # lightrover_navigationパッケージを探す
                'launch',
                'online_sync_launch.py' # 起動するファイル
            ])
        ]),
        # インクルードするLaunchファイルに渡す引数を設定
        launch_arguments={
            'use_sim_time': 'false', # シミュレーション時間は使用しない
            # ここで他のSLAM関連パラメータ（例: 'slam_params_file'）も渡すことができる
        }.items()
    )

    # `pub_odom`ノード（もし存在するなら）の起動
    # このノードの具体的な役割はソースコードを確認する必要がありますが、
    # オドメトリをパブリッシュする役割を担っていると推測されます。
    # pub_odom_node = Node(
    #     package='lightrover_navigation',
    #     executable='pub_odom',
    #     name='pub_odom'
    # )

    # LaunchDescriptionオブジェクトを返し、起動するノードとアクションをリストとして渡します。
    return LaunchDescription([
        rviz_arg,
        rviz_node,
        launch_slam,
        # pub_odom_node, # 必要に応じてコメントを解除
    ])