#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このLaunchファイルは、LightRoverの3DモデルをRViz2で表示するためのものです。
# URDFモデルを読み込み、ロボットの状態をパブリッシュし、RViz2を起動します。

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """LaunchDescriptionを生成します。"""

    # lightrover_description パッケージの共有ディレクトリへのパスを取得
    description_package_path = get_package_share_path('lightrover_description')
    # デフォルトのURDF/Xacroモデルファイルへのパス
    default_model_path = description_package_path / 'urdf/lightrover_urdf.xacro'
    # デフォルトのRViz設定ファイルへのパス
    default_rviz_config_path = description_package_path / 'rviz/lightrover.rviz'

    # --- Launch引数の宣言 ---

    # 'gui'引数: joint_state_publisher_gui を使うかどうかを決定するフラグ
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    # 'model'引数: 使用するロボットモデルファイルのパス
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    # 'rvizconfig'引数: 使用するRViz設定ファイルのパス
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    # --- ノードの定義 ---

    # robot_description パラメータの値を生成
    # 'xacro' コマンドを実行して、Xacroファイルから完全なURDFを生成します。
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # robot_state_publisher ノード
    # 'robot_description' パラメータを購読し、URDFに基づいて
    # ロボットの各リンク間のTF（座標変換）情報をブロードキャストします。
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # joint_state_publisher ノード (GUIなし版)
    # 'gui'引数が'false'の場合にのみ起動されます (UnlessCondition)。
    # /joint_states トピックにデフォルトのジョイント値をパブリッシュします。
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # joint_state_publisher_gui ノード (GUIあり版)
    # 'gui'引数が'true'の場合にのみ起動されます (IfCondition)。
    # GUIウィンドウを表示し、スライダーで各ジョイントの値を手動で変更し、
    # /joint_states トピックにパブリッシュできます。
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # rviz2 ノード
    # 指定された設定ファイル（.rviz）でRViz2を起動します。
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # LaunchDescriptionオブジェクトを返し、起動する引数とノードをリストとして渡します。
    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
