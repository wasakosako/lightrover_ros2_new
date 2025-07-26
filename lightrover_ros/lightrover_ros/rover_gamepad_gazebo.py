#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このプログラムは、ゲームパッド（ジョイスティック）からの入力を受け取り、
# ライトローバーの目標速度（Twistメッセージ）に変換してパブリッシュするためのROS2ノードです。
# `joy`ノード（ROS2標準のジョイスティックドライバ）と連携して動作します。

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

# グローバル変数としてTwistメッセージを保持
# コールバック関数内で計算し、メインループでパブリッシュする構成も可能ですが、
# このサンプルではコールバック内で直接パブリッシュしています。
speed = Twist()

class GamePad(Node):
    """
    ゲームパッドの入力を処理し、速度指令をパブリッシュするクラス。
    """
    def __init__(self):
        super().__init__('rover_gamepad')
        # 'rover_twist' トピックへTwistメッセージをパブリッシュするためのPublisherを作成
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # 'joy' トピックからJoyメッセージを購読するためのSubscriberを作成
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback, # コールバック関数を登録
            10
        )
        self.get_logger().info('Gamepad node has been started. Waiting for Joy messages...')

    def joy_callback(self, data):
        """
        `/joy` トピックを受信するたびに呼び出されるコールバック関数。
        アナログスティックの値をロボットの並進速度と角速度にマッピングします。
        """
        global speed

        # data.axesのインデックスは、使用するゲームパッドの種類や設定によって異なります。
        # 一般的な設定では、左スティックの上下が `axes[1]`、
        # 右スティック（または左スティック）の左右が `axes[2]` や `axes[3]` に対応します。

        # axes[1] (左スティック上下) を並進速度 (linear.x) に変換
        # 係数0.1を乗算して、最大速度を0.1 m/sに制限しています。
        speed.linear.x = data.axes[1] * 0.1

        # axes[2] (スティック左右) を旋回速度 (angular.z) に変換
        # 係数2.0を乗算して、旋回速度の感度を調整しています。
        speed.angular.z = data.axes[2] * 2.0

        # 計算した速度指令をパブリッシュ
        self.publisher_.publish(speed)

        # このsleepは必須ではありませんが、メッセージの送信頻度を調整する目的で
        # 入っている可能性があります。ただし、コールバック関数内でのsleepは
        # 他の処理をブロックする可能性があるため、通常は推奨されません。
        # time.sleep(0.05)

def rover_gamepad(args=None):
    """メインの実行関数"""
    rclpy.init(args=args)

    # GamePadノードのインスタンスを作成
    game_pad_node = GamePad()

    # ノードをスピンさせ、コールバック関数が呼ばれるのを待つ
    rclpy.spin(game_pad_node)

    # プログラム終了時にノードを破棄
    game_pad_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    rover_gamepad()