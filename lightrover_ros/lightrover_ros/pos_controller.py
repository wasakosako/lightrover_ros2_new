#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このプログラムは、目標速度に基づいてライトローバーのモーターを
# 位置制御（実質的には速度制御）するためのROS2ノードです。
# `/rover_twist` トピックで目標速度を受け取り、
# `/odom` トピックから現在の速度をフィードバックとして使用し、
# I2Cサービスを介してモータードライバに指令値を送信します。

import rclpy
from rclpy.node import Node
import sys
from lightrover_interface.srv import Wrc201Msg
import time
import math
import lightrover_ros.vs_wrc201_motor as vs_wrc201_motor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# --- 定数定義 (VS-WRC201 メモリマップアドレス) ---
# これらはモータードライバの特定の機能を設定・制御するためのアドレスです。
MU8_O_EN = 0x10      # 出力有効化フラグ
MU8_TRIG = 0x11      # トリガ設定
MS16_FB_PG0 = 0x20   # フィードバック制御ゲイン (Pゲイン) Ch.0
MS16_FB_PG1 = 0x22   # フィードバック制御ゲイン (Pゲイン) Ch.1

MS32_A_POS0 = 0x48   # 目標位置/速度 Ch.0
MS32_A_POS1 = 0x4c   # 目標位置/速度 Ch.1

MS16_T_OUT0 = 0x50   # トルク指令値 Ch.0
MS16_T_OUT1 = 0x52   # トルク指令値 Ch.1

MU16_FB_PCH0 = 0x30  # フィードバック制御周期 Ch.0
MU16_FB_PCH1 = 0x32  # フィードバック制御周期 Ch.1

# --- グローバル変数 ---
# 現在のロボットの速度 (オドメトリから取得)
current_linear_x = 0.0
current_angular_z = 0.0

# 左右の車輪の現在の速度 [右, 左] (m/s)
current_wheel_v = [0.0, 0.0]
# 左右の車輪の目標速度 [右, 左] (m/s)
target_wheel_v = [0.0, 0.0]

# 車輪間距離の半分 (m)
ROVER_D = 0.143 / 2.0

# モーター制御ロジック（PIDなど）をカプセル化したクラスのインスタンス
motor_controller = vs_wrc201_motor.VsWrc201Motor()

class DriveMotor(Node):
    """
    モーターを駆動するためのメインクラス。
    トピックの購読とI2Cサービスへのリクエストを管理します。
    """
    def __init__(self):
        super().__init__('pos_controller')
        # I2C通信サービスのクライアントを作成
        self.i2c_client = self.create_client(Wrc201Msg, 'wrc201_i2c')
        while not self.i2c_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('I2C service not available, waiting again...')
        self.req = Wrc201Msg.Request()

        # オドメトリ情報を購読するためのSubscriberを作成
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,  # コールバック関数を登録
            10
        )

        # 目標速度を購読するためのSubscriberを作成
        self.drive_subscriber = self.create_subscription(
            Twist,
            'rover_twist',  # ゲームパッドやNav2から配信される
            self.target_velocity_callback,  # コールバック関数を登録
            10
        )

    def odom_callback(self, msg):
        """
        `/odom` トピックを受信するたびに呼び出されるコールバック関数。
        現在のロボットの速度を取得し、モーターへの指令値を計算・送信します。
        この関数が実質的な制御ループの中心です。
        """
        global current_linear_x, current_angular_z, current_wheel_v, ROVER_D, target_wheel_v

        # オドメトリメッセージから現在の並進・角速度を取得
        current_linear_x = msg.twist.twist.linear.x
        current_angular_z = msg.twist.twist.angular.z

        # ロボットの速度から左右の車輪の速度を計算（逆運動学）
        current_wheel_v[1] = (current_linear_x + ROVER_D * current_angular_z)  # 左輪
        current_wheel_v[0] = -1.0 * (current_linear_x - ROVER_D * current_angular_z) # 右輪

        # 制御ロジックを呼び出し、モーターへの出力値（指令値）を計算
        output = motor_controller.pos_controll(current_wheel_v, target_wheel_v)

        # 計算された指令値をモータードライバに送信
        self.drive_motor(output[0], output[1])

    def target_velocity_callback(self, msg):
        """
        `/rover_twist` トピックを受信するたびに呼び出されるコールバック関数。
        目標速度をグローバル変数に保存します。
        """
        global ROVER_D, target_wheel_v

        # 目標の並進・角速度から、左右の車輪の目標速度を計算（逆運動学）
        target_wheel_v[1] = (msg.linear.x + ROVER_D * msg.angular.z)  # 左輪
        target_wheel_v[0] = -1.0 * (msg.linear.x - ROVER_D * msg.angular.z) # 右輪

    def set_request(self, addr, data, length, cmd):
        """I2Cサービスへのリクエストメッセージを簡単に作成するためのヘルパー関数"""
        self.req.addr = addr
        self.req.data = data
        self.req.length = length
        self.req.cmd = cmd
        return self.req

    def drive_motor(self, r_speed, l_speed):
        """計算された速度指令値をI2Cサービス経由でモータードライバに送信します。"""
        # 右輪の目標速度を書き込み
        self.i2c_client.call_async(self.set_request(MS32_A_POS0, r_speed, 4, 'w'))
        # 左輪の目標速度を書き込み
        self.i2c_client.call_async(self.set_request(MS32_A_POS1, l_speed, 4, 'w'))
        # 書き込んだ目標値をモータードライバに反映させるためのトリガを送信
        self.i2c_client.call_async(self.set_request(MU8_TRIG, 0x03, 1, 'w'))

def pos_cntrl(args=None):
    """メインの実行関数"""
    rclpy.init(args=args)

    pos_controller_node = DriveMotor()
    pos_controller_node.get_logger().info('Position controller node has been started.')

    # --- モータードライバの初期設定 ---
    # ここでは、I2Cサービスを直接呼び出して、モーター制御に必要な
    # パラメータ（Pゲインなど）をマイコンに書き込んでいます。
    # これらはノード起動時に一度だけ実行されます。
    pos_controller_node.i2c_client.call_async(pos_controller_node.set_request(MU8_O_EN, 0x00, 1, 'w'))
    pos_controller_node.i2c_client.call_async(pos_controller_node.set_request(MU8_TRIG, 0x0c, 1, 'w'))
    pos_controller_node.i2c_client.call_async(pos_controller_node.set_request(MS16_FB_PG0, 0x0080, 2, 'w'))
    pos_controller_node.i2c_client.call_async(pos_controller_node.set_request(MS16_FB_PG1, 0x0080, 2, 'w'))
    pos_controller_node.i2c_client.call_async(pos_controller_node.set_request(MU16_FB_PCH0, 0x09C4, 2, 'w'))
    pos_controller_node.i2c_client.call_async(pos_controller_node.set_request(MU16_FB_PCH1, 0x09C4, 2, 'w'))
    # 最後にモーター出力を有効化
    pos_controller_node.i2c_client.call_async(pos_controller_node.set_request(MU8_O_EN, 0x03, 1, 'w'))

    # ノードをスピンさせ、コールバック関数が呼ばれるのを待つ
    rclpy.spin(pos_controller_node)

if __name__ == '__main__':
    pos_cntrl()