#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このプログラムは、モーターエンコーダーの値を元に、ライトローバーのオドメトリ
# （自己位置と速度）を計算し、ROS2トピックとして配信するためのノードです。

import rclpy
from rclpy.node import Node
import sys
from lightrover_interface.srv import Wrc201Msg
import time
import math
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
from nav_msgs.msg import Odometry

# --- 定数定義 ---

# VS-WRC201のモーターエンコーダ値が格納されているメモリマップアドレス
MS32_M_POS0 = 0x60  # モーター0 (右輪に対応することが多い)
MS32_M_POS1 = 0x64  # モーター1 (左輪に対応することが多い)

# エンコーダカウンタのオーバーフローを検出するための閾値
# カウンタは32bitですが、急激な変化はノイズやエラーの可能性があるため、
# 妥当な範囲内での差分のみを計算対象とします。
DIFF_COUNT_LIMIT = 1048575

# 車輪の物理的パラメータ
WHEEL_DIAMETER = 60.0 / 1000  # 車輪の直径 (m)
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi  # 車輪の円周 (m)

# エンコーダの性能パラメータ
ENC_COUNTS_PER_TURN = 1188.024  # モーター1回転あたりのエンコーダカウント数
ENC_PER_M = ENC_COUNTS_PER_TURN / WHEEL_CIRCUMFERENCE  # 1mあたりのエンコーダカウント数

# 車体の物理的パラメータ
ROVER_TREAD = 0.143  # 車輪間の距離 (トレッド) (m)
ROVER_D = ROVER_TREAD / 2.0  # 車輪間距離の半分

# --- グローバル変数 ---
# 本来はクラスのメンバ変数として管理するのが望ましいですが、
# このサンプルではグローバル変数として定義されています。

# 1つ前のループで取得したエンコーダの生カウント値
pre_count = [0.0, 0.0]
# 1つ前のループとのエンコーダカウントの差分
diff_count = [0, 0]

# 時間計測用の変数
pre_time = time.time()
diff_time = 0

# 計算されたロボットの自己位置 (x, y) と姿勢 (th)
x = 0.0
y = 0.0
th = 0.0

class OdometryManager(Node):
    """
    オドメトリ計算のメインクラス。
    サービスへのリクエスト、トピックへのパブリッシュ、TFのブロードキャストを管理します。
    """
    def __init__(self):
        super().__init__('wrc201_odometry')
        # I2C通信サービス(`wrc201_i2c`)のクライアントを作成
        self.read_enc_client = self.create_client(Wrc201Msg, 'wrc201_i2c')
        # サービスの準備ができるまで待機
        while not self.read_enc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('I2C service not available, waiting again...')
        self.req = Wrc201Msg.Request()
        # オドメトリ情報をパブリッシュするためのPublisherを作成
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

    def get_encoder_values(self):
        """I2Cサービスを非同期に呼び出し、左右のエンコーダ値を取得します。"""
        # 右輪のエンコーダ値を取得するリクエスト
        self.req.addr = MS32_M_POS0
        self.req.data = 0
        self.req.length = 4
        self.req.cmd = 'r'
        self.future_enc_a = self.read_enc_client.call_async(self.req)

        # 左輪のエンコーダ値を取得するリクエスト
        # Note: 同じリクエストオブジェクトを使いまわしているため、addrの変更だけでOK
        self.req.addr = MS32_M_POS1
        self.future_enc_b = self.read_enc_client.call_async(self.req)

def calculate_speed(enc_val):
    """
    エンコーダの値からロボットの並進速度と角速度を計算します。
    差動二輪モデルの運動学に基づいています。
    """
    global diff_time, pre_time, diff_count, pre_count

    # 前回からの経過時間を計算
    now_time = time.time()
    diff_time = now_time - pre_time
    pre_time = now_time

    if enc_val is None:
        return None

    # 前回からのエンコーダカウントの差分を計算
    for i in range(2):
        # オーバーフローを考慮し、差分が極端に大きくない場合のみ更新
        if abs(enc_val[i] - pre_count[i]) < DIFF_COUNT_LIMIT:
            # モーターの回転方向と座標系の関係から-1.0を乗算
            diff_count[i] = -1.0 * (enc_val[i] - pre_count[i])

    pre_count = enc_val

    # 各車輪の移動距離 (m) を計算
    distance = [float(diff_count[0]) / ENC_PER_M, float(diff_count[1]) / ENC_PER_M]
    # 各車輪の速度 (m/s) を計算
    speed = [distance[0] / diff_time, distance[1] / diff_time]

    # ロボット本体の並進速度 (linear_x) と旋回速度 (angular_z) を計算
    # linear_x = (右輪速度 + 左輪速度) / 2  (このコードでは左右が逆)
    # angular_z = (右輪速度 - 左輪速度) / トレッド (このコードでは左右が逆)
    linear_x = (speed[0] - speed[1]) / 2.0
    angular_z = -1.0 * (speed[0] + speed[1]) / (2.0 * ROVER_D)

    return linear_x, angular_z

def calculate_odometry_pose(vx, vth):
    """速度を積分して、ロボットの自己位置(x, y, th)を更新します。"""
    global x, y, th, diff_time

    # 経過時間(diff_time)と速度(vx, vth)を使って、各成分の変化量を計算
    # シンプルなオイラー積分
    delta_x = vx * math.cos(th) * diff_time
    delta_y = vx * math.sin(th) * diff_time
    delta_th = vth * diff_time

    # 自己位置を更新
    x += delta_x
    y += delta_y
    th += delta_th

def lightrover_odometry(args=None):
    """メインの実行関数"""
    rclpy.init(args=args)

    odom_manager = OdometryManager()
    odom_manager.get_logger().info('Odometry node has been started.')

    # TFフレームをブロードキャストするためのTransformBroadcasterを初期化
    odom_br = TransformBroadcaster(odom_manager)

    # 最初のエンコーダ値を取得開始
    odom_manager.get_encoder_values()

    while rclpy.ok():
        # コールバック関数などを処理
        rclpy.spin_once(odom_manager)

        velocities = None
        # 両方のエンコーダ値の取得が完了したかチェック
        if odom_manager.future_enc_a.done() and odom_manager.future_enc_b.done():
            # サービスの結果を取得
            enc_a_result = odom_manager.future_enc_a.result().read_data
            enc_b_result = odom_manager.future_enc_b.result().read_data
            # 速度を計算
            velocities = calculate_speed([enc_a_result, enc_b_result])

        if velocities is None:
            # 速度が計算できなかった場合はループの最初に戻る
            continue
        else:
            # 1. 自己位置の更新
            calculate_odometry_pose(velocities[0], velocities[1])

            now_time = odom_manager.get_clock().now().to_msg()

            # 2. TF (odom -> base_footprint) の作成とブロードキャスト
            odom_tf = TransformStamped()
            odom_tf.header.stamp = now_time
            odom_tf.header.frame_id = 'odom'
            odom_tf.child_frame_id = 'base_footprint'

            # 姿勢(th)をオイラー角からクォータニオンに変換
            odom_quat = tf_transformations.quaternion_from_euler(0, 0, th)

            odom_tf.transform.translation.x = x
            odom_tf.transform.translation.y = y
            odom_tf.transform.translation.z = 0.0
            odom_tf.transform.rotation.x = odom_quat[0]
            odom_tf.transform.rotation.y = odom_quat[1]
            odom_tf.transform.rotation.z = odom_quat[2]
            odom_tf.transform.rotation.w = odom_quat[3]

            odom_br.sendTransform(odom_tf)

            # 3. Odometryメッセージの作成とパブリッシュ
            odom = Odometry()
            odom.header.stamp = now_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"

            # Pose (位置と姿勢) を設定
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = odom_quat[0]
            odom.pose.pose.orientation.y = odom_quat[1]
            odom.pose.pose.orientation.z = odom_quat[2]
            odom.pose.pose.orientation.w = odom_quat[3]

            # Twist (速度) を設定
            odom.twist.twist.linear.x = velocities[0]
            odom.twist.twist.linear.y = 0.0  # 差動二輪なのでy方向の速度は0
            odom.twist.twist.angular.z = velocities[1]

            odom_manager.odom_publisher.publish(odom)

            # 次のループのために、再度エンコーダ値の取得を開始
            odom_manager.get_encoder_values()

        # ループの周期を保つための待機
        time.sleep(0.025)  # 約40Hz

if __name__ == "__main__":
    lightrover_odometry()