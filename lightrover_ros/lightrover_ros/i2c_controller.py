#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このプログラムは、I2C通信を介してモータードライバボード「VS-WRC201」を
# 制御するためのROS2サービスサーバーです。
# 他のノードは、このサービスを通じてハードウェア（モータードライバ）と通信します。

import rclpy
from rclpy.node import Node
import lightrover_ros.vs_wrc201_i2c as vs_wrc201_i2c
import time

from lightrover_interface.srv import Wrc201Msg

# VS-WRC201のデフォルトI2Cアドレス(0x10)でI2C通信オブジェクトを初期化
i2c = vs_wrc201_i2c.VsWrc201I2c(0x10)

class I2cController(Node):
    """
    'wrc201_i2c'という名前のサービスを提供するクラス。
    このサービスは、Wrc201Msg型のメッセージを使用して、
    モータードライバのメモリマップへの読み書き要求を処理します。
    """
    def __init__(self):
        super().__init__('wrc201_i2c_server')
        # 'wrc201_i2c'サービスを作成。リクエストはhandle_wrc201_i2cメソッドで処理される
        self.srv = self.create_service(Wrc201Msg, 'wrc201_i2c', self.handle_wrc201_i2c)
        self.get_logger().info('WRC201 I2C service server has been started.')

    def handle_wrc201_i2c(self, request, response):
        """
        サービスリクエストを処理するコールバック関数。
        リクエストの`cmd`フィールドに応じて、異なるI2C操作を実行します。

        Args:
            request: Wrc201Msg.Request オブジェクト
            response: Wrc201Msg.Response オブジェクト

        Returns:
            Wrc201Msg.Response オブジェクト
        """
        # self.get_logger().info('Incoming request: addr=%d, cmd="%s"' % (request.addr, request.cmd))

        # 'w': Write (書き込み) コマンド
        if request.cmd == "w":
            # 指定されたアドレスに、指定された長さのデータを書き込む
            try:
                if request.length == 4:  # 4バイト
                    i2c.write_4_byte(request.addr, request.data)
                elif request.length == 2:  # 2バイト
                    i2c.write_2_byte(request.addr, request.data)
                elif request.length == 1:  # 1バイト
                    i2c.write_1_byte(request.addr, request.data)
                response.read_data = 1  # 成功を示すために1を返す
            except IOError as e:
                self.get_logger().error(f"I2C write error: {e}")
                return None
            return response

        # 's': Send (送信) コマンド
        elif request.cmd == "s":
            # ローカルのメモリマップキャッシュの内容をすべてマイコンに書き込む
            try:
                i2c.send_write_map()
                response.read_data = 1  # 成功
            except IOError as e:
                self.get_logger().error(f"I2C send_write_map error: {e}")
                return None
            return response

        # 'rm': Read Map (マップ読み込み) コマンド
        elif request.cmd == 'rm':
            # マイコンからすべてのメモリマップを読み込み、ローカルキャッシュを更新する
            try:
                i2c.read_all()
                response.read_data = 1  # 成功
            except IOError as e:
                self.get_logger().error(f"I2C read_all error: {e}")
                return None
            return response

        # 'r': Read (読み込み) コマンド
        elif request.cmd == "r":
            # 指定されたアドレスから、指定された長さのデータを読み込む
            try:
                # まず、指定されたアドレスと長さのデータをマイコンから読み込む
                i2c.read_memmap(request.addr, request.length)
                # 次に、ローカルキャッシュからその値を読み出して返す
                if request.length == 4:  # 4バイト
                    response.read_data = i2c.read_s32map(request.addr)
                elif request.length == 2:  # 2バイト
                    response.read_data = i2c.read_s16map(request.addr)
                elif request.length == 1:  # 1バイト
                    response.read_data = i2c.read_s8map(request.addr)
                else:
                    response.read_data = 0
            except IOError as e:
                self.get_logger().error(f"I2C read error: {e}")
                return None
            return response

def wrc201_i2c_server(args=None):
    """ノードの初期化と実行を行うメイン関数"""
    rclpy.init(args=args)

    # I2cControllerノードを作成
    i2c_controller_node = I2cController()

    # ROSパラメータからI2Cデバイスアドレスを取得。指定がなければ0x10を使用。
    p_dev_addr = i2c_controller_node.declare_parameter('dev_addr', 0x10).value

    # I2C通信オブジェクトにデバイスアドレスを設定
    i2c.set_dev_addr(p_dev_addr)
    # 起動時にマイコンから全メモリマップを読み込む
    i2c.read_all()
    # メモリマップの初期設定を行う（PIDゲインなど）
    i2c.init_memmap(2.0)
    # 設定した値をマイコンに書き込む
    i2c.send_write_map()

    # ノードをスピンさせ、サービスリクエストを待ち受ける
    rclpy.spin(i2c_controller_node)

if __name__ == '__main__':
    wrc201_i2c_server()