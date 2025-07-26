#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このライブラリは、VS-WRC201で制御されるモーターの
# 速度制御（PID）および位置制御のロジックを提供します。
# 上位のノード（例: pos_controller.py）から呼び出され、
# 目標速度と現在の速度（または位置）に基づいて、モータードライバに送るべき
# 指令値を計算します。

import time
import math
import numpy as np

class VsWrc201Motor:
    """
    モーター制御の計算ロジックをカプセル化するクラス。
    """

    # --- 定数定義 ---
    # 物理パラメータ
    ROVER_D = 0.0717767  # 車輪間距離の半分 (m)
    TIRE_CIRCUMFERENCE = 60 * math.pi / 1000  # タイヤの円周 (m)
    ENC_COUNTS_PER_TURN = 1188.024  # タイヤ1回転あたりのエンコーダカウント数
    ENC_PER_M = ENC_COUNTS_PER_TURN / TIRE_CIRCUMFERENCE  # 1mあたりのエンコーダカウント数

    # モーター制御パラメータのインデックス
    INDEX_MAX_SPEED = 0
    INDEX_MAX_RAD = 1
    INDEX_K_V2MP = 2
    INDEX_K_P = 3
    INDEX_K_I = 4
    INDEX_K_D = 5
    INDEX_PAD_DEAD = 6
    INDEX_PAD_MAX = 7
    INDEX_MAX_ACC = 8

    # デフォルトのモーター制御パラメータ [最大速度, 最大角速度, V2MP, Kp, Ki, Kd, ...]
    std_motor_param = [0.5, math.pi, 520.0, 3.6, 2.0, 3.6, 30.0, 127.0, 1.5]
    # 現在のモーター制御パラメータ（実行時に変更される可能性あり）
    motor_param = list(std_motor_param)

    # --- 状態変数 ---
    # 速度関連
    ctl_v_com = [0.0, 0.0]  # 最終的な目標車輪速度 [右, 左]
    v_com = [0.0, 0.0]      # 加速減速を考慮した、現在フレームでの目標車輪速度 [右, 左]
    v_enc = [0.0, 0.0]      # エンコーダから計算された現在の車輪速度 [右, 左]
    avr_v = [0.0, 0.0]      # 移動平均フィルタをかけた現在の車輪速度 [右, 左]
    sum_v = [[0.0] * 5, [0.0] * 5] # 移動平均計算用のバッファ

    # PID制御関連
    v_diff = [0.0, 0.0]         # 速度の偏差 (目標 - 現在)
    prev_v_diff = [0.0, 0.0]    # 1ステップ前の速度偏差
    prev2_v_diff = [0.0, 0.0]   # 2ステップ前の速度偏差
    m_com = [0.0, 0.0]          # PID計算後のモーター出力指令値 [右, 左]
    prev_m = [0, 0]             # 1ステップ前のモーター出力指令値

    # 位置制御関連
    buf_enc_com = [0.0, 0.0] # 位置指令の小数点以下の誤差を保持するバッファ

    # 時間管理
    pre_t_calc_2_v = -1.0
    pre_t_pos_ctl = -1.0
    pid_time = 0.0

    # その他
    MIN_OUTPUT = 2500  # 停止状態から動き出すための最小出力値
    M_L = 1  # 左モーターのインデックス
    M_R = 0  # 右モーターのインデックス

    def calc_2_v(self, x_speed, z_speed):
        """
        ロボットの目標並進速度(x_speed)と目標角速度(z_speed)から、
        左右の車輪がそれぞれ出すべき目標速度を計算します（逆運動学）。
        """
        # パラメータをデフォルト値にリセット
        self.set_motor_param_2_std()

        # 差動二輪モデルの逆運動学の式
        # 左輪速度 = 並進速度 - (トレッド/2) * 角速度
        # 右輪速度 = 並進速度 + (トレッド/2) * 角速度
        # この実装では左右のモーターの回転方向が逆のため、右に-1.0を乗算
        self.ctl_v_com[self.M_L] = (x_speed - self.ROVER_D * z_speed)
        self.ctl_v_com[self.M_R] = -1.0 * (x_speed + self.ROVER_D * z_speed)

        # 計算された車輪速度が最大速度を超えている場合、比率を保ったままスケールダウンする
        max_abs_v = max(abs(self.ctl_v_com[0]), abs(self.ctl_v_com[1]))
        if max_abs_v > self.motor_param[self.INDEX_MAX_SPEED]:
            scale = self.motor_param[self.INDEX_MAX_SPEED] / max_abs_v
            self.ctl_v_com[0] *= scale
            self.ctl_v_com[1] *= scale

        return self.ctl_v_com

    def ctl_2_v_com(self):
        """
        目標速度(ctl_v_com)に滑らかに到達するように、加速度制限を考慮した
        中間的な目標速度(v_com)を計算します。
        """
        if self.pre_t_calc_2_v < 0:
            self.pre_t_calc_2_v = time.time()
            return

        # 経過時間を計算
        new_micros = time.time()
        elapsed_time = new_micros - self.pre_t_calc_2_v
        self.pre_t_calc_2_v = new_micros

        # 各車輪について、最大加速度を超えないように目標速度を更新
        for i in range(2):
            v_diff = self.ctl_v_com[i] - self.v_com[i]
            max_delta_v = self.motor_param[self.INDEX_MAX_ACC] * elapsed_time

            if abs(v_diff) > max_delta_v:
                self.v_com[i] += max_delta_v if v_diff > 0 else -max_delta_v
            else:
                self.v_com[i] = self.ctl_v_com[i]

            # ほぼゼロになったら完全にゼロにする
            if self.ctl_v_com[i] == 0.0 and abs(self.v_com[i]) <= 0.02:
                self.v_com[i] = 0.0

    def set_motor_param_2_std(self):
        """モーターパラメータをデフォルト値にリセットします。"""
        self.motor_param = list(self.std_motor_param)

    def pid_controll(self, rover_v, target_v):
        """
        PID制御ロジック。現在の車輪速度(rover_v)と目標車輪速度(target_v)を
        元に、モーターへの出力指令値(m_com)を計算します。
        """
        # 制御周期が短すぎる場合は処理をスキップ
        current_time = time.time()
        if current_time - self.pid_time < 0.00001: # 10us
            return self.m_com
        self.pid_time = current_time

        self.v_enc = rover_v      # 現在の速度を設定
        self.ctl_v_com = target_v # 最終目標速度を設定

        # 加速度を考慮した中間目標速度を計算
        self.ctl_2_v_com()

        for i in range(2):
            # --- 移動平均フィルタ ---
            # ノイズの多いエンコーダの値を平滑化する
            if abs(self.v_enc[i]) > 2 * self.motor_param[self.INDEX_MAX_SPEED]:
                self.v_enc[i] = self.sum_v[i][0] # 異常値は前回の値で代用
            
            # 過去5回分の速度を合計して平均を計算
            self.sum_v[i].pop()
            self.sum_v[i].insert(0, self.v_enc[i])
            self.avr_v[i] = sum(self.sum_v[i]) / 5.0

            # --- PID計算 ---
            self.v_diff[i] = self.v_com[i] - self.avr_v[i] # P (比例) 項: 偏差
            
            # I (積分) 項: 偏差の累積 (この実装では直接的ではないが、prev_mがその役割を担う)
            # D (微分) 項: 偏差の変化率
            p_term = (self.v_diff[i] - self.prev_v_diff[i]) * self.motor_param[self.INDEX_K_P]
            d_term = ((self.v_diff[i] - self.prev_v_diff[i]) - (self.prev_v_diff[i] - self.prev2_v_diff[i])) * self.motor_param[self.INDEX_K_D]
            i_term = self.v_diff[i] * self.motor_param[self.INDEX_K_I]

            # 指令値を更新
            self.m_com[i] = self.prev_m[i] + int((i_term + p_term + d_term) * self.motor_param[self.INDEX_K_V2MP])

            # --- 出力調整 ---
            # 最小出力値（静止摩擦を超えるため）
            if abs(target_v[i]) >= 0.05 and abs(self.m_com[i]) <= self.MIN_OUTPUT:
                self.m_com[i] = self.MIN_OUTPUT if target_v[i] > 0 else -self.MIN_OUTPUT

            # 最大出力値（リミッター）
            self.m_com[i] = max(-5000, min(5000, self.m_com[i]))

            # 過去の値を更新
            self.prev2_v_diff[i] = self.prev_v_diff[i]
            self.prev_v_diff[i] = self.v_diff[i]
            self.prev_m[i] = self.m_com[i]

        # 両輪の目標速度と実測速度がほぼゼロなら、出力を完全にゼロにする
        if self.v_com[self.M_L] == 0.0 and self.v_com[self.M_R] == 0.0 and \
           abs(self.avr_v[self.M_L]) < 0.05 and abs(self.avr_v[self.M_R]) < 0.05:
            self.m_com = [0, 0]
            self.prev_m = [0, 0]

        return self.m_com

    def pos_controll(self, rover_v, target_v):
        """
        位置制御ロジック。目標車輪速度(target_v)を積分して、
        モータードライバに送るべき目標エンコーダカウント(位置指令)を計算します。
        """
        self.ctl_v_com = target_v

        # 加速度を考慮した中間目標速度を計算
        self.ctl_2_v_com()

        # 目標速度(m/s)を目標エンコーダ速度(count/s)に変換
        e_v_com = [
            self.v_com[self.M_L] * self.ENC_PER_M,
            self.v_com[self.M_R] * self.ENC_PER_M
        ]

        # 経過時間を計算
        if self.pre_t_pos_ctl < 0:
            self.pre_t_pos_ctl = time.time()
        new_micros = time.time()
        elapsed_time = new_micros - self.pre_t_pos_ctl
        self.pre_t_pos_ctl = new_micros

        # 経過時間分の目標移動量を計算し、バッファに加算
        self.buf_enc_com[self.M_L] += e_v_com[self.M_L] * elapsed_time
        self.buf_enc_com[self.M_R] += e_v_com[self.M_R] * elapsed_time

        # バッファから整数部分のエンコーダ指令値を抽出
        enc_com = [
            int(self.buf_enc_com[self.M_L]),
            int(self.buf_enc_com[self.M_R])
        ]

        # 抽出した分をバッファから減算（小数点以下の誤差を次に持ち越す）
        self.buf_enc_com[self.M_L] -= enc_com[self.M_L]
        self.buf_enc_com[self.M_R] -= enc_com[self.M_R]

        return enc_com