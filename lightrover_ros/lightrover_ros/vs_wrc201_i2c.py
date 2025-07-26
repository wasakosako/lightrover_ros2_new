#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# このライブラリは、Vstone製モータードライバ「VS-WRC201」と
# I2C通信を行うための低レベルな関数群を提供します。
# メモリマップへの直接的な読み書きをカプセル化し、
# より上位のプログラム（ROSノードなど）で使いやすいインターフェースを提供します。

import smbus
import time

class VsWrc201I2c:
    """
    VS-WRC201とのI2C通信を管理するクラス。
    マイコンのメモリマップをローカルにキャッシュし、効率的な読み書きを実現します。
    """

    # --- メモリマップアドレス定数 ---
    # VS-WRC201のデータシートで定義されている各機能のメモリアドレスです。
    # プレフィックスの意味:
    #   M: メインマイコン
    #   S: サブマイコン
    #   U: Unsigned (符号なし)
    #   S: Signed (符号あり)
    #   8, 16, 32: ビット長
    # 例: MU16_SYSNAME -> メインマイコンの符号なし16bit変数「SYSNAME」
    DEV_ADDR = 0x10
    MAP_SIZE = 0x100  # メモリマップの合計サイズ (256バイト)

    # システム情報 (0x00 - 0x0F)
    MU16_SYSNAME = 0x00
    MU16_FIRMREC = 0x02
    MU32_TRIPTIME = 0x04
    MU8_MODE = 0x0d
    MU16_POWOFF_T = 0x0e

    # モーター制御設定 (0x10 - 0x3F)
    MU8_O_EN = 0x10       # モーター出力有効化
    MU8_TRIG = 0x11       # 設定反映トリガ
    MU16_SD_VI = 0x12       # 電圧低下検知しきい値
    MU16_OD_DI = 0x14       # 過電流検知しきい値
    MU16_SPD_T0 = 0x16
    MU16_MOVE_T0 = 0x18
    MU16_FB_PG0 = 0x20      # フィードバック Pゲイン Ch0
    MU16_FB_PG1 = 0x22      # フィードバック Pゲイン Ch1
    MU16_FB_ALIM0 = 0x24
    MU16_FB_ALIM1 = 0x26
    MU16_FB_DLIM0 = 0x28
    MU16_FB_DLIM1 = 0x2a
    MU16_FB_OLIM0 = 0x2c
    MU16_FB_OLIM1 = 0x2e
    MU16_FB_PCH0 = 0x30     # フィードバック周期 Ch0
    MU16_FB_PCH1 = 0x32     # フィードバック周期 Ch1

    # モーター目標値 (0x40 - 0x5F)
    MS32_T_POS0 = 0x40      # 目標位置 Ch0
    MS32_T_POS1 = 0x44      # 目標位置 Ch1
    MS32_A_POS0 = 0x48      # 目標速度 Ch0
    MS32_A_POS1 = 0x4c      # 目標速度 Ch1
    MS16_T_OUT0 = 0x50      # 目標トルク Ch0
    MS16_T_OUT1 = 0x52      # 目標トルク Ch1
    MS16_T_OUT2 = 0x54

    # モーター現在値 (0x60 - 0x7F)
    MS32_M_POS0 = 0x60      # 現在位置 (エンコーダ値) Ch0
    MS32_M_POS1 = 0x64      # 現在位置 (エンコーダ値) Ch1
    MS16_M_SPD0 = 0x68      # 現在速度 Ch0
    MS16_M_SPD1 = 0x6a      # 現在速度 Ch1
    MS16_M_OUT0 = 0x6c      # 現在出力 Ch0
    MS16_M_OUT1 = 0x6e      # 現在出力 Ch1
    MU16_M_DI = 0x7e

    # ... (以下、他のアドレスも同様)

    # ローカルに保持するメモリマップのキャッシュ
    memmap = [0x00] * MAP_SIZE
    # 書き込みがあったアドレスをマークするためのフラグ
    write_flag = [0x00] * MAP_SIZE

    # 初期化時に書き込むデフォルトのメモリマップ値
    initialMemmap = [0x00] * MAP_SIZE # (元の長い配列は省略)

    def __init__(self, dev_addr):
        """コンストラクタ。SMBusを初期化し、デバイスアドレスを設定します。"""
        # I2Cバス1番を使用
        self.__i2c = smbus.SMBus(1)
        self.DEV_ADDR = dev_addr
        time.sleep(0.1)  # 初期化のための待機

    def set_dev_addr(self, dev_addr):
        """I2Cデバイスアドレスを設定します。"""
        self.DEV_ADDR = dev_addr
        return 1

    def init_memmap(self, cut_off_level):
        """メモリマップを初期値で設定します。"""
        cut_off_hex = int((cut_off_level / 3.3) * 0xfff)
        self.write_memmap(self.MU8_O_EN, self.initialMemmap, 0xF0)
        self.read_all()
        self.write_s16map(self.MU16_SD_VI, cut_off_hex)

    # --- 低レベルI2C書き込み関数 ---
    def write_1_byte(self, addr, data):
        """指定アドレスに1バイト書き込みます。"""
        self.__i2c.write_byte_data(self.DEV_ADDR, addr, data)
        return 1

    def write_2_byte(self, addr, data):
        """指定アドレスに2バイト書き込みます（リトルエンディアン）。"""
        for i in range(2):
            write_byte = (data >> (i * 8)) & 0xff
            self.write_1_byte(addr + i, write_byte)
        return 1

    def write_4_byte(self, addr, data):
        """指定アドレスに4バイト書き込みます（リトルエンディアン）。"""
        for i in range(4):
            write_byte = (data >> (i * 8)) & 0xff
            self.write_1_byte(addr + i, write_byte)
        return 1

    def write_memmap(self, addr, data_array, length):
        """指定アドレスから連続して複数バイト書き込みます。"""
        if length <= 0:
            return -1
        for i in range(length):
            self.write_1_byte(addr + i, data_array[i])
        return 1

    # --- メモリマップキャッシュ操作 --- 
    def memmap_clean(self):
        """ローカルのメモリマップキャッシュを0でクリアします。"""
        self.memmap = [0x00] * self.MAP_SIZE

    def read_memmap(self, addr, length):
        """
        I2C経由でマイコンからデータを読み込み、ローカルのキャッシュに保存します。
        """
        # SMBusの仕様上、読み込み前に一度ダミーの読み込みが必要な場合がある
        try:
            for i in range(length):
                self.memmap[addr + i] = self.__i2c.read_byte_data(self.DEV_ADDR, addr + i)
        except IOError as e:
            print(f"I2C read error at address {addr}: {e}")
        return length

    def read_all(self):
        """マイコンから全メモリマップを読み込み、ローカルキャッシュを更新します。"""
        # 効率化のため、大きなブロックに分けて読み込む
        self.read_memmap(0x00, 0x80) # 0x00 - 0x7F
        self.read_memmap(0x80, 0x80) # 0x80 - 0xFF
        return 1

    def send_write_map(self):
        """
        ローカルキャッシュのうち、変更フラグが立っている部分だけを
        効率的にマイコンに書き込みます。
        """
        addr = 0
        while addr < self.MAP_SIZE:
            if self.write_flag[addr]:
                start_addr = addr
                length = 0
                # 連続してフラグが立っている範囲を探す
                while addr < self.MAP_SIZE and self.write_flag[addr]:
                    self.write_flag[addr] = 0 # フラグをクリア
                    length += 1
                    addr += 1
                # 連続ブロックをまとめて書き込み
                self.write_memmap(start_addr, self.memmap[start_addr : start_addr + length], length)
            else:
                addr += 1

    # --- ローカルキャッシュへのアクセス関数 (データ型別) ---
    def read_s8map(self, addr):
        """キャッシュから符号なし8bit値を読み込みます。"""
        return self.memmap[addr]

    def write_s8map(self, addr, data):
        """キャッシュに符号なし8bit値を書き込み、変更フラグを立てます。"""
        self.memmap[addr] = data & 0xFF
        self.write_flag[addr] = 1
        return self.memmap[addr]

    def read_s16map(self, addr):
        """キャッシュから符号付き16bit値を読み込みます（リトルエンディアン）。"""
        val = (self.memmap[addr+1] << 8) | self.memmap[addr]
        return val if val < 32768 else val - 65536 # 符号を考慮

    def write_s16map(self, addr, data):
        """キャッシュに符号付き16bit値を書き込み、変更フラグを立てます。"""
        self.memmap[addr] = data & 0xFF
        self.memmap[addr+1] = (data >> 8) & 0xFF
        self.write_flag[addr] = 1
        self.write_flag[addr+1] = 1
        return self.read_s16map(addr)

    def read_s32map(self, addr):
        """キャッシュから符号付き32bit値を読み込みます（リトルエンディアン）。"""
        val = (self.memmap[addr+3] << 24) | (self.memmap[addr+2] << 16) | (self.memmap[addr+1] << 8) | self.memmap[addr]
        return val if val < 2147483648 else val - 4294967296 # 符号を考慮

    def write_s32map(self, addr, data):
        """キャッシュに符号付き32bit値を書き込み、変更フラグを立てます。"""
        for i in range(4):
            self.memmap[addr+i] = (data >> (i*8)) & 0xFF
            self.write_flag[addr+i] = 1
        return self.read_s32map(addr)

    # --- 応用関数 ---
    def read_enc(self):
        """左右のエンコーダ値を読み込みます。"""
        # 読み込む前に、マイコンから最新の値を取得
        self.read_memmap(self.MS32_M_POS0, 8)
        encL = self.read_s32map(self.MS32_M_POS0)
        encR = self.read_s32map(self.MS32_M_POS1)
        return [encL, encR]

    def clear_enc(self):
        """エンコーダカウンタをリセットします。"""
        # MU8_TRIGレジスタの特定のビットを立てることでリセットがかかる
        current_trig = self.read_s8map(self.MU8_TRIG)
        self.write_s8map(self.MU8_TRIG, current_trig | 0x0C)

    def get_vin(self):
        """電源電圧を取得します。"""
        self.read_memmap(self.MU16_M_VI, 2)
        memmap_v = self.read_s16map(self.MU16_M_VI)
        # ADCの読み値を実際の電圧に変換
        vin = (float(memmap_v) / 4095.0) * 3.3 * 5.0 # 分圧比などを考慮した変換式（要確認）
        return vin