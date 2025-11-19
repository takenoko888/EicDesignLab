#!/usr/bin/python3
# coding: UTF-8

"""
ライントレーサー（PID制御版）

「電子情報通信設計製図」新潟大学工学部工学科電子情報通信プログラム

【キモ①】ADC (MCP3004): SPI通信でアナログ値を取得
【キモ②】モータードライバ (DRV8835): IN/INモード、ブレーキ機能でキレのある旋回
【本命】 PID制御:
  - P (比例): ズレた分だけ戻す基本制御
  - I (積分): モーター個体差による定常偏差を補正
  - D (微分): P制御の行き過ぎを抑え、ハンチング防止
"""

import numpy as np
import time
from gpiozero import MCP3004, Robot, Motor
from signal import pause

def clamped(v):
    """ 値の制限 [0.6,1] """
    return max(0,min(0.6,v))

class LFController:
    """ ライントレース制御クラス 
        
        ライントレース制御アルゴリズムを実装する。

        入力　フォトリフレクの値 [0,1]x4
        出力　モータ制御信号 [-1,1]x2
    """
    
    def __init__(self,prs = None):
        self._prs = prs
        # 制御パラメータ
        self.base_speed = 0.05  # 基本速度（大きくすると速くなるが、カーブで脱線しやすくなる）
        # PIDゲイン
        self.Kp = 0.8   # P (比例): 現在のズレに対する反応の強さ。大きくすると急カーブに強くなるが、ふらつきやすくなる。
        self.Ki = 0.01  # I (積分): 過去のズレの蓄積。小さなズレが続く場合に効く。大きくしすぎると暴走の原因になる。
        self.Kd = 0.5   # D (微分): ズレの変化スピードに対する反応。ふらつき（振動）を抑えるブレーキの役割。
        # PID制御用の変数
        self.last_error = 0.0   # 前回の誤差
        self.integral = 0.0     # 誤差の累積
        self.last_time = time.time()

    def prs2mtrs(self):
        """フォトリフレクタの値をモーター制御値に変換"""
        # 時間差を計算（PID制御の精度向上のため）
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: dt = 1e-3 # ゼロ除算防止
        self.last_time = current_time

        # フォトリフレクタの値を読み出し
        # 白を検出すると 0，黒を検出すると 1 (シミュレータの仕様によるが、通常は黒が高い値)
        s = np.array([ self._prs[idx].value for idx in range(len(self._prs)) ])

        # ラインからのズレを計算（重み付け平均）
        # センサー配置: [左外, 左中, 右中, 右外] (インデックス 0, 1, 2, 3)
        # 左にずれたらマイナス、右にずれたらプラスになるように重み付け
        # 外側の値(絶対値)を大きくすると急カーブに反応しやすくなる
        # 内側の値(絶対値)を大きくすると直進性が増し、交差点で迷いにくくなる
        weights = np.array([-2.5, -1.5, 1.5, 2.6])
        error = np.dot(s, weights)

        # PID制御の計算
        # P (比例)
        p_term = self.Kp * error

        # I (積分)
        self.integral += error * dt
        self.integral = max(-5.0, min(5.0, self.integral))  # ワインドアップ防止
        i_term = self.Ki * self.integral

        # D (微分)
        d_term = self.Kd * (error - self.last_error) / dt
        self.last_error = error

        # PID補正値の合計
        correction = p_term + i_term + d_term

        # 左右モーターの速度を決定
        # correction > 0 (右寄り) -> 左モーター増速、右モーター減速 -> 右旋回
        # correction < 0 (左寄り) -> 左モーター減速、右モーター増速 -> 左旋回
        left = self.base_speed + correction
        right = self.base_speed - correction

        return (clamped(left), clamped(right))

    @property
    def photorefs(self):
        return self._prs

    @photorefs.setter
    def photorefs(self,prs):
        self._prs = prs

    def line_follow(self):
        """制御値を生成し続ける"""
        while True:
                yield self.prs2mtrs()

def main():
    """メイン関数"""
    # モータードライバ接続ピン（DRV8835）
    PIN_AIN1 = 6
    PIN_AIN2 = 5
    PIN_BIN1 = 26
    PIN_BIN2 = 27
    NUM_CH = 4

    # キモ② DRV8835 (IN/INモード): pwm=Trueでブレーキ機能が有効
    # → カーブで内側のタイヤが空転せず、ブレーキがかかってキレのある旋回
    
    # 1. DRV8835 (IN/INモード) に合わせて、個別のMotorオブジェクトを作成
    #    (forward, backward) のペアでピンを指定し、pwm=True に設定
    motor_left = Motor(forward=PIN_AIN1, backward=PIN_AIN2, pwm=True)
    motor_right = Motor(forward=PIN_BIN1, backward=PIN_BIN2, pwm=True)

    # 2. 作成したMotorオブジェクトをRobotクラスに渡す
    # これで、キモであるブレーキ機能が有効になります！
    motors = Robot(left=motor_left, right=motor_right)

    # キモ① MCP3004: SPI通信でアナログ値取得
    photorefs = [MCP3004(channel=i) for i in range(NUM_CH)]

    # 本命！ PID制御でライントレース開始
    lf = LFController(photorefs)
    motors.source = lf.line_follow()
    pause()  # Ctrl+Cで停止

if __name__ == '__main__':
    main()