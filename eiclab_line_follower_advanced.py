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
from gpiozero import MCP3004, Robot
from signal import pause

class LineFollower:
    """PID制御でラインを追従するクラス"""

    def __init__(self, photorefs):
        # キモ① ADC (MCP3004): 4つのフォトリフレクタ
        self.prs = photorefs
        # 制御パラメータ
        self.base_speed = 0.4  # 基本速度（両輪の平均）
        # 本命！ PIDゲイン
        self.Kp = 0.5   # P (比例): ズレた分だけ戻す
        self.Ki = 0.1   # I (積分): 定常偏差を補正（モーター個体差対策）
        self.Kd = 0.2   # D (微分): 行き過ぎを抑える（ハンチング防止）
        # PID制御用の変数
        self.last_error = 0.0   # 前回の誤差（D制御用）
        self.integral = 0.0     # 誤差の累積（I制御用）
        self.last_time = time.time()

    def prs2mtrs(self):
        """フォトリフレクタの値をモーター制御値に変換"""
        # 時間差を計算（PID制御の精度向上のため）
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # キモ① MCP3004でセンサー値を読み取り（SPI通信）
        s = np.array([self.prs[i].value for i in range(4)])

        # ラインからのズレを計算（重み付け平均）
        # センサー配置: [左外, 左中, 右中, 右外]
        weights = np.array([-2.0, -1.0, 1.0, 2.0])
        error = np.dot(s, weights)

        # 本命！ PID制御の計算
        # P (比例): ズレた分だけ戻す
        p_term = self.Kp * error

        # I (積分): 定常偏差を補正（モーター個体差でまっすぐ進まない問題を解決）
        self.integral += error * dt
        self.integral = max(-5.0, min(5.0, self.integral))  # ワインドアップ防止
        i_term = self.Ki * self.integral

        # D (微分): 行き過ぎを抑える（急カーブでのふらつき防止）
        d_term = self.Kd * (error - self.last_error) / dt
        self.last_error = error

        # PID補正値の合計
        correction = p_term + i_term + d_term

        # 左右モーターの速度を決定
        # キモ② DRV8835のIN/INモード + PWM = ブレーキ機能でキレのある旋回
        left = self.base_speed + correction
        right = self.base_speed - correction

        return (clamped(left), clamped(right))

    def line_follow(self):
        """制御値を生成し続ける"""
        while True:
                yield self.prs2mtrs()

def clamped(v):
    """値を-1.0～1.0に制限"""
    return max(-1, min(1, v))

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
    motors = Robot(left=(PIN_AIN1, PIN_AIN2),
                   right=(PIN_BIN1, PIN_BIN2),
                   pwm=True)  # 重要！PWM有効でブレーキ機能が働く

    # キモ① MCP3004: SPI通信でアナログ値取得
    photorefs = [MCP3004(channel=i) for i in range(NUM_CH)]

    # 本命！ PID制御でライントレース開始
    lf = LineFollower(photorefs)
    motors.source = lf.line_follow()
    pause()  # Ctrl+Cで停止

if __name__ == '__main__':
    main()