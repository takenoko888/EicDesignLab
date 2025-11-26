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
  - D (微分): P制御の行き過ぎを抑る

参考サイト
	https://gpiozero.readthedocs.io/en/stable/index.html
"""

import numpy as np
import time
from gpiozero import MCP3004, Robot, Motor, Button
from subprocess import check_call
from signal import pause

# 状態管理
running = False
motors = None

def clamped(v):
    """ 値の制限 [-1,1] """
    return max(-1,min(1,v))

class LFController:
    """ ライントレース制御クラス 
        
        ライントレース制御アルゴリズムを実装する。

        入力　フォトリフレクの値 [0,1]x4
        出力　モータ制御信号 [-1,1]x2
    """
    
    def __init__(self,prs = None):
        self._prs = prs
        # 制御パラメータ
        self.base_speed = 0.3   # 基本速度（0.3 に設定)
        # PIDゲイン
        self.Kp = 1.0   # P (比例): 1.0 に設定
        self.Ki = 0.5   # I (積分): 0.5 に設定
        self.Kd = 0   # D (微分): ゼロに設定
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
        # センサー配置: [左外, 左中, 右中, 右外] 
        weights = np.array([-3.2, -1.2, 1.2, 3.2])
        
        # 交差点対策: 中央の2つのセンサーが両方とも反応している場合
        # ラインの真上にいると判断し、交差ラインの影響を受けないよう「強制的に直進」させる
        if s[1] > 0.4 and s[2] > 0.4:
             error = 0.0
        else:
             # 通常時は全センサーを使用
             # 外側のセンサーの重みを調整
             weights = np.array([-0, -1.2, 1.2, 0]) 
             error = np.dot(s, weights)

        # PID制御の計算
        # P (比例)
        p_term = self.Kp * error

        # I (積分)
        self.integral += error * dt
        self.integral = max(-5.0, min(5.0, self.integral))  
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
        global running
        while True:
            if running:
                yield self.prs2mtrs()
            else:
                yield (0, 0)

def toggle():
    """ 開始停止切替関数 """
    global running, motors
    running = not running
    if not running and motors:
        motors.stop()

def shutdown():
    """ シャットダウン関数 """
    if motors:
        motors.stop()
    check_call(['sudo', 'poweroff'])

def main():
    """ メイン関数 """
    global motors
    
    # 接続ピン
    PIN_BT = 3 # GPIO25かもしれない
    
    # モーター設定
    motor_left = Motor(forward=6, backward=5, pwm=True)
    motor_right = Motor(forward=26, backward=27, pwm=True)
    motors = Robot(left=motor_left, right=motor_right)
    
    # フォトリフレクタ設定
    photorefs = [MCP3004(channel=i) for i in range(4)]
    
    # ボタン長押し時間設定
    button = Button(PIN_BT, hold_time=2)
    # ボタン押下時のコールバック設定
    button.when_pressed = toggle
    # ボタン長押し時のコールバック設定
    button.when_held = shutdown

    # ライントレース開始
    lf = LFController(photorefs)
    motors.source = lf.line_follow()
    
    # 停止(Ctrl+c)まで待機
    pause()

if __name__ == '__main__':
    main()