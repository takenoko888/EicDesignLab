#!/usr/bin/python3
# coding: UTF-8
"""
ライントレーサー起動ボタン

ボタンを押すとライントレースを開始し、もう一度押すと停止する。
長押し（2秒）でシャットダウン。

「電子情報通信設計製図」新潟大学工学部工学科電子情報通信プログラム
"""

import numpy as np
import time
from gpiozero import MCP3004, Robot, Motor, Button, LED
from subprocess import check_call
from signal import pause
import threading

# ========== ピン設定 ==========
PIN_BUTTON = 3      # スタート/ストップボタン
PIN_LED = 17        # 状態表示LED（オプション）
PIN_AIN1 = 6        # モーター左 前進
PIN_AIN2 = 5        # モーター左 後退
PIN_BIN1 = 26       # モーター右 前進
PIN_BIN2 = 27       # モーター右 後退
NUM_CH = 4          # フォトリフレクタ数

def clamped(v):
    """ 値の制限 [-1,1] """
    return max(-1, min(1, v))

class LFController:
    """ ライントレース制御クラス """
    
    def __init__(self, prs=None):
        self._prs = prs
        self._running = False
        # 制御パラメータ
        self.base_speed = 0.3
        # PIDゲイン
        self.Kp = 1.0
        self.Ki = 0.5
        self.Kd = 0
        # PID制御用の変数
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def reset(self):
        """制御状態をリセット"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def start(self):
        """走行開始"""
        self._running = True
        self.reset()

    def stop(self):
        """走行停止"""
        self._running = False

    @property
    def is_running(self):
        return self._running

    def prs2mtrs(self):
        """フォトリフレクタの値をモーター制御値に変換"""
        if not self._running:
            return (0, 0)  # 停止

        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 1e-3
        self.last_time = current_time

        s = np.array([self._prs[idx].value for idx in range(len(self._prs))])

        # 交差点対策
        if s[1] > 0.4 and s[2] > 0.4:
            error = 0.0
        else:
            weights = np.array([-0, -1.2, 1.2, 0])
            error = np.dot(s, weights)

        # PID制御
        p_term = self.Kp * error
        self.integral += error * dt
        self.integral = max(-5.0, min(5.0, self.integral))
        i_term = self.Ki * self.integral
        d_term = self.Kd * (error - self.last_error) / dt
        self.last_error = error

        correction = p_term + i_term + d_term
        left = self.base_speed + correction
        right = self.base_speed - correction

        return (clamped(left), clamped(right))

    @property
    def photorefs(self):
        return self._prs

    @photorefs.setter
    def photorefs(self, prs):
        self._prs = prs

    def line_follow(self):
        """制御値を生成し続ける"""
        while True:
            yield self.prs2mtrs()


class LineFollowerDaemon:
    """ライントレーサーデーモン"""

    def __init__(self):
        # モーター設定
        motor_left = Motor(forward=PIN_AIN1, backward=PIN_AIN2, pwm=True)
        motor_right = Motor(forward=PIN_BIN1, backward=PIN_BIN2, pwm=True)
        self.motors = Robot(left=motor_left, right=motor_right)

        # フォトリフレクタ設定
        self.photorefs = [MCP3004(channel=i) for i in range(NUM_CH)]

        # コントローラ設定
        self.controller = LFController(self.photorefs)
        self.motors.source = self.controller.line_follow()

        # ボタン設定
        self.button = Button(PIN_BUTTON, hold_time=2)
        self.button.when_pressed = self.on_button_pressed
        self.button.when_held = self.on_button_held

        # LED設定（オプション：接続されていなくても動作）
        try:
            self.led = LED(PIN_LED)
        except:
            self.led = None

        print("ライントレーサーデーモン起動")
        print("  ボタン短押し: スタート/ストップ")
        print("  ボタン長押し(2秒): シャットダウン")

    def on_button_pressed(self):
        """ボタン押下時のコールバック"""
        if self.controller.is_running:
            # 走行中 → 停止
            self.controller.stop()
            if self.led:
                self.led.off()
            print("停止")
        else:
            # 停止中 → 走行開始
            self.controller.start()
            if self.led:
                self.led.on()
            print("走行開始")

    def on_button_held(self):
        """ボタン長押し時のコールバック（シャットダウン）"""
        print("シャットダウン...")
        self.controller.stop()
        self.motors.stop()
        if self.led:
            self.led.blink(on_time=0.1, off_time=0.1)
        check_call(['sudo', 'wall', 'poweroff'])
        check_call(['sudo', 'poweroff'])

    def run(self):
        """デーモン実行"""
        pause()


def main():
    """メイン関数"""
    daemon = LineFollowerDaemon()
    daemon.run()


if __name__ == '__main__':
    main()
