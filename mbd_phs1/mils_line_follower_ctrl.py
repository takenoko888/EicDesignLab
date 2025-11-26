#!/usr/bin/python3
# coding: UTF-8
"""
ライントレーサ制御クラス

説明

　制御アルゴリズムの変更についてはprs2mtrs() メソッドを編集してください。

参考資料

- 三平 満司：「非ホロノミック系のフィードバック制御」計測と制御
　1997 年 36 巻 6 号 p. 396-403
　
「電子情報通信設計製図」新潟大学工学部工学科電子情報通信プログラム

All rights revserved 2019-2023 (c) Shogo MURAMATSU
"""
import numpy as np
import time

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
        self.base_speed = 0.3   # 基本速度（0.05 -> 0.2 に変更してキビキビ動くように）
        # PIDゲイン
        self.Kp = 1.0   # P (比例): の行列演算を再現するため 1.0 に設定
        self.Ki = 0.5   # I (積分): 
        self.Kd = 0   # D (微分): 「うまくいくコード」には微分項がないためゼロに設定
        # PID制御用の変数
        self.last_error = 0.0   # 前回の誤差
        self.integral = 0.0     # 誤差の累積
        self.last_time = time.time()

    def prs2mtrs(self):
        """ フォトリフレクタからモータ制御信号への変換メソッド """

        # 時間差を計算
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
        weights = np.array([-3.2, -1.2, 1.2, 3.2])
        
        # 交差点対策: 中央の2つのセンサーが両方とも反応している場合
        # ラインの真上にいると判断し、交差ラインの影響を受けないよう「強制的に直進」させる
        # これにより、X字交差などで左右に振られるのを防ぐ
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
