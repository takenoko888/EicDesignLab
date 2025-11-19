#!/usr/bin/python3
# coding: UTF-8
"""
ライントレーサー物理モデル

説明

　物理モデルの変更については以下のパラメータおよび mtrs2twist() メソッドを編集してください。
  
  - LF_MOUNT_POS_PRF # フォトリフレクタの配置
  - LF_WEIGHT        # 車体の重さ g
  - SHAFT_LENGTH     # シャフト長 mm （１本）
  - TIRE_DIAMETER    # タイヤ直径 mm
  - COEF_K_P         # 比例制御係数
  - PARAMS_MU_CLIN     # 直線運動の粘性摩擦係数
  - PARAMS_NU_CROT     # 旋廻運動の粘性摩擦係数

参考資料

- 三平 満司：「非ホロノミック系のフィードバック制御」計測と制御
　36 巻 6 号 p. 396-403, 1997 年 
- Gregor Klancar, Andrej Zdesar, Saso Blazic and Igor Skrjanc: "Wheeled Mobile Robotics,"
  Elsevier, 2017

「電子情報通信設計製図」新潟大学工学部工学科電子情報通信プログラム

All rights revserved 2019-2023 (c) Shogo MURAMATSU
"""
from mils_line_follower_ctrl import LFController
from mils_line_follower_phrf import LFPhotoReflector
from scipy.integrate import odeint
import numpy as np
import pygame

# 車体のパラメータ
#
# シャフト中心(+)からのフォトリフレクタ(*)の
# 相対座標[mm]
#  
#       --|--          * pr1 (dx1,dy1)
#         |           * pr2 (dx2,dy2)       
#  (0,0)  + -------------            → x
#   ↓     |           * pr3 (dx3,dy3)       
#   y   --|--          * pr4 (dx4,dy4)
#
# ((dx1,dy1), (dx2,dy2), (dx3,dy3), (dx4,dy4)) 
#
LF_MOUNT_POS_PRF = ((40,-45), (25,-20), (25,20), (40,45)) # mm
LF_WEIGHT = 360    # 車体の重さ g（グラム）
SHAFT_LENGTH = 50  # シャフト長 mm （１本）
TIRE_DIAMETER = 58 # タイヤ直径 mm

# 制御パラメータ（要調整）
COEF_K_P = 3.0 # 比例制御係数

## 物理モデルパラメータ（要調整）
PARAMS_MU_CLIN = 1e-3 # kg/s 車体の直線運動の粘性摩擦係数（便宜上）
PARAMS_NU_CROT = 1e-3 # kg·m^2/s 車体の旋廻運動の粘性摩擦係数（便宜上）

# フォトリフレクタ数
NUM_PHOTOREFS = 4

# 色の定義
WHITE  = (255, 255, 255)
BLACK  = (  0,   0,   0)
GREEN  = (  0, 255,   0)    
BLUE   = (  0,   0, 255)
YELLOW = (  255, 128, 0)

def rotate_pos(pos,center,angle):
    """ 座標位置の回転 """
    rotmtx = np.asarray([
        [ np.cos(angle), -np.sin(angle) ],
        [ np.sin(angle),  np.cos(angle) ]
    ])
    return rotmtx.dot(pos-center) + center

class LFPhysicalModel:
    """ ライントレーサ物理モデルクラス 
        
        ライントレーサの物理モデルを実装しています。
        モータ制御信号が力に比例するという非常に単純なモデルです。
        
        左右の和を前後運動、左右の差を回転運動に換算しています。

        入力　モータ制御信号 [-1,1]x2        
        出力　フォトリフレクタの値 [0,1]x4 
    """    
    
    def __init__(self, course, \
            weight = LF_WEIGHT, \
            mntposprs = LF_MOUNT_POS_PRF):

        # プロパティの設定
        self._course = course
        self._weight = weight # g 
        self._mntposprs = mntposprs
        self._x_mm = SHAFT_LENGTH + 10 # mm
        self._y_mm = SHAFT_LENGTH + 10 # mm
        self._angle_rad = 0.0 # rad

        # 初期化
        self.reset()

        # 制御機とフォトリフレクタ設定
        self._controller = LFController()
        self._prs = [ LFPhotoReflector(self._course) \
            for idx in range(NUM_PHOTOREFS)]
        for idx in range(NUM_PHOTOREFS):
            self._prs[idx].value = 0.0
        self._controller.photorefs = self._prs

    def mtrs2twist(self,mtrs,v0,w0,fps):
        """ モータ制御信号→速度変換 
            h = 1/fps 間隔で制御信号をゼロ次ホールドすると仮定        
        """
       
        # 車体重量の換算
        mc_kg = 1e-3*self._weight # g -> kg

        # モーター電圧から速度・角速度の計算
        u_r = mtrs[1]
        u_l = mtrs[0]
        ulin = COEF_K_P*(u_r + u_l)/2.0 # 直線運動
        urot = COEF_K_P*(u_r - u_l)/2.0 # 回転運動

        # サンプリング間隔
        h = 1/fps
        
        # 直線速度の計算      
        mu_clin = PARAMS_MU_CLIN # 直線運動の粘性摩擦係数 
        Tlin = (mc_kg+0.5)/(mu_clin+20)  # 時定数
        clin = 1/(0.4*mu_clin+8)
        #v1 = v0*np.exp(-h/Tlin) + clin*(1.0-np.exp(-h/Tlin))*ulin
        v1 = np.exp(-h/Tlin)*( v0 - clin*ulin ) + clin*ulin

        # 回転速度の計算
        nu_crot = PARAMS_NU_CROT # 回転運動の粘性摩擦係数 
        Trot = (mc_kg+0.5)/(40*nu_crot+20) # 時定数
        crot = 1/(8*nu_crot+0.4)
        #w1 = w0*np.exp(-h/Trot) + crot*(1.0-np.exp(-h/Trot))*urot
        w1 = np.exp(-h/Trot)*( w0 - crot*urot ) + crot*urot 

        # 出力
        twist = { "linear":{"x":v1, "y":0., "z":0.}, "angular":{"x":0., "y":0., "z":w1} }
        return twist

    def drive(self,fps):
        """ 車体駆動メソッド"""
        # センサ値更新 
        self._sense()
        
        # モーター制御信号取得
        mtrs = np.asarray(self._controller.prs2mtrs())

        # 車体状態更新
        self.updatestate(mtrs,fps)        
         
    def updatestate(self,mtrs,fps):
        """ 車体駆動メソッド （2020）"""

        # モータ―制御信号→Twist型
        v0_m_s = 1e-3*self._v_mm_s # 前時刻直線速度 m/s
        w0_rad_s = self._w_rad_s   # 前時刻角速度  rad/s
        twist = self.mtrs2twist(mtrs,v0_m_s,w0_rad_s,fps)
        v1_m_s = twist["linear"]["x"]    # 現時刻直線速度 m/s
        w1_rad_s = twist["angular"]["z"] # 現時刻角速度  rad/s

        # 位置・角度情報更新
        t = np.linspace(0,1/fps,2)
        pos = [ 0.0 for idx in range(3) ]
        pos[0] = 1e-3*self._x_mm # m
        pos[1] = 1e-3*self._y_mm # m
        pos[2] = self._angle_rad # rad
        p = odeint(self._odefun,pos,t,args=(v1_m_s,w1_rad_s))
        pos = p[-1]

        # 状態更新 
        self._v_mm_s = 1e3*v1_m_s # m/s -> mm/s
        self._rad_s = w1_rad_s
        self._x_mm = 1e3*pos[0] # m -> mm 
        self._y_mm = 1e3*pos[1] # m -> mm
        self._angle_rad = pos[2]

    def _odefun(self,pos,t,v,w):
        """ 状態方程式 """
        # d_ (  x ) = ( cosθ )v + ( 0 )ω
        # dt (  y )   ( sinθ )    ( 0 )
        #    (  θ )   (  0   )    ( 1 )
        phi = pos[2]
        return [ np.cos(phi)*v, np.sin(phi)*v, w ]

    @property
    def course(self):
        return self._course
    
    @property
    def angle(self):
        return self._angle_rad

    def reset(self):
        self._v_mm_s = 0.0 # mm/s
        self._w_rad_s = 0.0 # rad/s

    def set_position_mm(self,x,y):
        self._x_mm = x # mm
        self._y_mm = y # mm

    def move_px(self,dx_px,dy_px):
        res = self._course.resolution # mm/pixel        
        self._x_mm = self._x_mm + dx_px*res # mm
        self._y_mm = self._y_mm + dy_px*res # mm   

    def rotate(self,angle):
        self._angle_rad = angle # rad

    def set_interval(self,interval):
        self._interval = interval

    def draw_body(self,screen):
        rect = np.asarray(self.get_rect_px())
        center = np.asarray(self.get_center_px())

        # 車体の描画
        apos00 = np.dot([[1,0,0,0],[0,1,0,0]],rect)
        apos10 = np.dot([[1,0,0,0],[0,1,0,1]],rect)
        apos01 = np.dot([[1,0,1,0],[0,1,0,0]],rect)
        apos11 = np.dot([[1,0,1,0],[0,1,0,1]],rect)
        # 
        angle = self._angle_rad
        apos00 = rotate_pos(apos00,center,angle)
        apos10 = rotate_pos(apos10,center,angle)
        apos01 = rotate_pos(apos01,center,angle)
        apos11 = rotate_pos(apos11,center,angle)
        #
        pos00 = apos00.tolist()
        pos10 = apos10.tolist()
        pos01 = apos01.tolist()
        pos11 = apos11.tolist()
        #
        pygame.draw.polygon(screen, YELLOW, [pos00,pos01,pos11,pos10],0)

        # 解像度の読み込み
        res = self._course.resolution # mm/pixel

        # 左タイヤの描画
        pos_ltf = center + np.asarray([TIRE_DIAMETER/2,-SHAFT_LENGTH])/res
        pos_ltr = center + np.asarray([-TIRE_DIAMETER/2,-SHAFT_LENGTH])/res
        pos_ltf = (rotate_pos(pos_ltf,center,angle)+.5).astype(np.int32).tolist()
        pos_ltr = (rotate_pos(pos_ltr,center,angle)+.5).astype(np.int32).tolist()        
        pygame.draw.line(screen, BLACK, pos_ltf,pos_ltr,int(12/res))
        pygame.draw.circle(screen, BLACK, pos_ltf,int(6/res))
        pygame.draw.circle(screen, BLACK, pos_ltr,int(6/res))

        # 右タイヤ の描画     
        pos_rtf = center + np.asarray([TIRE_DIAMETER/2,SHAFT_LENGTH])/res
        pos_rtr = center + np.asarray([-TIRE_DIAMETER/2,SHAFT_LENGTH])/res
        pos_rtf = (rotate_pos(pos_rtf,center,angle)+.5).astype(np.int32).tolist()
        pos_rtr = (rotate_pos(pos_rtr,center,angle)+.5).astype(np.int32).tolist()        
        pygame.draw.line(screen, BLACK, pos_rtf,pos_rtr,int(12/res))   
        pygame.draw.circle(screen, BLACK, pos_rtf,int(6/res))
        pygame.draw.circle(screen, BLACK, pos_rtr,int(6/res))        

        # フォトリフレクタ描画
        for idx in range(NUM_PHOTOREFS):
            pos = center + np.asarray(self._mntposprs[idx])/res
            pos = (rotate_pos(pos,center,angle)+.5).astype(np.int32).tolist()
            if LFPhotoReflector.ACTIVE_WHITE:
                red = (int(self._prs[idx].value*255.0), 0, 0)
            else:
                red = (int((1.0-self._prs[idx].value)*255.0), 0, 0)
            pygame.draw.circle(screen, red, pos, 4)

    def get_rect_px(self):
        # 車体の四隅の座標
        res = self._course.resolution # mm/pixel
        pos_cx_mm = self._x_mm # pixel
        pos_cy_mm = self._y_mm # pixel
        car_width_mm = 2*SHAFT_LENGTH # 車体幅 in mm
        car_length_mm = car_width_mm+60 # 車体長 in mm
        pos_topleft_x_px = int((pos_cx_mm-2.5*SHAFT_LENGTH)/res+.5)
        pos_topleft_y_px = int((pos_cy_mm-0.7*SHAFT_LENGTH)/res+.5)
        car_width_px = (car_width_mm-0.6*SHAFT_LENGTH)/res # 車体幅 in pixel  
        car_length_px = (car_length_mm-0.6*SHAFT_LENGTH)/res # 車体長 in pixel    
        rect = [pos_topleft_x_px,pos_topleft_y_px, car_length_px, car_width_px]
        return rect

    def get_center_px(self):
        # 車体の回転の中心（シャフトの中心）
        res = self._course.resolution # mm/pixel        
        cx_mm = self._x_mm # pixel
        cy_mm = self._y_mm # pixel
        cx_px = cx_mm/res
        cy_px = cy_mm/res
        return [cx_px,cy_px]

    def _sense(self):
        """ フォトリフレクタの位置情報の更新 """

        # 解像度
        res = self._course.resolution # mm/pixel                
        # 車体の位置と向き
        center_px = self.get_center_px()
        angle = self._angle_rad

        # フォトリフレクタ位置を設定
        for idx in range(NUM_PHOTOREFS):
            pos = center_px + np.asarray(self._mntposprs[idx])/res
            self._prs[idx].pos_px = rotate_pos(pos,center_px,angle)
