#!/usr/bin/python3
# coding: UTF-8
"""
ライントレーサー物理モデル（車体の重心と車輪間の中心がずれたモデル）

説明

　物理モデルの変更については以下のパラメータおよび mtrs2twist() メソッドを編集してください。
  
  - LF_MOUNT_POS_PRF # フォトリフレクタの配置
  - LF_WEIGHT        # 車体の重さ g
  - SHAFT_LENGTH     # シャフト長 mm (１本あたり)
  - TIRE_DIAMETER    # タイヤ直径 mm

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
LF_MOUNT_POS_PRF = ((30,-60), (10,-20), (10,20), (30,60)) # mm
LF_WEIGHT = 360    # 車体の重さ g（グラム）
SHAFT_LENGTH = 50  # シャフト長 mm （１本あたり）
TIRE_DIAMETER = 58 # タイヤ直径 mm

# 制御パラメータ（要調整）
COEF_K_P = 3.0 # 比例制御係数

# 物理モデルパラメータ（要調整）
PARAMS_MU_C = 1e-3 # kg/s 車体の直線運動の粘性摩擦係数（便宜上）
PARAMS_IOTA_C = 1e-4 # kg·m^2/s 車体の旋廻運動の粘性摩擦係数（便宜上）
PARAMS_ELL_C = -30e-3 # m 車体の重心から車輪間の中心までの距離 
PARAMS_N = 58.2 # ギヤ比
PARAMS_L_C = 2*SHAFT_LENGTH*1e-3 # m 車輪間の距離
PARAMS_R_W = (1/2)*TIRE_DIAMETER*1e-3 # m 車輪の半径　
PARAMS_K_T = 2e-3 # N·m/A モータのトルク係数（概算）
PARAMS_K_B = 2e-3 # V·s/rad モータの逆起電力定数（概算）
PARAMS_R_A = 2.0  # Ω 電機子抵抗（概算）
PARAMS_J_C = (1/3)*LF_WEIGHT*1e-3*((160e-3/2)**2+(60e-3/2)**2) # kg·m^2 車体の重心周りの慣性モーメント（概算）
PARAMS_J_M = (1/2)*(5e-3)*((5e-3)**2) # kg·m^2 電機子の慣性モーメント（概算）
#PARAMS_J_W = (3/4)*(21e-3)*((25e-3)**2+(4e-3)**2) # kg·m^2 車輪の慣性モーメント（概算）
PARAMS_J_G = 0 # kg·m^2 ギヤの慣性モーメント (ギヤ比 n>>1 より近似)
PARAMS_IOTA_M = 1e-6 # kg·m^2/s モータの粘性摩擦係数（概算）
#PARAMS_IOTA_W = 0 # kg·m^2/s 車輪の粘性摩擦係数 (ギヤ比 n>>1 より近似)
PARAMS_IOTA_G = 0 # kg·m^2/s ギヤの粘性摩擦係数 (ギヤ比 n>>1 より近似)

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
        # サンプリング間隔
        h = 1/fps

        # モーター電圧から速度・角速度の計算
        u_r = mtrs[1]
        u_l = mtrs[0]
        u_lin = (u_r + u_l)/2.0 # 直線運動
        u_rot = (u_r - u_l)/2.0 # 回転運動

        # 速度・角速度計算（微分方程式の数値解）
        t = np.linspace(0,h,2)
        y0 = [ 0.0 for idx in range(2) ]
        y0[0] = v0
        y0[1] = w0
        y1 = odeint(self._odedydt,y0,t,args=(u_lin,u_rot))
        v1 = y1[-1][0]
        w1 = y1[-1][1]

        # 出力
        twist = { "linear":{"x":v1, "y":0., "z":0.}, "angular":{"x":0., "y":0., "z":w1} }
        return twist

    def _odedydt(self,y,t,u_lin,u_rot):
        """ 動力学(Dynamic)モデルの状態方程式 """
        v = y[0]
        w = y[1]

        # 車体重量の換算
        mc_kg = 1e-3*self._weight # g -> kg

        # 各種パラメータ
        n = PARAMS_N
        r_w = PARAMS_R_W
        k_t = PARAMS_K_T
        k_b = PARAMS_K_B
        R_a = PARAMS_R_A
        J_m = PARAMS_J_M
        J_g = PARAMS_J_G       
        iota_m = PARAMS_IOTA_M
        iota_g = PARAMS_IOTA_G
        # 
        iota_bar_m = iota_m + k_t*k_b/R_a
        iota_bar_g = iota_g + (n**2)*iota_bar_m
        J_bar_g = J_g + (n**2)*J_m
        zeta = n*k_t/R_a 

        # 直線速度の計算
        mu_c = PARAMS_MU_C  # 直線運動の粘性摩擦係数
        #
        D_lin = iota_bar_g + (1/2)*mu_c*(r_w**2) 
        J_lin = J_bar_g + (1/2)*mc_kg*(r_w**2)
        T_lin = J_lin / D_lin # 時定数
        K_lin = COEF_K_P*(zeta*r_w) / D_lin
        N_lin = (1/2)*mc_kg*(r_w**2)

        # 回転速度の計算
        iota_c = PARAMS_IOTA_C # 回転運動の粘性摩擦係数
        l_c = PARAMS_ELL_C
        L_c = PARAMS_L_C
        J_c = PARAMS_J_C
        r_c = (2*r_w/L_c)
        #
        D_rot = iota_bar_g + (1/2)*(mu_c*(l_c**2)+iota_c )*(r_c**2)
        J_rot = J_bar_g + (1/2)*(J_c + mc_kg*(l_c**2))*(r_c**2)
        T_rot = J_rot / D_rot # 時定数
        K_rot = COEF_K_P*(zeta*r_c) / D_rot
        N_rot = (1/2)*mc_kg*(r_c**2)

        # dy/dt = f(y,t,v,w)
        dydt0 = (1/T_lin) * ( -l_c*(N_lin/D_lin)*(-w*w) - v + K_lin*u_lin )
        dydt1 = (1/T_rot) * ( -l_c*(N_rot/D_rot)*( v*w) - w + K_rot*u_rot )

        return [ dydt0, dydt1 ]

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

        # 位置・角度情報更新（微分方程式の数値解）
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
        """ 運動(Kinematic)モデルの状態方程式 """
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
