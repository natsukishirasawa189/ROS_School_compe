#!/usr/bin/env python
# -*- coding: utf-8 -*-

# print用
from __future__ import print_function
#　ROSの基本ライブラリ
import rospy
# マニフェストにあるROS用のライブラリを読み込む
import roslib
roslib.load_manifest('lecture_pkg')
#　ステートマシン用ライブラリ
import smach
import smach_ros
# プログラムの終了等の管理
import signal
import sys
# opencv用
import cv2
# 行列を扱う
import numpy as np
# 数学ライブラリ
import math
# タイマーのインポート
import time
# 今回主に使うROSメッセージ型をインポート
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image # 画像
from cv_bridge import CvBridge, CvBridgeError #　Imageメッセージをopencvの画像へコンバート
# 幾何学変換のための関数
from tf.transformations import quaternion_from_euler #オイラー角からクォータニオンへ変換
from move_base_msgs.msg import MoveBaseActionResult # ナビゲーションの終了を受け取る

cont = True
flag1 = 0
flag2 = 0
flag3 = 0
timeout = 0

#　画像処理
class image_converter:
    # 終了処理　デストラクタ
    def __del__(self):
        cv2.destroyAllWindows()
        self.image_sub.unregister()

    # 初期化　コンストラクタ
    def __init__(self):
        # パブリッシャーの作成
        # self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        # カメラのサブスクライブ callback:image_rawにデータが来たら実行される関数
        self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)
        # メンバ初期化
        self.x1 = 0
        self.y1 = 0
        self.x2 = 0
        self.y2 = 0
        self.area = 0
        self.cx = 0
        self.cy = 0

    # 追跡対象の色範囲（Hueの値域）
    def is_target(roi): 
        return (roi <= 30) | (roi >= 180)

    def callback(self,data):
        # 例外処理
        try:
            # カメラのimageを受け取る
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
        # 受け取った画像をhsvに変換 h：彩度　s:明度　v:輝度
        # cvtColor(画像,形式to形式) 画像の形式を変更する関数　他にもグレースケールにするものなどがある
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # hsv(x,y,channel) chnnle:(b,g,r)
        h = hsv[:,:,0]
        s = hsv[:,:,1]
        # 受け取った画像と同サイズのマスク画像を初期化 h.shape = (width,hight) dtype:uint8
        mask = np.zeros(h.shape, dtype=np.uint8)
        # マスク画像の作成　条件に合うインデックスを黒(255)にする
        # 彩度が20より小さく200より大きい,輝度が150より大きい
        # mask[is_target(h) & (s > 150)] = 255
        mask[((h<20)|(h>200)) & (s > 150)] = 255
        
        # マスク処理 bitwise_and:maskと入力画像を合成（and）
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)    

        #　ラベリング処理
        label = cv2.connectedComponentsWithStats(mask)

        # ブロブ情報を項目別に抽出
        # ブロブ番号　-1しているのは引数として使うため
        n = label[0] - 1
        # data 左上、右下の座標、大きさ等
        data = np.delete(label[2], 0, 0)
        # center 中心座標
        center = np.delete(label[3], 0, 0)  

        # ブロブ面積最大のインデックス
        # dataが存在すれば実行
        if data.any():
            # argmax配列からもっとも大きい値を取ってくる
            max_index = np.argmax(data[:,4])
            # 左上x:data[:,0],左上y:data[:,1]
            # 横幅:data[:,2],高さ:data[:,3]
            self.x1 = data[:,0][max_index]
            self.y1 = data[:,1][max_index]
            self.x2 = data[:,0][max_index]+data[:,2][max_index]
            self.y2 = data[:,1][max_index]+data[:,3][max_index]
            # 面積:data[:,4]
            self.area = data[:,4][max_index]
            # 中心座標x,y
            self.cx = center[:,0][max_index]
            self.cy = center[:,1][max_index]

            # 短形とテキストの書き込み
            # 短形　rectangle(書き込む画像、左上座標、右下座標、カラー)　カラー:(b,g,r)
            cv2.rectangle(cv_image, (self.x1, self.y1), (self.x2, self.y2), (255, 0, 0))
            # テキスト　putText(書き込む画像、テキスト、左上座標、フォント、倍率、カラー,線の太さ)　カラー:(b,g,r)
            cv2.putText(cv_image, str(self.area), (self.x1, self.y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                        1.0, (0, 0, 255), thickness=2)

            # Wait処理
            # 赤が視界から消えたらフラグ立ち上げ
            if self.area < 3000:
                global flag1
                flag1 = 1
            
            # Followの処理
            _, w, _ = cv_image.shape
            global rot # 回転
            global vel # 速度
            global timeout
            rot = 0
            vel = 0
            if self.cx > w/2.0 + 50:		 	# 対象の中心がカメラの中心より右
                rot = 3.14*(self.cx - w/2.0)/(w*2)
            elif self.cx < w/2.0 - 50: 			# 対象の中心がカメラの中心より左
                rot = 3.14*(self.cx - w/2.0)/(w*2)
            else: 					# 対象の中心がカメラの中心なら回転しない
                rot = 0

            if self.area > 180000:			# 対象の大きさが一定以上ならストップ
                vel = 0
                rot = 0
                timeout = timeout + 1
            elif self.area < 50000:			# 対象の大きさが一定以下なら追わない
                vel = 0
                rot = 0
                timeout = timeout + 1
            else:					# 対象の大きさに反比例する
                vel = 180000.0/(self.area*20.0)
                timeout = 0

            print(self.cx)
            print("time=" + str(timeout))
            print("vel=" + str(vel))
            print("rot=" + str(rot))

        # 描画
        # imshow(タイトル、画像)
        cv2.imshow("Image window", cv_image)
        cv2.imshow("mask", result)
        # waitKey：キー入力を待つ　今回は画像の表示が安定するように記述
        cv2.waitKey(3)

# define state Wait
class Wait(smach.State):
    # 初期化処理
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
    
    # 実行処理
    def execute(self, userdata):
        rospy.loginfo('Executing state Wait')
        # flag1:赤い物体があるとき1
        # cont:Crt+cの検出
        global flag1, cont
        # 処理待ち
        while flag1 == 0 and cont:
            rospy.sleep(0.1)
        return 'outcome1'

# define state GoStartPoint
class GoStartPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])
        # ゴールのパブリッシャを生成
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', # トピック名
                                        PoseStamped,              # 型
                                        queue_size=1,             # 送信キューのサイズ
                                        latch=True)               # データを次の更新まで保持する
        # 現在位置をサブスクライブ
        self.pose_sub = rospy.Subscriber('/amcl_pose/pose/pose/postion/x',   # トピック名
                                         PoseWithCovarianceStamped)          # 型
        # ナビゲーション結果を受け取り
        self.result_sub = rospy.Subscriber('/move_base/result',   # トピック名
                                           MoveBaseActionResult,  # 型
                                           self.callback )       # コールバック
    def callback(self,data):
        global flag2
        # ナビゲーションが何らかの形で終了したらフラグを立てる
        flag2 = 1
        # サブスクライバの削除
        self.result_sub.unregister()

    def execute(self, userdata):
        rospy.loginfo('Executing state GoStartPoint')
        # ゴールの作成
        self.goal = PoseStamped()
        self.goal.header.frame_id = 'map'        # 世界座標系で指定する
        self.goal.header.stamp = rospy.Time.now()  # タイムスタンプは今の時間
        self.goal.pose.position.x = -3.0
        self.goal.pose.position.y = 0.0
        self.goal.pose.position.z = 0.0
        # オイラーからクォータニオンに変換
        q = quaternion_from_euler(0, 0, math.radians(90))
        self.goal.pose.orientation = Quaternion(*q)
        # パブリッシュ
        self.goal_pub.publish(self.goal)  # 実際にメッセージを配信
        amcl_pose = PoseWithCovarianceStamped()
        rospy.loginfo(self.goal)

        global flag2,cont
        # ナビゲーション待ち
        while flag2 == 0 and cont:
            rospy.sleep(0.1)
        return 'outcome2'

# define state Follow
class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])
        # テレオペ用
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # 現在位置を読み込むオブジェクトを生成
        self.pose_sub = rospy.Subscriber('/amcl_pose/pose/pose/postion/x',   # トピック名
                                         PoseWithCovarianceStamped)          # 型

    def execute(self, userdata):
        rospy.loginfo('Executing state Follow')

        # image_converter()から値を持ってくる
        global rot
        global vel
        global timeout
        global flag3,cont
        timeout = 0
        # timeoutするかCtr+cで止まる　flag3は結局使ってない
        while timeout < 20 and flag3 == 0 and cont:
            # テレオペ指示
            cmd = Twist()
            cmd.linear.x = vel
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = -rot
            cmd_pub.publish(cmd)
            amcl_pose = PoseWithCovarianceStamped()
            #rospy.loginfo(self.goal)
            rospy.sleep(0.1)
        
        # なぜかoutcome3に行っても処理が終わらなかったから終了処理を追加
        rospy.signal_shutdown('finish')
        return 'outcome3'

# main
def main():
    # ノードの立ち上げ (disable_signals = True Ctrl+cを受け付けなくなる)
    rospy.init_node('state_machine', disable_signals = True)

    global cmd_pub, ic
    # テレオペ用パブリッシャー
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # 画像処理の実行
    ic = image_converter()

    # ステートマシンの初期化
    sm = smach.StateMachine(outcomes=['outcome4'])

    # ステータスの定義
    with sm:
        # コンテナにステータスを追加
        smach.StateMachine.add('Wait', Wait(), 
                               transitions={'outcome1':'GoStartPoint'})
        smach.StateMachine.add('GoStartPoint', GoStartPoint(), 
                               transitions={'outcome2':'Follow'})
        smach.StateMachine.add('Follow', Follow(), 
                               transitions={'outcome3':'outcome4'})

    # ステートマシンのサーバーを作成・スタート（初期化）
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # ステートマシンの処理を実行
    outcome = sm.execute()

    # 処理が終了しないようにする
    rospy.spin()

    # ステートマシンの停止
    sis.stop()

# Ctrl+Cを受け取るhandler
def handler(signal, frame):
    global cont,cmd_pub,ic
    # 画像処理を止める
    del ic
    # ステートマシンの全ループを止める
    cont = False
    rate = rospy.Rate(10) # 10hz
    rate.sleep()
    print("shutdown")
    #goal = PoseStamped()
    #goal.header.frame_id = 'base_footprint'        # 世界座標系で指定する
    #goal.header.stamp = rospy.Time.now()  # タイムスタンプは今の時間
    #goal.pose.position.x = 0.0
    #goal.pose.position.y = 0.0
    #goal.pose.position.z = 0.0
    #q = quaternion_from_euler(0, 0, math.radians(90))
    #goal.pose.orientation = Quaternion(*q)
    
    # 停止コマンドを送る
    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.linear.y = 0.0
    cmd.linear.z = 0.0
    cmd.angular.x = 0.0
    cmd.angular.y = 0.0
    cmd.angular.z = 0.0
    cmd_pub.publish(cmd)
    #cmd_pub.publish(goal)  # 実際にメッセージを配信
    rate.sleep()
    #rospy.loginfo(cmd)

if __name__ == '__main__':
    # Ctrl+Cを受け取るhandlerを指定
    signal.signal(signal.SIGINT, handler)
    main()

