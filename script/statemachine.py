#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import rospy
import smach
import smach_ros
import signal
import roslib
roslib.load_manifest('lecture_pkg')
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
# 数学ライブラリ
import math
# タイマーのインポート
import time
# 今回主に使うROSメッセージ型をインポート
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
# 幾何学変換のための関数
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult

flag1 = 0
flag2 = 0
flag3 = 0

class image_converter:
    def __del__(self):
        cv2.destroyAllWindows()
        self.image_sub.unregister()

    def __init__(self):
        # パブリッシャーの作成
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        # カメラのサブスクライブ
        self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)
        # 変数初期化
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
        try:
            # カメラのimageを受け取る
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
             print(e)
    
        # マスク画像の作成
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h = hsv[:,:,0]
        s = hsv[:,:,1]
        mask = np.zeros(h.shape, dtype=np.uint8)
        mask[((h<20)|(h>200)) & (s > 150)] = 255

        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)    

        #　ラベリング処理
        label = cv2.connectedComponentsWithStats(mask)

        # ブロブ情報を項目別に抽出
        n = label[0] - 1
        data = np.delete(label[2], 0, 0)
        center = np.delete(label[3], 0, 0)  

        # ブロブ面積最大のインデックス
        if data.any():
            max_index = np.argmax(data[:,4])

            self.x1 = data[:,0][max_index]
            self.y1 = data[:,1][max_index]
            self.x2 = data[:,0][max_index]+data[:,2][max_index]
            self.y2 = data[:,1][max_index]+data[:,3][max_index]
            self.area = data[:,4][max_index]
            self.cx = center[:,0][max_index]
            self.cy = center[:,1][max_index]

            # 短形とテキストの書き込み
            cv2.rectangle(cv_image, (self.x1, self.y1), (self.x2, self.y2), (255, 0, 0))
            cv2.putText(cv_image, str(self.area), (self.x1, self.y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                        1.0, (0, 0, 255), thickness=2)

            # Wait処理
            if self.area < 3000:
                global flag1
                flag1 = 1
            
            # Followの処理
            _, w, _ = cv_image.shape
            global rot
            global vel
            rot = 0
            vel = 0
            if self.cx > w/2.0 + 50:
                rot = 3.14*(w/2.0 - self.cx)/(w/5.0)
            elif self.cx < w/2.0 - 50:
                rot = 3.14*(self.cx - w/2.0)/(w/5.0)
            else:
                rot = 0

            if self.area > 180000:
                vel = 0
            elif self.area < 50000:
                vel = 0
            else:
                vel = (180000 - self.area)/(180000.0/1.0)
            
            print(self.cx)
            print("vel=" + str(vel))
            print("rot=" + str(rot))

        # 描画
        cv2.imshow("Image window", cv_image)
        cv2.imshow("mask", result)
        cv2.waitKey(3)

# define state Wait
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Wait')
        global flag1
        while flag1 == 0:
            rospy.sleep(0.1)
        return 'outcome1'

# define state GoStartPoint
class GoStartPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])
        # ゴール配信者オブジェクトを生成
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', # トピック名
                                        PoseStamped,              # 型
                                        queue_size=1,             # 送信キューのサイズ
                                        latch=True)               # データを次の更新まで保持する
        # 現在位置を読み込むオブジェクトを生成
        self.pose_sub = rospy.Subscriber('/amcl_pose/pose/pose/postion/x',   # トピック名
                                         PoseWithCovarianceStamped)          # 型

        self.result_sub = rospy.Subscriber('/move_base/result',   # トピック名
                                           MoveBaseActionResult,  # 型
                                           self.callback )       # コールバック
    def callback(self,data):
        global flag2
        flag2 = 1
        self.result_sub.unregister()

    def execute(self, userdata):
        rospy.loginfo('Executing state GoStartPoint')
        self.goal = PoseStamped()
        self.goal.header.frame_id = 'map'        # 世界座標系で指定する
        self.goal.header.stamp = rospy.Time.now()  # タイムスタンプは今の時間
        self.goal.pose.position.x = -3.0
        self.goal.pose.position.y = 0.0
        self.goal.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, math.radians(90))
        self.goal.pose.orientation = Quaternion(*q)
        #self.goal_pub.publish(self.goal)  # 実際にメッセージを配信
        amcl_pose = PoseWithCovarianceStamped()
        rospy.loginfo(self.goal)

        #global flag2
        #while flag2 == 0:
        #    rospy.sleep(0.1)
        return 'outcome2'

# define state Follow
class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])
        # ゴール配信者オブジェクトを生成
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', # トピック名
                                        PoseStamped,              # 型
                                        queue_size=1,             # 送信キューのサイズ
                                        latch=True)               # データを次の更新まで保持する
        # 現在位置を読み込むオブジェクトを生成
        self.pose_sub = rospy.Subscriber('/amcl_pose/pose/pose/postion/x',   # トピック名
                                         PoseWithCovarianceStamped)          # 型

    def execute(self, userdata):
        rospy.loginfo('Executing state Follow')

        global rot
        global vel

        global flag3
        while flag3 == 0:
            self.goal = PoseStamped()
            self.goal.header.frame_id = 'base_link'        # 世界座標系で指定する
            self.goal.header.stamp = rospy.Time.now()  # タイムスタンプは今の時間
            self.goal.pose.position.x = vel
            self.goal.pose.position.y = 0.0
            self.goal.pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, rot)
            self.goal.pose.orientation = Quaternion(*q)
            self.goal_pub.publish(self.goal)  # 実際にメッセージを配信
            amcl_pose = PoseWithCovarianceStamped()
            rospy.loginfo(self.goal)
            rospy.sleep(0.1)
            
        rospy.signal_shutdown('finish')
        return 'outcome3'

# main
def main():
    rospy.init_node('state_machine', disable_signals = True)

    # Start image_converter()
    ic = image_converter()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Wait', Wait(), 
                               transitions={'outcome1':'GoStartPoint'})
        smach.StateMachine.add('GoStartPoint', GoStartPoint(), 
                               transitions={'outcome2':'Follow'})
        smach.StateMachine.add('Follow', Follow(), 
                               transitions={'outcome3':'outcome4'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    del ic

# handler for signal
def handler(signal, frame):
    global cont
    cont = False

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    main()

