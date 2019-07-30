#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import rospy
import smach
import smach_ros

import roslib
roslib.load_manifest('lecture_pkg')
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# define state Wait
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        # パブリッシャーの作成
        self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.bridge = CvBridge()
        # カメラのサブスクライブ
        self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)
        self.flag = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Wait')
        if self.flag == 0:
            return 'outcome1'
        else:
            cv2.destroyAllWindows()
            self.image_sub.unregister()
            return 'outcome2'

    # 追跡対象の色範囲（Hueの値域）
    def is_target(roi): 
        return (roi <= 30) | (roi >= 180)

    def callback(self,data):
        rospy.loginfo('Callback')
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
        max_index = np.argmax(data[:,4])

        x1 = data[:,0][max_index]
        y1 = data[:,1][max_index]
        x2 = data[:,0][max_index]+data[:,2][max_index]
        y2 = data[:,1][max_index]+data[:,3][max_index]
        area = data[:,4][max_index]

        # 短形とテキストの書き込み
        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0))
        cv2.putText(cv_image, str(area), (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                1.0, (0, 0, 255), thickness=2)
        
        if area < 3000:
            self.flag = 1

        # 描画
        cv2.imshow("Image window", cv_image)
        cv2.imshow("mask", result)
        cv2.waitKey(3)


# define state GoStartPoint
class GoStartPoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GoStartPoint')
        # ここにナビゲーションの処理を書く
        rospy.sleep(1)
        return 'outcome3'

# define state Follow
class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome4'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Follow')
        # ここにFollowのプログラムを書く
        rospy.sleep(1)
        return 'outcome4'

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Wait', Wait(), 
                               transitions={'outcome1':'Wait', 
                                            'outcome2':'GoStartPoint'})
        smach.StateMachine.add('GoStartPoint', GoStartPoint(), 
                               transitions={'outcome3':'Follow'})
        smach.StateMachine.add('Follow', Follow(), 
                               transitions={'outcome4':'outcome5'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()

