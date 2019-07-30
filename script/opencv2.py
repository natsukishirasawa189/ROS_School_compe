#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import roslib
roslib.load_manifest('lecture_pkg')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# 追跡対象の色範囲（Hueの値域）
def is_target(roi): 
    return (roi <= 30) | (roi >= 180)

class image_converter:

  def __init__(self):
    # パブリッシャーの作成
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    # カメラのサブスクライブ
    self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback)

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
    max_index = np.argmax(data[:,4])

    # 面積最大ブロブの各種情報を表示
    # print("外接矩形の左上x座標", data[:,0][max_index])
    # print("外接矩形の左上y座標", data[:,1][max_index])
    # print("外接矩形の幅", data[:,2][max_index])
    # print("外接矩形の高さ", data[:,3][max_index])
    # print("面積", data[:,4][max_index])
    # print("中心座標:\n",center[max_index])

    x1 = data[:,0][max_index]
    y1 = data[:,1][max_index]
    x2 = data[:,0][max_index]+data[:,2][max_index]
    y2 = data[:,1][max_index]+data[:,3][max_index]
    area = data[:,4][max_index]

    # 短形とテキストの書き込み
    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0))
    cv2.putText(cv_image, str(area), (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 
                1.0, (0, 0, 255), thickness=2)
    
    # 描画
    cv2.imshow("Image window", cv_image)
    cv2.imshow("mask", result)
    cv2.waitKey(3)

    try:
      # パブリッシュ
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  # 画像処理を実行
  ic = image_converter()
  # ノードの立ち上げ
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
