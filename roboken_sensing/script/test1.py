#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge

def video_to_msg(video_path, topic_name):
    rospy.init_node('video_to_msg_node', anonymous=True)
    rate = rospy.Rate(30)  # フレームレートを設定

    cap = cv2.VideoCapture(video_path)
    bridge = CvBridge()

    pub = rospy.Publisher(topic_name, Image, queue_size=100)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.loginfo('break')
            break


        # OpenCVの画像データをROSのImageメッセージに変換
        image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")

        # メッセージをパブリッシュ
        pub.publish(image_msg)
        print('hello')

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    video_path = '/mnt/c/20231121_013646000_iOS.mp4'  # 動画ファイルのパス
    topic_name = 'camera'  # 使用するROSトピックの名前
    video_to_msg(video_path, topic_name)