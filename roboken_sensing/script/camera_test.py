#!/usr/bin/env python
# -*- coding: utf-8 -*-
from logging import captureWarnings
import cv2
import numpy as np
import rospy 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from roboken_sensing.msg import target
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray



class cameraNode():
    def __init__(self):
        self.sub = rospy.Subscriber('camera/color/image_raw', Image, self.callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.callback)
        self.camera_info_subscriber = rospy.Subscriber("/camera_info",CameraInfo, self.data_K)
        self.pub = rospy.Publisher('target_point', target, queue_size=10)
        self.c_x = 0
        self.c_y = 0
        self.f_x = 0
        self.f_y = 0 
    def data_K(self,camera_info):

        self.c_x = camera_info.K[0,2]
        self.c_y = camera_info.K[1,2]
        self.f_x = camera_info.K[0,0]
        self.f_y = camera_info.K[1,1] 

    def callback(self,image_msg,):
        rospy.loginfo("callback start")
        bridge = CvBridge()
        
        try:
            # ROSのImageメッセージをOpenCVの画像に変換
            img_depth = bridge.imgmsg_to_cv2(image_msg,"passthrough")
            img_hsv = bridge.imgmsg_to_cv2(image_msg, "bgr8")
            rospy.loginfo("debug1")
        except CvBridgeError as e:
            rospy.loginfo("error")
            rospy.logerr(e)
            return
        
        
            
        max_index_B = 0
        max_index_R = 0
        max_index_Y = 0 
        c_x2 = self.c_x
        c_y2 = self.c_y
        f_x2 = self.f_x
        f_y2 = self.f_y
        hsv = cv2.cvtColor(img_hsv, cv2.COLOR_BGR2HSV)
        cv2.imshow('frame1', hsv)
       
        lower_B = np.array([105,80,40])
        upper_B = np.array([130,255,255])
            
        frame_mask_B = cv2.inRange(hsv, lower_B, upper_B)
            
            
            
            
        dst_B = cv2.bitwise_and(img_hsv, img_hsv, mask=frame_mask_B)
            
        #     cv2.imshow("dst", dst)
            
        dst_g_B = cv2.cvtColor(dst_B, cv2.COLOR_BGR2GRAY)
        
        nlabels_B, labels_B, stats_B, center_B = cv2.connectedComponentsWithStats(dst_g_B)
        center_B = np.delete(center_B, 0, 0)
        stats_B = np.delete(stats_B, 0, 0)
        n_B = nlabels_B - 1
        if n_B != 0:
            max_index_B = np.argmax(stats_B[:,4])
            x, y = center_B[max_index_B]
            cv2.drawMarker(dst_B, (int(x), int(y)), (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, thickness=2)
            cv2.putText(dst_B,text='2',org=(int(x+50), int(y)),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.0,color=(0, 255, 0),thickness=2,lineType=cv2.LINE_4)
                
        #    circles = cv2.HoughCircles(hsv_g, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20, param1=100, param2=50, minRadius=0)
        #     print(circles)
        lower_R = np.array([0,150,100])
        upper_R = np.array([10,255,255])
        
        frame_mask1_R = cv2.inRange(hsv, lower_R, upper_R)
        
        lower_R = np.array([165,150,100])
        upper_R = np.array([179,255,255])
        
        frame_mask2_R= cv2.inRange(hsv, lower_R, upper_R)
        
        frame_mask_R = frame_mask1_R + frame_mask2_R
        
        dst_R = cv2.bitwise_and(img_hsv, img_hsv, mask=frame_mask_R)
        dst_g_R = cv2.cvtColor(dst_R, cv2.COLOR_BGR2GRAY)
        nlabels_R, labels_R, stats_R, center_R = cv2.connectedComponentsWithStats(dst_g_R)
        center_R = np.delete(center_R, 0, 0)
        stats_R = np.delete(stats_R, 0, 0)
        n_R = nlabels_R - 1
        if n_R != 0:
            max_index_R = np.argmax(stats_R[:,4])
            x, y = center_R[max_index_R]
            cv2.drawMarker(dst_R, (int(x), int(y)), (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, thickness=2)
            cv2.putText(dst_R,text='2',org=(int(x+50), int(y)),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.0,color=(0, 255, 0),thickness=2,lineType=cv2.LINE_4)

        lower_Y = np.array([20,100,40])
        upper_Y = np.array([40,255,255])
        
        frame_mask_Y = cv2.inRange(hsv, lower_Y, upper_Y)
        dst_Y = cv2.bitwise_and(img_hsv, img_hsv, mask=frame_mask_Y)

        dst_g_Y = cv2.cvtColor(dst_Y, cv2.COLOR_BGR2GRAY)
    
        nlabels_Y, labels_Y, stats_Y, center_Y = cv2.connectedComponentsWithStats(dst_g_Y)
        center_Y = np.delete(center_Y, 0, 0)
        stats_Y = np.delete(stats_Y, 0, 0)
        n_Y = nlabels_Y - 1
        if n_Y != 0:
            max_index_Y = np.argmax(stats_Y[:,4])
            x, y = center_Y[max_index_Y]
            cv2.drawMarker(dst_Y, (int(x), int(y)), (0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, thickness=2)
            cv2.putText(dst_Y,text='3',org=(int(x+50), int(y)),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.0,color=(0, 255, 0),thickness=2,lineType=cv2.LINE_4)    
        cv2.imshow('frame1', dst_B)
        cv2.imshow('frame2', dst_R)
        cv2.imshow('frame3', dst_Y)

        if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.is_shutdown()
        target_data = target()
        target_data.target_color = 3
        target_data.center_Y = Float32MultiArray()
        target_data.center_B = Float32MultiArray()
        target_data.center_R = Float32MultiArray()
        x_s = 0
        y_s = 0
        mydict = {"0":-1}
        
        if len(stats_B)>0:        
            if stats_B[max_index_B,4]>100:
                mydict["2"]=stats_B[max_index_B,4]
                x_s = stats_B[max_index_B,2]
                rospy.logwarn(type(x_s))
                y_s = stats_B[max_index_B,3]
                rospy.logwarn(type(y_s))

                z_B = img_depth[x_s,y_s]
                rospy.logwarn(type(z_B))
                x_w = (x_s-c_x2)*z_B/f_x2
                rospy.logwarn(type(x_w))
                y_w = (y_s-c_y2)*z_B/f_y2
                rospy.logwarn(type(y_w))
                array_B =[x_w,y_w,z_B]

                #log show img_depth[x_s,y_s]
                rospy.logwarn(img_depth[x_s,y_s])
                # rospy.loginfo(len(array_B))
                # array_forPublish_B = Float32MultiArray(data=array_B)
                # rospy.loginfo(len(array_forPublish_B.data))
                # rospy.loginfo(array_forPublish_B)
                # target_data.center_B = array_forPublish_B
                target_data.center_B = array_B

        if len(stats_R>0):
            if stats_R[max_index_R,4]>100:
            	mydict["1"] = stats_R[max_index_R,4]
                x_s = stats_R[max_index_R,2]
                y_s = stats_R[max_index_R,3]
                z_R = img_depth[x_s,y_s]
                x_w = (x_s-c_x2)*z_R/f_x2
                y_w = (y_s-c_y2)*z_R/f_y2
                array_R =[x_w,y_w,z_R]
                array_forPublish_R = Float32MultiArray(data=array_R)
                target_data.center_R = array_forPublish_R
        
        if len(stats_Y)>0:
            if stats_Y[max_index_Y,4]>100:
            	mydict["3"] = stats_Y[max_index_Y,4]
                x_s = stats_Y[max_index_Y,2]
                y_s = stats_Y[max_index_Y,3]
                z_Y = img_depth[x_s,y_s]
                x_w = (x_s-c_x2)*z_Y/f_x2
                y_w = (y_s-c_y2)*z_Y/f_y2
                array_Y =[x_w,y_w,z_Y]
                array_forPublish_Y = Float32MultiArray(data=array_Y)
                target_data.center_Y = array_forPublish_Y
                
        if max(mydict) >-1:
            target_data.target_color = int(max(mydict))
            self.target_pub(target_data)
            #self.publish(target_msg)
        else:
            target_data.target_color = 3
            self.target_pub(target_data)
            #self.publish(target_msg)
   

    def target_pub(self,data):
        rospy.loginfo('debag2')
        self.pub.publish(data)
        rospy.loginfo(data.target_color)
        rospy.loginfo(data.center_Y)
        r = rospy.Rate(10) 
        rospy.loginfo('receive')   

if __name__ == '__main__':
    rospy.init_node('camera_Node')
    node = cameraNode()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)



