#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge

if __name__=='__main__':
    rospy.init_node('mav_path')
    videof = '/media/bobin/DATA1/UAV/project611/video/VID_20161220_151529.mp4'
    cap = cv2.VideoCapture(videof)

    while True:
        ret,frame = cap.read()
        if ret:
            cv2.imshow(frame)
            cv2.waitKey(10)
        else:
            break
    #rospy.spin()

