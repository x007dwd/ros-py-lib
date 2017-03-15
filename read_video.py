import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image
# We do not use cv_bridge it does not support CompressedImage in python
from cv_bridge import CvBridge, CvBridgeError


def main(args):
	'''Initializes and cleanup ros node'''
	tdelay = rospy.get_param('/publishImage/time_delay')
	pub = rospy.Publisher('/uav_pose/image', Image,queue_size=10)
	rospy.init_node('pulishCamera', anonymous=True)
	rate = rospy.Rate(33.243) # 10hz
	msg = Image()
	bridge = CvBridge()
	cap = cv2.VideoCapture(args[1])
	time.sleep(tdelay)
	while not rospy.is_shutdown():
		# hello_str = "hello world %s" % rospy.get_time()
		ret,image_np =  cap.read()
		if not ret:
			continue;
		# msg.header.stamp = rospy.Time.now()
		# msg.format = "jpeg"
		# msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()

		#rospy.loginfo(hello_str)
		pub.publish(bridge.cv2_to_imgmsg(image_np,'bgr8'))
		# cv2.imshow('a',image_np)
		# cv2.waitKey(10)
		rate.sleep()


if __name__ == '__main__':
	main(sys.argv)
