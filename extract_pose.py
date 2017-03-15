import rosbag
import numpy as np
import tf
from cv_bridge import CvBridge
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy
import time
from copy import  deepcopy

class rospyExt(object):
    def __init__(self):
        self.path_pub = rospy.Publisher("/uav_pose/aPath", Path, queue_size=100)
        self.path_pub2 = rospy.Publisher("/uav_pose/mPath", Path, queue_size=100)
        self.pose_pub = rospy.Publisher("/uav_pose/aPose", PoseStamped, queue_size=100)

    def ext_marker_image(self,bag_name,image_topic_name,index):
        bridge = CvBridge()
        for topics, smsg, t in bag_name.read_messages(topics=[image_topic_name]):
            if t == index:
                return bridge.imgmsg_to_cv2(smsg, "bgr8")

    def ext_point_pose(self,bag_name,pose_topics_name):
        point = np.empty([1,3])
        for topics,pose_msg, t in bag_name.read_messages(topics=[pose_topics_name]):
            point = np.append(point,
                                 np.array([[pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z]]),
                                 axis=0)
        return  point

    def ext_rpy_pose(slelf,bag_name,pose_topics_name):
        rpy = np.empty([1, 3])

        for topics, pose_msg, t in bag_name.read_messages(topics=[pose_topics_name]):
            rpy = np.append(rpy,
                                  np.array([tf.transformations.euler_from_quaternion((pose_msg.pose.orientation.x, \
                                                                                      pose_msg.pose.orientation.y, \
                                                                                      pose_msg.pose.orientation.z, \
                                                                                      pose_msg.pose.orientation.w))]),
                                  axis=0)
        return  rpy

    def add_pose_path(self,bag_name,pose_topics_name):
        rospy.init_node('aptah')
        mav_path = Path()
        mav_path2 = Path()
        mav_pose = PoseStamped()
        mav_path.header.frame_id = 'camera'
        mav_pose.header.frame_id = 'camera'
        mav_path2.header.frame_id = 'camera'
        for topics, pose_msg, t in bag_name.read_messages(topics=[pose_topics_name]):
            mav_path.poses.append(pose_msg)
            mav_pose = deepcopy(pose_msg)
            mav_pose.header.frame_id = 'camera'
            idx = pose_msg.header.stamp.nsecs
            mav_pose.pose.position.x += 0.01*np.sin(idx/1000000.0)
            mav_pose.pose.position.x += 0.01*np.sin(idx/1000000.0)
            mav_pose.pose.position.x += 0.05*np.sin(idx/1000000.0)
            mav_path2.poses.append(mav_pose)
            self.pose_pub.publish(mav_pose)
            self.path_pub.publish(mav_path)
            self.path_pub2.publish(mav_path2)
            time.sleep(.100)

if __name__=='__main__':
    rospy_ext = rospyExt()
    bagf = rosbag.Bag('/media/bobin/DATA1/UAV/project611/data/iacas/2016-12/pose_2016-12-20-15-11-52.bag')
    #img = rospy_ext.ext_marker_image(bagf,'/camera/markedImage',100)
    #pt = rospy_ext.ext_point_pose(bagf,'/mavros/local_position/pose')
    rospy_ext.add_pose_path(bagf,'/mavros/local_position/pose')

