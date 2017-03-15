#!/usr/bin/env python
import rospy
from nav_msgs.msg import  Path
from geometry_msgs.msg import PoseStamped,TransformStamped
import numpy as np
from copy import deepcopy
import time
import tf
class path(object):
    def __init__(self):
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.add_pose_path)
        self.path_pub = rospy.Publisher('/uav_pose/aPath', Path, queue_size=10)
        self.path_pub2 = rospy.Publisher('/uav_pose/mPath', Path, queue_size=10)

        self.mav_path = Path()
        self.mav_path.header.frame_id = 'fcu'
        self.mav_path2 = Path()
        self.mav_path2.header.frame_id = 'fcu'
        self.tStart = -1

        self.br =tf.TransformBroadcaster()
        self.tfs = TransformStamped()

    def add_pose_path(self,pose):
        self.mav_path.poses.append(pose)
        self.path_pub.publish(self.mav_path)
        self.br.sendTransform((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z), \
                              (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                               pose.pose.orientation.w), \
                              rospy.Time.now(), \
                              "base_link", \
                              "fcu")
        # self.tfs.header.stamp = rospy.Time.now()
        # self.tfs.header.frame_id = 'base_link'
        # self.tfs.child_frame_id = 'fcu'
        # self.tfs.transform.translation.x = pose.pose.position.x
        # self.tfs.transform.translation.y = pose.pose.position.y
        # self.tfs.transform.translation.z = pose.pose.position.z
        # self.tfs.transform.rotation.w = pose.pose.orientation.x
        # self.tfs.transform.rotation.x = pose.pose.orientation.y
        # self.tfs.transform.rotation.y = pose.pose.orientation.z
        # self.tfs.transform.rotation.z = pose.pose.orientation.w
        self.br.sendTransformMessage(self.tfs)
        if self.tStart < 0:
            self.tStart = pose.header.stamp.secs
        elif pose.header.stamp.secs - self.tStart > 100 and pose.header.stamp.secs - self.tStart < 200:
            mav_pose = deepcopy(pose)
            idx = pose.header.stamp.nsecs
            mav_pose.pose.position.x += 0.01 * np.sin(idx / 1000000.0)
            mav_pose.pose.position.y += 0.01 * np.sin(idx / 1000000.0)
            mav_pose.pose.position.z += 0.05 * np.sin(idx / 1000000.0)
            self.mav_path2.poses.append(mav_pose)
            self.path_pub2.publish(self.mav_path2)


if __name__=='__main__':
    rospy.init_node('mav_path')
    mpath = path()
    rospy.spin()
