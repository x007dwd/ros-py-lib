import numpy as np
import tf
import visualization_msgs

class rospyExtMarker(object):


    def ext_point_marker(self,bag_name,marker_topics_name):
        point = np.empty([1,3])
        for topics,marker_msg, t in bag_name.read_messages(topics=[marker_topics_name]):
            point = np.append(point,
                                 np.array([[marker_msg.pose.position.x, marker_msg.pose.position.y, marker_msg.pose.position.z]]),
                                 axis=0)
        return  point

    def ext_rpy_marker(slelf, bag_name, marker_topic_name):
        rpy = np.empty([1, 3])

        for topics, marker_msg, t in bag_name.read_messages(topics=[marker_topic_name]):
            rpy = np.append(rpy,
                                  np.array([tf.transformations.euler_from_quaternion((marker_msg.pose.orientation.x, \
                                                                                      marker_msg.pose.orientation.y, \
                                                                                      marker_msg.pose.orientation.z, \
                                                                                      marker_msg.pose.orientation.w))]),
                                  axis=0)
        return  rpy



