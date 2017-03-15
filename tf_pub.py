#!/usr/bin/env python

import rospy
import numpy as np
import tf
from  geometry_msgs.msg  import  TransformStamped



if __name__ == '__main__':
    rospy.init_node('drone_tf_broadcaster')
    #drone_name = rospy.get_param('dronename')
    br =tf.TransformBroadcaster()
    tfs = TransformStamped()
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        tfs.header.stamp = rospy.Time.now()
        tfs.header.frame_id = 'base_link'
        tfs.child_frame_id = 'world'
        idx = tfs.header.stamp.secs
        peroid = 1
        tfs.transform.translation.x = 100 * np.sin(idx/peroid)
        tfs.transform.translation.x = 100 * np.cos(idx/peroid)
        tfs.transform.translation.x = 100 * np.sin(idx/peroid)
        #tfs.transform.rotation.w
        br.sendTransformMessage(tfs)

        rate.sleep()
    rospy.spin()
