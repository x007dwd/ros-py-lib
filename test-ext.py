import rosbag
import matplotlib.pyplot as plt
import extract_pose as ep
import  extract_marker as em

from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def randrange(n, vmin, vmax):
    return (vmax - vmin)*np.random.rand(n) + vmin



def test_1():
    bagf = rosbag.Bag('/media/bobin/DATA1/SLAM/data/apriltag/tag-only-1.bag')
    ros_ext = em.rospyExtMarker()
    pt = ros_ext.ext_point_marker(bagf,'/visualization_marker')
    print  pt.shape
    plt.plot(pt[:,0],pt[:,1],'x')
    plt.figure()
    plt.plot(pt[:,0],pt[:,2],'x')
    plt.figure()
    plt.plot(pt[:,:2])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    n = 100
    ax.scatter(pt[:,0],pt[:,1],pt[:,2])

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    fig2 = plt.figure()

    ax2 = fig2.add_subplot(111,projection='3d')
    ax2.plot(pt[:,0],pt[:,1],pt[:,2])


    plt.show()

def test2():
    bagf = rosbag.Bag('/media/bobin/DATA1/SLAM/data/apriltag/tag-uav_marker-2.bag')
    ros_ext = em.rospyExtMarker()
    pt1 = ros_ext.ext_point_marker(bagf, '/visualization_marker')
    pt2 = ros_ext.ext_point_marker(bagf, '/uav_pose/cameraPoseMsg')

    plt.plot(pt2[1:, 0], pt2[1:, 1], '-x')
    plt.figure()
    plt.plot(pt2[1:, 0], pt2[1:, 2], '-x')

    idx = np.int32(np.arange(pt2.shape[0]) / 1.0 * pt1.shape[0] / pt2.shape[0])

    pt3 = 0.9 * pt2
    pt3[:,0] += 1000*pt1[idx, 0]-50
    pt3[:, 1] += 1000 * pt1[idx, 1] - 200
    pt3[:, 2] += 150 * pt1[idx, 2]
    pidx = range(pt2.shape[0])
    plt.figure(),plt.plot(pidx,pt3[:,],pidx, pt2[:,])

    plt.legend(['UAV Marker position x',\
                'UAV Marker position y',\
                'UAV Marker position z',\
                'April Tag position x', \
                'April Tag position y', \
                'April Tag position z'])
    plt.axis([0,500,-3000,7000])
    plt.xlabel('time index 0.1s')
    plt.ylabel('millimeter')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(pt2[1:, 0], pt2[1:, 1], pt2[1:, 2], label='Path of UAV Marker measurement')
    ax.scatter(pt2[1:, 0], pt2[1:, 1], pt2[1:, 2])

    ax.plot(pt3[1:, 0], pt3[1:, 1], pt3[1:, 2],c='r',label='Path of April Tag measurement')
    ax.scatter(pt3[1:, 0], pt3[1:, 1], pt3[1:, 2],c='r')

    ax.set_xlabel('X millimeter')
    ax.set_ylabel('Y millimeter')
    ax.set_zlabel('Z millimeter')


    plt.show()
if __name__=='__main__':
    test2()
