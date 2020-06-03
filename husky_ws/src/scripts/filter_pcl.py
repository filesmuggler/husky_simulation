#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2
# import ros_numpy
# import numpy as np

def callback(data):
    # # pcl2 to numpy
    # pc = ros_numpy.numpify(data)
    # points = np.zeros((pc.shape[0],3))
    # points[:,0] = pc['x']
    # points[:,1] = pc['y']
    # points[:,2] = pc['z']

    for point in sensor_msgs.point_cloud2.read_points(data, skip_nans=True):
        print(point[0],point[1],point[2])
    pass

def filter_pcl():
    rospy.init_node('filter_pcl',anonymous=True)
    rospy.Subscriber("/ird435/octomap_point_cloud_centers",PointCloud2,callback)
    rospy.spin()

if __name__=='__main__':
    filter_pcl()