#!/usr/bin/env python
import rospy
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np



def pointcloud2_to_array(cloud_msg, squeeze=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray
    Reshapes the returned array to have shape (height, width), even if the height is 1.
    The reason for using np.fromstring rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = ros_numpy.point_cloud2.fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.fromstring(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(ros_numpy.point_cloud2.DUMMY_FIELD_PREFIX)] == ros_numpy.point_cloud2.DUMMY_FIELD_PREFIX)]]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width)) 


def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = ros_numpy.point_cloud2.dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg 




def callback(data):
    pcl2_array = pointcloud2_to_array(data)
    pcl2_new_array = []
    for point in pcl2_array:
        if point[2] > 0.0 and point[0]<0.5 and point[0]>-0.5 and point[1]<0.5 and point[1]>-0.5 or point[2]<0.10:
            #print(point)
            pass
        else:
            pcl2_new_array.append(point)
            print(point)

    avg_obj_center = [0,0,0]
    if len(pcl2_new_array) != 0:
        for point in pcl2_new_array:
            avg_obj_center[0]=avg_obj_center[0]+point[0]
            avg_obj_center[1]=avg_obj_center[0]+point[1]
            avg_obj_center[2]=avg_obj_center[0]+point[2]

        if(len(pcl2_new_array) is not 0):
            avg_obj_center[0]=avg_obj_center[0]*1.0/len(pcl2_new_array)
            avg_obj_center[1]=avg_obj_center[1]*1.0/len(pcl2_new_array)
            avg_obj_center[2]=avg_obj_center[2]*1.0/len(pcl2_new_array)

            print("center: ",avg_obj_center)

        new_pcl2 = array_to_pointcloud2(pcl2_new_array,frame_id='base_link')
        pub = rospy.Publisher("/hellothere",PointCloud2,queue_size=1000000)
        while not rospy.is_shutdown():
            pub.publish(new_pcl2)
            rospy.Rate(0.1).sleep()
    else:
        print("no object within reach of the camera")


def filter_pcl():
    rospy.init_node('filter_pcl',anonymous=True)
    rospy.Subscriber("/ird435/octomap_point_cloud_centers",PointCloud2,callback)
    rospy.spin()

if __name__=='__main__':
    filter_pcl()