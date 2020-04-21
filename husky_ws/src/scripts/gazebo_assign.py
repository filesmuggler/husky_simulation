#! /usr/bin/env python

from gazebo_msgs.srv import GetLinkState
import rospy
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


ur_coord = ""

def get_gazebo():
    try:
        link_coordinates = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        blockName = 'ur5_arm_wrist_3_link'
        blockFrame = 'world'
        resp_coordinates = link_coordinates(blockName, blockFrame)
        print '\n'
        print 'Status.success = ', resp_coordinates.success
        print(blockName)
        print("name " + str(blockName))
        #print(str(resp_coordinates))
        print("Position x : " + str(resp_coordinates.link_state.pose.position.x))
        print("Position y : " + str(resp_coordinates.link_state.pose.position.y))
        print("Position z : " + str(resp_coordinates.link_state.pose.position.z))
        print("Quaternion X : " + str(resp_coordinates.link_state.pose.orientation.x))
        print("Quaternion Y : " + str(resp_coordinates.link_state.pose.orientation.y))
        print("Quaternion Z : " + str(resp_coordinates.link_state.pose.orientation.z))
        print("Quaternion W : " + str(resp_coordinates.link_state.pose.orientation.w))

        return resp_coordinates


    except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))



if __name__ == '__main__':
    ur_coord = get_gazebo()
    print("ur_coord: " + str(ur_coord))

    rospy.init_node('set_pose')

    state_msg = ModelState()
    state_msg.model_name = 'box'
    state_msg.pose.position.x = ur_coord.link_state.pose.position.x
    state_msg.pose.position.y = ur_coord.link_state.pose.position.y + 0.25
    state_msg.pose.position.z = ur_coord.link_state.pose.position.z
    state_msg.pose.orientation.x = ur_coord.link_state.pose.orientation.x
    state_msg.pose.orientation.y = ur_coord.link_state.pose.orientation.y
    state_msg.pose.orientation.z = ur_coord.link_state.pose.orientation.z
    state_msg.pose.orientation.w = ur_coord.link_state.pose.orientation.w

    print("state_msg: " + str(state_msg))

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
