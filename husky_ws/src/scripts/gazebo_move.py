#! /usr/bin/env python

from gazebo_msgs.srv import GetLinkState
import rospy
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState


ur5_coord = ""

def get_gazebo():
    try:
        while not rospy.is_shutdown():
            ur5_link_coordinates = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            ur5_blockName = 'ur5_arm_wrist_3_link'
            ur5_blockFrame = 'world'
            ur5_resp_coordinates = ur5_link_coordinates(ur5_blockName, ur5_blockFrame)
        
            ur5_coord = ur5_resp_coordinates

            rospy.init_node('set_pose')

            state_msg = ModelState()
            # state_msg.model_name = 'unit_cylinder'
            state_msg.model_name = 'coke_can'
            state_msg.pose.position.x = 0.0#ur5_coord.link_state.pose.position.x + 0.2
            state_msg.pose.position.y = 0.25 #ur5_coord.link_state.pose.position.y
            state_msg.pose.position.z = -0.05#ur5_coord.link_state.pose.position.z
            state_msg.pose.orientation.x = 0.0 #ur5_coord.link_state.pose.orientation.x
            state_msg.pose.orientation.y = 0.0 #ur5_coord.link_state.pose.orientation.y
            state_msg.pose.orientation.z = 0.0 #ur5_coord.link_state.pose.orientation.z
            state_msg.pose.orientation.w = 1.0 #ur5_coord.link_state.pose.orientation.w
            state_msg.reference_frame = "ur5_arm_wrist_3_link"

            print("state_msg: " + str(state_msg))

            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                resp = set_state( state_msg )

            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

            

    except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))



if __name__ == '__main__':
    get_gazebo()
    

    
