#! /usr/bin/env python

from gazebo_msgs.srv import GetLinkState
import rospy
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState



class Block:
    def __init__(self, name, reference_frame):
        self._name = name
        self._reference_frame = reference_frame

class Tutorial:

    _blockListDict = {
        'block_a': Block('ur5_arm_wrist_3_link', 'world'), ## musi byc world dla huskyiego z jakiejs niewiadomej nikomu przyczyny
        'block_b': Block('object_base_link', 'world'),

    }

    def show_gazebo_models(self):
        try:
            link_coordinates = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            for block in self._blockListDict.itervalues():
                blockName = str(block._name)
                blockFrame = str(block._reference_frame)
                resp_coordinates = link_coordinates(blockName, blockFrame)

                print '\n'
                print 'Status.success = ', resp_coordinates.success
                print(blockName)
                print("name " + str(block._name))
                #print(str(resp_coordinates))
                print("Position x : " + str(resp_coordinates.link_state.pose.position.x))
                print("Position y : " + str(resp_coordinates.link_state.pose.position.y))
                print("Position z : " + str(resp_coordinates.link_state.pose.position.z))
                print("Quaternion X : " + str(resp_coordinates.link_state.pose.orientation.x))
                print("Quaternion Y : " + str(resp_coordinates.link_state.pose.orientation.y))
                print("Quaternion Z : " + str(resp_coordinates.link_state.pose.orientation.z))
                print("Quaternion W : " + str(resp_coordinates.link_state.pose.orientation.w))



        except rospy.ServiceException as e:
            rospy.loginfo("Get Model State service call failed:  {0}".format(e))

    


if __name__ == '__main__':
    tuto = Tutorial()
    tuto.show_gazebo_models()

