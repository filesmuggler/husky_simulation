#!/usr/bin/env python
import argparse
import roslib  
import rospy
import actionlib

#move_base_msgs
from move_base_msgs.msg import *

def simple_move(coords):

    rospy.init_node('simple_move')

    #Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )

    #create goal
    goal = MoveBaseGoal()

    #use self?
    #set goal
    c = coords['floats']
    goal.target_pose.pose.position.x = c[0]
    goal.target_pose.pose.position.y = c[1]
    goal.target_pose.pose.orientation.w = c[2]
    goal.target_pose.header.frame_id = 'odom'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listner
    sac.wait_for_server()

    #send goal
    sac.send_goal(goal)

    #finish
    sac.wait_for_result()

    #print result
    print sac.get_result()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Move Husky to the point.')
    parser.add_argument('floats', metavar='N', type=float, nargs='+',
                    help='coordiantes in Gazebo <x, y, orientation_in_radians>')
    args = parser.parse_args()
    d = vars(args)
    print(d)
    try:
        simple_move(d)
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"