#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf


def callback(data):
    move_group = self.move_group     
    
    move_group.set_pose_target(data)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(data, current_pose, 0.01)

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
      for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
          return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
      return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
      return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class MoveGroupPythonIntefaceTutorial(object):
    def __init__(self):
      super(MoveGroupPythonIntefaceTutorial, self).__init__()

      moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
      rospy.Subscriber("/objectcoord",geometry_msgs.msg.Pose,callback)
      rospy.spin()

      robot = moveit_commander.RobotCommander()

      scene = moveit_commander.PlanningSceneInterface()

      group_name = "manipulator"
      move_group = moveit_commander.MoveGroupCommander(group_name)

      display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
      
      planning_frame = move_group.get_planning_frame()
      
      eef_link = move_group.get_end_effector_link()

      group_names = robot.get_group_names()
      robot.get_current_state()

      # Misc variables    
      self.robot = robot
      self.scene = scene
      self.move_group = move_group
      self.display_trajectory_publisher = display_trajectory_publisher
      self.planning_frame = planning_frame
      self.eef_link = eef_link
      self.group_names = group_names

    def go_to_pose_goal(self):
      move_group = self.move_group
      #pose_goal = geometry_msgs.msg.Pose()
      
      ## euler for plug
      # roll=1.54
      # pitch=0
      # yaw=3.14
      # ## pose for plug
      # pose_goal.position.x = 0.30
      # pose_goal.position.y = 0.70
      # pose_goal.position.z = 0.15
      ## euler for unplug
      # roll=1.54
      # pitch=0
      # yaw=0
      # ## pose for unplug
      # pose_goal.position.x = 0.30
      # pose_goal.position.y = -0.60
      # pose_goal.position.z = 0.15

      # quaternion = tf.transformations.quaternion_from_euler(roll,pitch,yaw)

      global pose
      print(pose)
      
      
      # pose.orientation.x = quaternion[0]
      # pose.orientation.y = quaternion[1]
      # pose.orientation.z = quaternion[2]
      # pose.orientation.w = quaternion[3]
      
      move_group.set_pose_target(pose)

      plan = move_group.go(wait=True)
      move_group.stop()
      move_group.clear_pose_targets()
      current_pose = self.move_group.get_current_pose().pose
      return all_close(pose, current_pose, 0.01)

def main():
    try:
        print("Grabing object...")
        tutorial = MoveGroupPythonIntefaceTutorial()
        #print "============ Press `Enter` to execute a movement using a pose goal ..."
        #raw_input()
        #tutorial.go_to_pose_goal()
        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
