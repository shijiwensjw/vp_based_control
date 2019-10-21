#!/usr/bin/env python

#
# Code adapted from https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
# to UR5 robot
#
from math import pi
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import random
## END_SUB_TUTORIAL

# def all_close(goal, actual, tolerance):
#   """
#   Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
#   @param: goal       A list of floats, a Pose or a PoseStamped
#   @param: actual     A list of floats, a Pose or a PoseStamped
#   @param: tolerance  A float
#   @returns: bool
#   """
#   all_equal = True
#   if type(goal) is list:
#     for index in range(len(goal)):
#       if abs(actual[index] - goal[index]) > tolerance:
#         return False
#
#   elif type(goal) is geometry_msgs.msg.PoseStamped:
#     return all_close(goal.pose, actual.pose, tolerance)
#
#   elif type(goal) is geometry_msgs.msg.Pose:
#     return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
#
#   return True

class MoveGroupTutorial(object):
  """MoveGroupTutorial"""
  def __init__(self):
    super(MoveGroupTutorial, self).__init__()

    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_tutorial_ur5', anonymous=True)

    # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    # the robot:
    robot = moveit_commander.RobotCommander()

    # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    # to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    # to one group of joints.  In this case the group is the joints in the UR5
    # arm so we set ``group_name = manipulator``. If you are using a different robot,
    # you should change this value to the name of your robot arm planning group.
    group_name = "manipulator" # See .srdf file to get available group names
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    postion_publish = rospy.Publisher('/move_group/pose',
                                           geometry_msgs.msg.Pose,
                                           queue_size=20)

    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.postion_publish = postion_publish
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def pose_publish(self):

    group = self.group
    postion_publish = self.postion_publish
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
      # pose_pub = geometry_msgs.msg.Pose()
      pose_pub = group.get_current_pose().pose
      # rospy.loginfo(pose_pub)
      postion_publish.publish(pose_pub)
      rate.sleep()

  def get_pose(self):
    group = self.group
    current_pose = group.get_current_pose().pose
    print("Current pose: ", current_pose)
    return 0

def main():
  try:
    # print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    # raw_input()
    tutorial = MoveGroupTutorial()


    # tutorial.get_pose()
    tutorial.pose_publish()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
