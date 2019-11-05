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
import std_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import random
import numpy as np
## END_SUB_TUTORIAL

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
    # scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    # to one group of joints.  In this case the group is the joints in the UR5
    # arm so we set ``group_name = manipulator``. If you are using a different robot,
    # you should change this value to the name of your robot arm planning group.
    group_name = "manipulator" # See .srdf file to get available group names
    group = moveit_commander.MoveGroupCommander(group_name)

    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                                moveit_msgs.msg.DisplayTrajectory,
    #                                                queue_size=20)
    #postion_publish = rospy.Publisher('/move_group/pose',
                                          # geometry_msgs.msg.Pose,
                                          # queue_size=20)
    # self.is_running_pub = rospy.Publisher('/is_running', std_msgs.msg.Bool, queue_size=10)

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
    self.robot = robot
    # self.scene = scene
    self.group = group
    # self.display_trajectory_publisher = display_trajectory_publisher
    #self.postion_publish = postion_publish
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

    # self._low_bound = np.array([-0.416, 0.352, 0.167, -1])
    # self._high_bound = np.array([0.556, 0.926, 0.170, 1])
    self._low_bound = np.array([-0.35, 0.360, 0.118, -1])
    self._high_bound = np.array([0.23, 0.920, 0.125, 1])

    group.set_max_velocity_scaling_factor(0.18)
    group.set_max_acceleration_scaling_factor(0.08)

  # def is_running_publish(self, run_flag=False):
  #   rate = rospy.Rate(100)
  #   while not rospy.is_shutdown():
  #     self.is_running_pub.publish(run_flag)
  #     rate.sleep()

  def go_to_up_state(self):

    group = self.group

    joint_goal = group.get_current_joint_values()
    print(type(joint_goal), joint_goal)

    joint_goal[0] = 0
    joint_goal[1] = -pi * 0.5
    joint_goal[2] = 0
    joint_goal[3] = -pi * 0.5
    joint_goal[4] = 0
    joint_goal[5] = 0

    group.go(joint_goal, wait=True)

    group.stop()

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):

    group = self.group

    current_pose = group.get_current_pose().pose
    # print("Current pose: ", current_pose)

    pose_goal = geometry_msgs.msg.Pose()


    pose_goal.position.x = random.uniform(self._low_bound[0], self._high_bound[0])
    pose_goal.position.y = random.uniform(self._low_bound[1], self._high_bound[1])
    pose_goal.position.z = random.uniform(self._low_bound[2], self._high_bound[2])

    pose_goal.orientation.w = 0.535960768954  #1.0
    pose_goal.orientation.x = -0.415964133446
    pose_goal.orientation.y = 0.352373748554
    pose_goal.orientation.z = 0.644633721705


    # self.is_running_publish(run_flag=True)
    group.set_pose_target(pose_goal)
    #group.set_random_target()
    # self.is_running_pub.publish(True)

    plan = group.go(wait=True)

    group.stop()
    # self.is_running_publish(run_flag=False)
    group.clear_pose_targets()
    # self.is_running_pub.publish(False)

    current_pose = group.get_current_pose().pose
    print("New current pose: ", current_pose)

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def reset(self, flag):
    pose_goal = geometry_msgs.msg.Pose()

    if flag == 0:

      pose_goal.position.x = self._low_bound[0] - 0.05
      pose_goal.position.y = random.uniform(self._low_bound[1], self._high_bound[1])
      pose_goal.position.z = random.uniform(self._low_bound[2], self._high_bound[2])

    elif flag == 1:

      pose_goal.position.x = self._high_bound[0] + 0.05
      pose_goal.position.y = random.uniform(self._low_bound[1], self._high_bound[1])
      pose_goal.position.z = random.uniform(self._low_bound[2], self._high_bound[2])

    pose_goal.orientation.w = 0.535960768954  #1.0
    pose_goal.orientation.x = -0.415964133446
    pose_goal.orientation.y = 0.352373748554
    pose_goal.orientation.z = 0.644633721705

    self.group.set_pose_target(pose_goal)
    plan = self.group.go(wait=True)
    self.group.stop()
    self.group.clear_pose_targets()
    print("Set at bound point")


  def get_pose(self):
    group = self.group
    current_pose = group.get_current_pose().pose
    print("Current pose: ", current_pose)
    return 0

  def execute_plan(self, plan):

    group = self.group

    group.execute(plan, wait=True)


def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    tutorial = MoveGroupTutorial()

    #tutorial.get_pose()
    #tutorial.pose_publish()

    #print "============ Press `Enter` to execute a movement using a joint state goal ..."
    #raw_input()
    #tutorial.go_to_up_state()

    for i in range(0,200):
        print "Begin {}th trial".format(i)
        if i%10 == 0:
            tutorial.reset(0)
            tutorial.reset(1)
        # try:
        run_correct = tutorial.go_to_pose_goal()
        if not run_correct:
            print("Wrong......")
            sys.exit()
	    #tutorial.pose_publish()



    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
