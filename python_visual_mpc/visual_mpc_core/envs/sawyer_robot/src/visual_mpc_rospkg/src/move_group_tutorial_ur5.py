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
    #postion_publish = rospy.Publisher('/move_group/pose',
                                          # geometry_msgs.msg.Pose,
                                          # queue_size=20)

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
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    #self.postion_publish = postion_publish
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def pose_publish(self):

    group = self.group
    postion_publish = self.postion_publish
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pose_pub = geometry_msgs.msg.Pose()
	pose_pub = group.get_current_pose().pose
        rospy.loginfo(pose_pub)
        postion_publish.publish(pose_pub)
        rate.sleep()

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
    print("Current pose: ", current_pose)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 0.535960768954  #1.0

    random_pose_x = 0.33 * random.uniform(-1,1)
    random_pose_y = 0.24 * random.uniform(-1,1)
    pose_goal.position.x = 0.22078085026 + random_pose_x
    pose_goal.position.y = 0.68844698851 + random_pose_y

    pose_goal.position.z = 0.191160579684

    pose_goal.orientation.x = -0.415964133446
    pose_goal.orientation.y = 0.352373748554
    pose_goal.orientation.z = 0.644633721705

    group.set_pose_target(pose_goal)
    #group.set_random_target()

    plan = group.go(wait=True)

    group.stop()

    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    print("New current pose: ", current_pose)

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def get_pose(self):
    group = self.group
    current_pose = group.get_current_pose().pose
    print("Current pose: ", current_pose)
    return 0


  def plan_cartesian_path(self, scale=1):
    group = self.group

    current_pose = group.get_current_pose().pose
    print("Current pose: ", current_pose)

    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x = 0.2
    wpose.position.y = 0.01
    wpose.position.z = 0.2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.2
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    return plan, fraction



  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);


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

    for i in range(1,100):
        print "============ Press `Enter` to execute a movement using a pose goal ..."
        tutorial.go_to_pose_goal()
	    #tutorial.pose_publish()



    #print "============ Press `Enter` to execute go to up state ..."
    #raw_input()
    #tutorial.go_to_up_state()

    #print "============ Press `Enter` to plan and display a Cartesian path ..."
    #raw_input()
    #cartesian_plan, fraction = tutorial.plan_cartesian_path()

    #print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    #raw_input()
    #tutorial.display_trajectory(cartesian_plan)

    #print "============ Press `Enter` to execute a saved path ..."
    #raw_input()
    #tutorial.execute_plan(cartesian_plan)

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
