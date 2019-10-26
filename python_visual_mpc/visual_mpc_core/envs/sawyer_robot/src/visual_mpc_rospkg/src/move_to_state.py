
import sys
import rospy
import numpy as np
import random


import moveit_commander
import geometry_msgs.msg


class MoveGroupUR(object):
    """docstring for MoveGroupUR."""

    def __init__(self):
        super(MoveGroupUR, self).__init__()
        # self.arg = arg

        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_to_state_ur5')
        # robot = moveit_commander.RobotCommander()
        group_name = 'manipulator'
        group = moveit_commander.MoveGroupCommander(group_name)

        self.pose_goal = geometry_msgs.msg.Pose()


        # self.robot = robot
        # self.scene = scene
        self.group = group

    def move_to_state(self, target_xyz, target_orientation, duration = None):
        if duration is None:
            duration = 1
        # p1 = np.zeros(4)
        # p1[:3], p1[3] = self._get_xyz_angle()
        # p2 = np.zeros(4)
        # p2[:3], p2[3] = target_xyz, target_zangle

        print(target_xyz)
        random_pose_x = 0.33 * random.uniform(-1,1)
        random_pose_y = 0.24 * random.uniform(-1,1)
        # self.pose_goal.position.x = 0.22078085026 + target_xyz[0]
        # self.pose_goal.position.y = 0.68844698851 + target_xyz[1]
        #
        # # pose_goal.position.z = 0.191160579684
        # self.pose_goal.position.z = 0.191160579684 + target_xyz[2]

        self.pose_goal.position.x =  target_xyz[0]
        self.pose_goal.position.y =  target_xyz[1]

        # pose_goal.position.z = 0.191160579684
        self.pose_goal.position.z =  target_xyz[2]

        self.pose_goal.orientation.x = -0.415964133446
        self.pose_goal.orientation.y = 0.352373748554
        self.pose_goal.orientation.z = 0.644633721705
        self.pose_goal.orientation.w = 0.535960768954  #1.0

        # pose_goal.orientation.x = target_orientation.x
        # pose_goal.orientation.y = target_orientation.y
        # pose_goal.orientation.z = target_orientation.z
        # pose_goal.orientation.w = target_orientation.w

        self.group.set_pose_target(self.pose_goal)
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
