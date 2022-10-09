#!/usr/bin/env python
# coding: UTF-8

import sys
import moveit_commander
import rospy
from geometry_msgs.msg import Pose
import pdb
 
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)

    # ノードの初期化
    rospy.init_node('robot_info')
    robot = moveit_commander.RobotCommander(robot_description="arm_robot/robot_description")
    scene = moveit_commander.PlanningSceneInterface(ns="arm_robot")
    group_name = "arm0"
    move_group = moveit_commander.MoveGroupCommander(robot_description="arm_robot/robot_description", ns="arm_robot", name=group_name)
    # ロボット情報
    print "==Robot Info=="
    print "[ group_names ]", robot.get_group_names()
    print "[ current_state ] ", robot.get_current_state()
    print ""


    # グループ情報
    print "==Group Info=="
    print "[ name ] ", move_group.get_name()
    print "[ planning_frame ] ", move_group.get_planning_frame()
    print "[ interface_description ] ", move_group.get_interface_description()
    print ""

    # ジョイント情報
    print "==Joint Info=="
    print "[ active_joints ] ", move_group.get_active_joints()
    print "[ joints ] ", move_group.get_joints()
    print "[ current_joint_values ] ", move_group.get_current_joint_values()
    print ""

    # エンドエフェクタ情報
    print "==EndEffector Info=="
    print "[ has_end_effector_link ] ", move_group.has_end_effector_link()
    print "[ end_effector_link ] ", move_group.get_end_effector_link()
    print "[ current_pose ] ", move_group.get_current_pose()
    print "[ current_rpy ] ", move_group.get_current_rpy()
    print ""

    pdb.set_trace

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 1.4
    joint_goal[2] = 0
    joint_goal[3] = -3.14 / 4
    joint_goal[4] = 0
    joint_goal[5] = 3.14 / 6

    # 関節ゴールによるモーションプランニングの実行
    move_group.go(joint_goal, wait=True)

    # 動きが残っていないことを保証
    move_group.stop()



        # 姿勢ゴール
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    # 姿勢ゴールの指定
    move_group.set_pose_target(pose_goal)

    # 姿勢ゴールによるモーションプランニングの実行
    plan = move_group.go(wait=True)

    # 動きが残っていないことを保証
    move_group.stop()

    # 姿勢によるプランニングの後にターゲットをクリアするのが望ましい(関節のクリアはない)
    move_group.clear_pose_targets()