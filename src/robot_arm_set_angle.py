#!/usr/bin/env python
# coding: UTF-8
import sys
from math import pi

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy


def main():
    # MoveitCommanderの初期化
    moveit_commander.roscpp_initialize(sys.argv)

    # ノードの生成
    rospy.init_node("joint_planner")

    # MoveGroupCommanderの準備
    move_group = moveit_commander.MoveGroupCommander("arm0")
    robot=moveit_commander.RobotCommander()
    scene=moveit_commander.PlanningSceneInterface()
    # 関節の角度でゴール状態を指定
    joint_goal = [0, 0.6, 0.6, 0, 0.375, 0]
    move_group.set_joint_value_target(joint_goal)

    # モーションプランの計画と実行
    move_group.go(wait=True)

    # 後処理
    move_group.stop()


if __name__ == "__main__":
    main()