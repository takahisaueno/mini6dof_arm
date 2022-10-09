#!/usr/bin/env python
import sys, math, copy
import rospy, tf, geometry_msgs.msg
from moveit_commander import MoveGroupCommander, RobotCommander,PlanningSceneInterface, roscpp_initialize
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, Point, Vector3,TransformStamped
import tf2_ros
import tf_conversions
import pdb



class endeffector_rotation:
    def __init__(self):
        roscpp_initialize(sys.argv)
        self.robot = RobotCommander(robot_description="arm_robot/robot_description")
        self.scene = PlanningSceneInterface(ns="arm_robot")
        self.group_name = "arm0"
        self.move_group = MoveGroupCommander(robot_description="arm_robot/robot_description", ns="arm_robot", name=self.group_name)
        self.target_tf_br=tf2_ros.TransformBroadcaster()

        self.target_tf=TransformStamped()
        self.dx=0.05
        self.direction=1

    def start(self,point):
        self.target_point=Twist()
        self.target_point.linear=Vector3(x=point.x,y=point.y,z=point.z)
        self.target_point.angular=Vector3(x=0.0,y=0.5*3.141592,z=0.0)

        while not rospy.is_shutdown():
            self.changing_endeffector_angle()
            translation,quertanion=self.computing_quertanion(self.target_point)
            self.tf_transform_pose_maker(translation,quertanion)
            self.move_arm(translation,quertanion)
    
        rospy.loginfo("motion is finished")
        


    def computing_quertanion(self,target_pose):
        self.quertanion=tf_conversions.transformations.quaternion_from_euler(target_pose.angular.x,target_pose.angular.y,target_pose.angular.z)
        self.translation=target_pose.linear
        
        return self.translation,Quaternion(x=self.quertanion[0],y=self.quertanion[1],z=self.quertanion[2],w=self.quertanion[3])


    def move_arm(self,translation,quertanion):
        pose_goal=Pose()
        pose_goal.position=translation
        pose_goal.orientation=quertanion
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        rospy.loginfo("Reached Target")

    def tf_transform_pose_maker(self,translation,quertanion):
        self.target_tf.header.frame_id = "base_link"
        self.target_tf.child_frame_id = "position_target"
        self.target_tf.transform.translation=translation
        self.target_tf.transform.rotation=quertanion
        self.target_tf.header.stamp=rospy.Time.now()

        self.target_tf_br.sendTransform(self.target_tf)


    def changing_endeffector_angle(self):
        if self.target_point.angular.x>0.5:
            self.direction=0
        if self.target_point.angular.x<-0.5:
            self.direction=1

        if self.direction==1:
            self.target_point.angular.x=self.target_point.angular.x+self.dx
        else:
            self.target_point.angular.x=self.target_point.angular.x-self.dx


        











if __name__=="__main__":
    rospy.init_node("rotate_endeffector")
    motion=endeffector_rotation()
    target_point=Point()
    target_point.x=0.3
    target_point.y=0.0
    target_point.z=0.2
    motion.start(target_point)

    