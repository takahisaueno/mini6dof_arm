#!/usr/bin/env python
import sys, math, copy
import rospy, tf, geometry_msgs.msg
from moveit_commander import MoveGroupCommander, RobotCommander,PlanningSceneInterface, roscpp_initialize
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, Point, Vector3,TransformStamped
import tf2_ros
import tf2_msgs.msg
import tf_conversions
from tf.transformations import *
import pdb
import copy
import numpy as np


PI=3.1415926535

class endeffector_rotation:
    def __init__(self,movement=0):
        roscpp_initialize(sys.argv)
        self.robot = RobotCommander(robot_description="arm_robot/robot_description")
        self.scene = PlanningSceneInterface(ns="arm_robot")
        self.group_name = "arm0"
        self.move_group = MoveGroupCommander(robot_description="arm_robot/robot_description", ns="arm_robot", name=self.group_name)
        self.target_tf_br=tf2_ros.TransformBroadcaster()
        self.static_tf=tf2_ros.StaticTransformBroadcaster()

        self.endeffector_tf_broadcaster()

        self.target_tf=TransformStamped()
        self.dx=0.01
        self.direction_x=1
        self.direction_y=1
        self.direction_z=1
        self.r_angular_limit=0.9*PI
        self.g_angular_limit=0.14*PI
        self.b_angular_limit=0.9*PI
        self.non_euler_angle=Vector3(x=0.0,y=0.35*PI,z=0.0)



        self.pose_goal=Pose()
        self.pose_goal_list=[]


    def start(self,point):
        self.target_point=Twist()
        self.target_point.linear=Vector3(x=point.x,y=point.y,z=point.z)
        self.target_point.angular=Vector3(x=0.0,y=0.35*PI,z=0.0)
        
        while not rospy.is_shutdown():
            for timestep in range(50):
                self.changing_endeffector_angle()
                translation_endeffector,quertanion_endeffector=self.computing_quertanion(self.target_point)
                

                translation,quertanion=self.target_link_pose_analyzer(translation_endeffector,quertanion_endeffector,Vector3(x=0.0,y=0.0,z=0.012))
                self.pose_goal=self.make_pose_goal(translation,quertanion)
                self.pose_goal_list.append(copy.deepcopy(self.pose_goal))
            self.tf_transform_pose_maker(translation_endeffector,quertanion_endeffector)
            # self.move_arm(self.pose_goal)
            # self.tf_transform_pose_maker(translation,quertanion)

            self.move_arm_list(self.pose_goal_list)
                
            self.pose_goal_list=[]
    
        rospy.loginfo("motion is finished")
        


    def computing_quertanion(self,target_pose):
        self.quertanion=tf_conversions.transformations.quaternion_from_euler(target_pose.angular.x,target_pose.angular.y,target_pose.angular.z)
        self.translation=target_pose.linear
        
        return self.translation,Quaternion(x=self.quertanion[0],y=self.quertanion[1],z=self.quertanion[2],w=self.quertanion[3])

    def make_pose_goal(self,translation,quertanion):
        pose_goal=Pose()
        pose_goal.position=translation
        pose_goal.orientation=quertanion
        return pose_goal


    def move_arm(self,pose_goal):
    
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def move_arm_list(self,pose_goal_list):
        rospy.loginfo("moving with list")
    
        self.move_group.set_pose_targets(pose_goal_list)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # rospy.loginfo("Reached Target")

    def target_link_pose_analyzer(self,target_translation,target_quertanion,diff_frame_translation):
        Pab=[diff_frame_translation.x,diff_frame_translation.y,diff_frame_translation.z,0.0]
        Roa=[target_quertanion.x,target_quertanion.y,target_quertanion.z,target_quertanion.w]
        Pob=[target_translation.x,target_translation.y,target_translation.z,0.0]
        # pdb.set_trace()
        RoaPab_Roa=quaternion_multiply(quaternion_multiply(Roa,Pab).tolist(),quaternion_inverse(Roa).tolist())

        Poa=(np.array(Pob)-np.array(RoaPab_Roa)).tolist()

        return Vector3(x=Poa[0],y=Poa[1],z=Poa[2]), Quaternion(x=target_quertanion.x,y=target_quertanion.y,z=target_quertanion.z,w=target_quertanion.w)



    def tf_transform_pose_maker(self,translation,quertanion):
        self.target_tf.header.frame_id = "base_link"
        self.target_tf.child_frame_id = "endeffector_position_target"
        self.target_tf.transform.translation=translation
        self.target_tf.transform.rotation=quertanion
        self.target_tf.header.stamp=rospy.Time.now()

        self.target_tf_br.sendTransform(self.target_tf)


    def changing_endeffector_angle(self):
        if self.non_euler_angle.x>self.r_angular_limit:
            self.direction_x=0
        if self.non_euler_angle.x<-self.r_angular_limit:
            self.direction_x=1

        if self.direction_x==1:
            self.non_euler_angle.x=self.non_euler_angle.x+self.dx
        else:
            self.non_euler_angle.x=self.non_euler_angle.x-self.dx

      


        if self.non_euler_angle.y>0.35*PI+self.g_angular_limit:
            self.direction_y=0
        if self.non_euler_angle.y<0.35*PI-self.g_angular_limit:
            self.direction_y=1

        if self.direction_y==1:
            self.non_euler_angle.y=self.non_euler_angle.y+self.dx
        else:
            self.non_euler_angle.y=self.non_euler_angle.y-self.dx

        if self.non_euler_angle.z>self.b_angular_limit:
            self.direction_z=0
        if self.non_euler_angle.z<-self.b_angular_limit:
            self.direction_z=1

        if self.direction_z==1:
            self.non_euler_angle.z=self.non_euler_angle.z+self.dx
        else:
            self.non_euler_angle.z=self.non_euler_angle.z-self.dx

        self.euler_angle()


    def euler_angle(self):
        if self.non_euler_angle.x<0.0:
            self.target_point.angular.x=self.non_euler_angle.x+2*PI
        else:
            self.target_point.angular.x=self.non_euler_angle.x

   
        self.target_point.angular.y=self.non_euler_angle.y


        if self.non_euler_angle.z<0.0:
            self.target_point.angular.z=self.non_euler_angle.z+2*PI
        else:
            self.target_point.angular.z=self.non_euler_angle.z
        
        # rospy.loginfo("<"+str(self.target_point.angular.x)+","+str(self.target_point.angular.y)+","+str(self.target_point.angular.z)+">")



    def endeffector_tf_broadcaster(self):
        self.endeffector_tf_axis=TransformStamped()
        self.endeffector_tf_axis.header.stamp=rospy.Time.now()
        self.endeffector_tf_axis.header.frame_id="target_link"
        self.endeffector_tf_axis.child_frame_id="rotating_center_link"
        self.endeffector_tf_axis.transform.translation.x=0
        self.endeffector_tf_axis.transform.translation.y=0
        self.endeffector_tf_axis.transform.translation.z=0.012
        self.endeffector_tf_axis.transform.rotation.x=0.0
        self.endeffector_tf_axis.transform.rotation.y=0.0
        self.endeffector_tf_axis.transform.rotation.z=0.0
        self.endeffector_tf_axis.transform.rotation.w=1.0
        self.static_tf.sendTransform(self.endeffector_tf_axis)




if __name__=="__main__":
    rospy.init_node("rotate_endeffector")
    motion=endeffector_rotation()
    target_point=Point()
    target_point.x=0.30
    target_point.y=0.0
    target_point.z=0.18
    motion.start(target_point)

    