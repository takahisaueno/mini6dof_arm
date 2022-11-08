#!/usr/bin/env python

from operator import ne
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
import math
import numpy as np



PI=3.1415926535

class endeffector_rotation:
    def __init__(self):
        roscpp_initialize(sys.argv)
        self.starting_flag=False
        self.robot = RobotCommander(robot_description="arm_robot/robot_description")
        self.scene = PlanningSceneInterface(ns="arm_robot")
        self.group_name = "arm0"
        self.move_group = MoveGroupCommander(robot_description="arm_robot/robot_description", ns="arm_robot", name=self.group_name)
        self.target_tf_br=tf2_ros.TransformBroadcaster()
        self.sub_target_vector=rospy.Subscriber("yolov5/target_vector",Vector3,self.callback_target)
        self.static_tf=tf2_ros.StaticTransformBroadcaster()
        self.dynamic_tf=tf2_ros.TransformBroadcaster()
        self.tfBuffer=tf2_ros.Buffer()
        self.tf_listener=tf2_ros.TransformListener(self.tfBuffer)


        self.camera_tf_broadcaster()
        self.desired_target_link_broadcaster()

        self.target_tf=TransformStamped()

        self.initial_euler_x=-0.5*PI
        self.initial_euler_y=0.0*PI
        self.initial_euler_z=-0.5*PI

        self.non_euler_angle=Vector3(x=self.initial_euler_x,y=self.initial_euler_y,z=self.initial_euler_z)

        self.pose_goal=Pose()



    def start(self,point):
        self.target_point=Twist()
        self.target_point.linear=Vector3(x=point.x,y=point.y,z=point.z)
        self.target_point.angular=Vector3(x=self.initial_euler_x,y=self.initial_euler_y,z=self.initial_euler_z)
        self.desired_translation,self.desired_quertanion=self.computing_quertanion(self.target_point)
        while not rospy.is_shutdown():

            self.desired_camera_link_broadcaster(self.desired_translation,self.desired_quertanion)
            self.desired_transform=self.get_desired_camera_link_tf()
            if self.desired_transform!=None:
                self.pose_goal=self.TransformToPose(self.desired_transform)
                self.move_arm(self.pose_goal)
                self.starting_flag=True
    
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

    def TransformToPose(self,transform_item):
        target_point=Pose()
        target_point.position.x=transform_item.transform.translation.x
        target_point.position.y=transform_item.transform.translation.y
        target_point.position.z=transform_item.transform.translation.z
        target_point.orientation=transform_item.transform.rotation
        return target_point


    def move_arm(self,pose_goal):
    
        self.move_group.set_pose_target(pose_goal)
        self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
     

    def tf_transform_pose_maker(self,translation,quertanion):
        self.target_tf.header.frame_id = "base_link"
        self.target_tf.child_frame_id = "position_target"
        self.target_tf.transform.translation=translation
        self.target_tf.transform.rotation=quertanion
        self.target_tf.header.stamp=rospy.Time.now()

        self.target_tf_br.sendTransform(self.target_tf)



    def euler_format(self,euler_angle):
        if euler_angle[0]<0:
            euler_angle[0]=2*PI+euler_angle[0]


        if euler_angle[2]<0:
            euler_angle[2]=2*PI+euler_angle[2]

        return euler_angle

    def callback_target(self,target_vector):

        if self.starting_flag==True:
            self.sub_target_vector=target_vector
            print(target_vector.x,",",target_vector.y,",",target_vector.z,",")
            theta=math.acos(target_vector.z)
            phi=math.acos(target_vector.x/math.sin(theta))

            # rospy.loginfo(str(theta)+","+str(phi))
            euler_z=-PI/2+phi
            euler_x=-theta
            euler_y=0.0

            target_vector_list=np.array([[target_vector.x],[target_vector.y],[target_vector.z]])
            z_axis_list=np.array([[0],[0],[1]])

            print(superimposition_matrix(z_axis_list,target_vector_list))


            # print(euler_x,euler_y,euler_z)

            euler_angle=self.euler_format([euler_x,euler_y,euler_z])
            print(euler_angle)
            twist_from_present=Twist()

            twist_from_present.linear=Vector3(x=0.0,y=0.0,z=0.0)
            twist_from_present.angular=Vector3(x=euler_angle[0],y=euler_angle[1],z=euler_angle[2])
            translation,_quertanion=self.computing_quertanion(twist_from_present)

            
            diff_quat=[_quertanion.x,_quertanion.y,_quertanion.z,_quertanion.w]
            # print(diff_quat)
            present_cam_quat=[self.pose_goal.orientation.x,self.pose_goal.orientation.y,self.pose_goal.orientation.z,self.pose_goal.orientation.w]
            
            next_cam_quat=quaternion_multiply(diff_quat, present_cam_quat)

            self.desired_quertanion.x=next_cam_quat[0]
            self.desired_quertanion.y=next_cam_quat[1]  
            self.desired_quertanion.z=next_cam_quat[2]  
            self.desired_quertanion.w=next_cam_quat[3]  
            # print(next_cam_quat)
        
        




    def camera_tf_broadcaster(self):
        self.camera_tf_axis=TransformStamped()
        self.camera_tf_axis.header.stamp=rospy.Time.now()
        self.camera_tf_axis.header.frame_id="target_link"
        self.camera_tf_axis.child_frame_id="camera_link"
        self.camera_tf_axis.transform.translation.x=0
        self.camera_tf_axis.transform.translation.y=0
        self.camera_tf_axis.transform.translation.z=0.08
        self.camera_tf_axis.transform.rotation.x=0
        self.camera_tf_axis.transform.rotation.y=0
        self.camera_tf_axis.transform.rotation.z=-1/2**0.5
        self.camera_tf_axis.transform.rotation.w=1/2**0.5
        self.static_tf.sendTransform(self.camera_tf_axis)

    def desired_target_link_broadcaster(self):
        self.desired_camera_tf_axis=TransformStamped()
        self.desired_camera_tf_axis.header.frame_id="desired_camera_link"
        self.desired_camera_tf_axis.child_frame_id="desired_target_link"
        self.desired_camera_tf_axis.transform.translation.x=0
        self.desired_camera_tf_axis.transform.translation.y=0
        self.desired_camera_tf_axis.transform.translation.z=-0.08
        self.desired_camera_tf_axis.transform.rotation.x=0
        self.desired_camera_tf_axis.transform.rotation.y=0
        self.desired_camera_tf_axis.transform.rotation.z=1/2**0.5
        self.desired_camera_tf_axis.transform.rotation.w=1/2**0.5
        self.static_tf.sendTransform(self.desired_camera_tf_axis)

    def desired_camera_link_broadcaster(self,linear,quertanion):
        self.desired_tf=TransformStamped()
        self.desired_tf.header.frame_id="base_link"
        self.desired_tf.child_frame_id="desired_camera_link"
        self.desired_tf.transform.translation=linear

        self.desired_tf.transform.rotation=quertanion

        self.dynamic_tf.sendTransform(self.desired_tf)

    def get_desired_camera_link_tf(self):
        try:
            trans = self.tfBuffer.lookup_transform("base_link","desired_target_link", rospy.Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None
        # rospy.loginfo(str(trans.transform.translation.x)+","+str(trans.transform.translation.y)+","+str(trans.transform.translation.z))
        return trans







    
    def compute_target_link_queratanion(self,endeffector_quertanion):
        pass







if __name__=="__main__":
    rospy.init_node("rotate_endeffector")
    motion=endeffector_rotation()
    target_point=Point()
    target_point.x=0.37
    target_point.y=0.0
    target_point.z=0.19
    motion.start(target_point)

    