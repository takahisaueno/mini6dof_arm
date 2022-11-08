#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import socket
import pickle
import os
import rospkg
import random
import pdb
import yaml
import argparse
from geometry_msgs.msg import Vector3


class GetYoloData():
    def __init__(self,object=None):
        self.get_param()
        self.start_yolo_detect()
        self.get_camera_param()
        if(object!=None):
            self.object=object
        self.loop=rospy.Rate(20)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        connected = False
        while not connected:
            try:
                self.s.connect((socket.gethostname(), self.socket_address))
                connected = True
            except Exception as e:
                pass #Do nothing, just try again

        self.before_target_pixel_x=320
        self.before_target_pixel_y=240
        self.pub_target_vector = rospy.Publisher("yolov5/target_vector", Vector3, queue_size=1)


    def start_yolo_detect(self):
        self.rospack = rospkg.RosPack()
        
        self.ros_pack_path=self.rospack.get_path("mini6dof_arm")
        self.ros_src_path=os.path.dirname(self.ros_pack_path)
        self.yolo_path=os.path.join(self.ros_src_path,"yolov5","detect.py")
        self.socket_address=random.randint(1030, 9999)
        rospy.loginfo("----------------------------------"+str(self.source)+"----------------------------")
        os.system("python3 "+self.yolo_path+" --source "+str(self.source)+" --weights "+self.weights+" --socket_address "+str(self.socket_address)+" --socket_flag "+"&")
        rospy.sleep(5.0)
    
    def get_param(self):
        self.source=rospy.get_param("~source",0)
        self.get_image=rospy.get_param("~get_image",False)
        self.weights=rospy.get_param("~model","yolov5s.pt")
        rospy.loginfo("Camera"+str(self.source)+" parameter is setted")
        rospy.loginfo("weights is "+self.weights)
        


    def start_connection(self):
        while not rospy.is_shutdown():
            self.msg_byte = self.s.recv(1024)
            try:
                self.msg=pickle.loads(self.msg_byte)
            except pickle.UnpicklingError as e:
            # normal, somewhat expected
                continue
            self.calculate_vector(self.target_object_pixel_num())
            self.loop.sleep()
        self.s.close
    
    def target_object_pixel_num(self):
        target_list_index=[]
        pixel_distance_from_before=[]

        for i,object_list in enumerate(self.msg):
            if object_list[-1]==self.object:
                target_list_index.append(i)
        # pdb.set_trace()
        for target in target_list_index:
            pixel_distance=(self.target_middle_pixel(self.msg[target])[0]-self.before_target_pixel_x)**2+(self.target_middle_pixel(self.msg[target])[1]-self.before_target_pixel_y)**2
            pixel_distance_from_before.append(pixel_distance)
        
        if not pixel_distance_from_before:
            return None
        else:
            target_pixel=self.target_middle_pixel(self.msg[target_list_index[pixel_distance_from_before.index(min(pixel_distance_from_before))]])
            self.before_target_pixel_x=target_pixel[0]
            self.before_target_pixel_y=target_pixel[1]
            return target_pixel
        
    def target_middle_pixel(self,object_tensor_info):
        middle_pixel=[int(object_tensor_info[0]+object_tensor_info[2]/2),int(object_tensor_info[1]+object_tensor_info[3]/2)]
        # pdb.set_trace()
        return middle_pixel

    def get_camera_param(self):
        self.camera_param_path=rospy.get_param("~camera_param","/home/ubuntu/catkin_ws/src/camera_callibration")
        with open(self.camera_param_path, 'r') as yml:
            self.cam_param = yaml.safe_load(yml)
        self.camera_matrix=self.cam_param["camera_matrix"]["data"]

    def calculate_vector(self,target_pixel):
        if target_pixel!=None:
            target_vector=Vector3()
            target_vector.x=float(target_pixel[0]-self.camera_matrix[2])/self.camera_matrix[0]
            target_vector.y=float(target_pixel[1]-self.camera_matrix[5])/self.camera_matrix[4]
            target_vector.z=1.0

            target_vector_norm=Vector3()
            target_vector_norm.x=target_vector.x/(target_vector.x**2+target_vector.y**2+target_vector.z**2)**0.5
            target_vector_norm.y=target_vector.y/(target_vector.x**2+target_vector.y**2+target_vector.z**2)**0.5
            target_vector_norm.z=target_vector.z/(target_vector.x**2+target_vector.y**2+target_vector.z**2)**0.5
            
            
            self.pub_target_vector.publish(target_vector_norm)




def argParse():
    parse=argparse.ArgumentParser(description="")
    parse.add_argument("--object",type=str, default="person",help="")
    parse.add_argument("__name",type=str,help="")
    parse.add_argument("__log",type=str,help="")
    config=parse.parse_args()
    config.object=config.object.replace("_"," ")
    return config


if __name__=="__main__":
    rospy.init_node("yolov5")
    config=argParse()
    yolo_class=GetYoloData(object=config.object)
    yolo_class.start_connection()
    rospy.loginfo("finish connection")