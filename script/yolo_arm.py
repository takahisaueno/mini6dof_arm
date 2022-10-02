#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import socket
import pickle
import os
import rospkg
import random
import pdb


class GetYoloData():
    def __init__(self):
        self.get_param()
        self.start_yolo_detect()
        self.loop=rospy.Rate(20)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((socket.gethostname(), self.socket_address))


    def start_yolo_detect(self):
        self.rospack = rospkg.RosPack()
        
        self.ros_pack_path=self.rospack.get_path("mini6dof_arm")
        self.ros_src_path=os.path.dirname(self.ros_pack_path)
        self.yolo_path=os.path.join(self.ros_src_path,"yolov5","detect.py")
        self.socket_address=random.randint(1030, 9999)
        rospy.loginfo("----------------------------------"+str(self.source)+"----------------------------")
        os.system("python3 "+self.yolo_path+" --source "+str(self.source)+" --weights "+self.weights+" --socket_address "+str(self.socket_address)+" --socket_flag "+"&")
        rospy.sleep(10.0)
    
    def get_param(self):
        self.source=rospy.get_param("~source",0)
        self.get_image=rospy.get_param("~get_image",False)
        self.weights=rospy.get_param("~model","yolov5s.pt")
        rospy.loginfo("Camera"+str(self.source)+" parameter is setted")
        rospy.loginfo("weights is "+self.weights)
        


    def start_connection(self):
        while not rospy.is_shutdown():
            self.msg_byte = self.s.recv(1024)
            self.msg=pickle.loads(self.msg_byte)
            rospy.loginfo(self.msg)
            self.loop.sleep()
        self.s.close





if __name__=="__main__":
    rospy.init_node("yolov5")
    yolo_class=GetYoloData()
    yolo_class.start_connection()
    rospy.loginfo("finish connection")