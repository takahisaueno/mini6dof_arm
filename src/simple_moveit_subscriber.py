#!/usr/bin/env python



import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from mini6dof_arm.msg import joint_state_msg



class PubSubJointState():
    def __init__(self):
        self.sub_moveit=rospy.Subscriber("joint_states",JointState,self.callback_moveit)
        self.sub_real=rospy.Subscriber("presentJoint",JointState,self.callback_real)
        self.pub=rospy.Publisher("returning",Float32MultiArray,queue_size=10)
        
        self.command_joint=Float32MultiArray()


        


    def callback_moveit(self,data):
        # for i in range(6):
        #     exec("self.command_joint.joint{}_angle=data.position[{}]".format(i+1,i))
        self.command_joint.data=data.position
        
        # self.command_joint.data=4.0
        self.pub.publish(self.command_joint)
        rospy.loginfo("command_angle is:")
        rospy.loginfo(self.command_joint)



    def callback_real(self,data):
        self.real_angle=data
        rospy.loginfo("present_angle joint_state is:")
        rospy.loginfo(self.real_angle)





if __name__=="__main__":
    rospy.init_node("simple_moveit_subscriber")
    node=PubSubJointState()

    while not rospy.is_shutdown():
        # rospy.loginfo
        rospy.spin()


