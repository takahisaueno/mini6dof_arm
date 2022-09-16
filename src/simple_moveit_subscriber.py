#!/usr/bin/env python



import rospy
from sensor_msgs.msg import JointState
from dynamixel_sdk import *


#-----------------------------------------------------------------------------------------------

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = [0,1,2,3,4,5,6]    #DXL_ID dynamixel ID list
BAUDRATE                    = 115200             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyACM0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


#-----------------------------------------------------------------------------------------------



class PubSubJointState():
    def __init__(self):
        self.sub_moveit=rospy.Subscriber("joint_states",JointState,self.callback_moveit)


        


    def callback_moveit(self,data):
        rospy.INFO(data.position)



    
    def set_goal_position(self,data):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)
        

    def init_portHandler(self):
        self.portHandler = PortHandler(DEVICENAME)
    
    def init_packetHandler(self):
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)



if __name__=="__main__":
    rospy.init_node("simple_moveit_subscriber")
    node=PubSubJointState()

    while not rospy.is_shutdown():
        
        rospy.spin()


