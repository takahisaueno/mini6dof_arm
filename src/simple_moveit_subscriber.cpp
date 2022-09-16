#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <sensor_msgs/JointState.h>
#include "dynamixel_control.h"

/*-------------------------------SimpleMoveItSubscriber----------------------------------------------*/
class SimpleMoveItSubscriber
{
public:
    SimpleMoveItSubscriber();

private:
    ros::Subscriber joint_state_sub;
    void poseCallback(const sensor_msgs::JointStateConstPtr &pose);
    DynamixelControl dynamixel_ctrl;
    float target_joint_angle[Dynamixel_NUM];
    float present_joint_angle[Dynamixel_NUM];
    uint16_t present_pwm[Dynamixel_NUM];
    uint16_t present_load[Dynamixel_NUM];

    uint32_t prof_acc[Dynamixel_NUM];
    uint32_t prof_vel[Dynamixel_NUM];
};

/*--------------------------------SimpleMoveItSubscriber class method----------------------------------*/
SimpleMoveItSubscriber::SimpleMoveItSubscriber()
{
    bool toggle;
    ros::NodeHandle nh;
    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, &SimpleMoveItSubscriber::poseCallback, this);
    toggle = dynamixel_ctrl.toggleTorqueMode(1);
    for (int i = 0; i < Dynamixel_NUM; i++)
    {
        prof_acc[i] = 80;
        prof_vel[i] = 80;
    }
    dynamixel_ctrl.setProfile_acc(prof_acc);
    dynamixel_ctrl.setProfile_vel(prof_vel);
}

void SimpleMoveItSubscriber::poseCallback(const sensor_msgs::JointStateConstPtr &pose)
{
    for (int i = 0; i < Dynamixel_NUM; i++)
    {
        target_joint_angle[i] = pose->position[i];
    }

    dynamixel_ctrl.moveToAngle(target_joint_angle);
    // ROS_INFO_STREAM("Target joint angle is:"<< "[" << pose->position[0] << "," << pose->position[1] << "," << pose->position[2] << "," << pose->position[3] << "," << pose->position[4] << "," << pose->position[5] << "]");
    dynamixel_ctrl.getAngle(present_joint_angle);
    // ROS_INFO_STREAM("Present joint angle is: [" << present_joint_angle[0] << "," << present_joint_angle[1] << "," << present_joint_angle[2] << "," << present_joint_angle[3] << "," << present_joint_angle[4] << "," << present_joint_angle[5] << "]");
    dynamixel_ctrl.getPWM(present_pwm);
    // ROS_INFO_STREAM("Present joint PWM is: [" << present_pwm[0] << "," << present_pwm[1] << "," << present_pwm[2] << "," << present_pwm[3] << "," << present_pwm[4] << "," << present_pwm[5] << "]");
    dynamixel_ctrl.getLoad(present_load);
    ROS_INFO_STREAM("Present joint LOAD is: [" << present_load[0] << "," << present_load[1] << "," << present_load[2] << "," << present_load[3] << "," << present_load[4] << "," << present_load[5] << "]");
}

/*-------------------------------------main------------------------------------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_moveit");

    SimpleMoveItSubscriber simple_moveit;
    ros::spin();
    portHandler->closePort();
    return 0;
}