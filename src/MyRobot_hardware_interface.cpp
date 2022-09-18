#include "My_robot_hardware_interface.h"
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

MyRobot::MyRobot()
{
    Dynamixel_init();
    init();
}

MyRobot::~MyRobot()
{
}

void MyRobot::Dynamixel_init()
{
    bool toggle;
    toggle = dynamixel_ctrl.toggleTorqueMode(1);
    for (int i = 0; i < Dynamixel_NUM; i++)
    {
        prof_acc[i] = 80;
        prof_vel[i] = 80;
    }
    dynamixel_ctrl.setProfile_acc(prof_acc);
    dynamixel_ctrl.setProfile_vel(prof_vel);
}

void MyRobot::init()
{
    hardware_interface::JointStateHandle jointStateHandle1("arm1_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle1);
    hardware_interface::JointHandle jointPositionHandle1(jointStateHandle1, &joint_position_command_[0]);
    position_joint_interface_.registerHandle(jointPositionHandle1);

    hardware_interface::JointStateHandle jointStateHandle2("arm2_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandle2);
    hardware_interface::JointHandle jointPositionHandle2(jointStateHandle2, &joint_position_command_[1]);
    position_joint_interface_.registerHandle(jointPositionHandle2);

    hardware_interface::JointStateHandle jointStateHandle3("arm3_joint", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandle3);
    hardware_interface::JointHandle jointPositionHandle3(jointStateHandle3, &joint_position_command_[2]);
    position_joint_interface_.registerHandle(jointPositionHandle3);

    hardware_interface::JointStateHandle jointStateHandle4("arm4_joint", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
    joint_state_interface_.registerHandle(jointStateHandle4);
    hardware_interface::JointHandle jointPositionHandle4(jointStateHandle4, &joint_position_command_[3]);
    position_joint_interface_.registerHandle(jointPositionHandle4);

    hardware_interface::JointStateHandle jointStateHandle5("arm5_joint", &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]);
    joint_state_interface_.registerHandle(jointStateHandle5);
    hardware_interface::JointHandle jointPositionHandle5(jointStateHandle5, &joint_position_command_[4]);
    position_joint_interface_.registerHandle(jointPositionHandle5);

    hardware_interface::JointStateHandle jointStateHandle6("arm6_joint", &joint_position_[5], &joint_velocity_[5], &joint_effort_[5]);
    joint_state_interface_.registerHandle(jointStateHandle6);
    hardware_interface::JointHandle jointPositionHandle6(jointStateHandle6, &joint_position_command_[5]);
    position_joint_interface_.registerHandle(jointPositionHandle6);

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
}

void MyRobot::write()
{
    float joint_angle[Dynamixel_NUM];

    // positionJointSaturationInterface.enforceLimits(elapsed_time);
    for (int i = 0; i < Dynamixel_NUM; i++)
    {
        joint_angle[i] = (float)joint_position_command_[i];
    }

    dynamixel_ctrl.moveToAngle(joint_angle);
}

void MyRobot::read()
{
    float joint_angle[Dynamixel_NUM];
    dynamixel_ctrl.getAngle(joint_angle);
    for (int i = 0; i < Dynamixel_NUM; i++)
    {
        joint_position_[i] = (double)joint_angle[i];
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MyRobot_hardware_interface_node");
    MyRobot robot;

    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Time t = ros::Time::now();
    ros::Rate rate(50);

    while (ros::ok())
    {
        ros::Duration d = ros::Time::now() - t;
        ros::Time t = ros::Time::now();

        robot.read();
        cm.update(t, d);
        robot.write();
        rate.sleep();
    }
    return 0;
}