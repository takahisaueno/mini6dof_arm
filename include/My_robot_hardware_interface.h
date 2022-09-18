#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include "dynamixel_control.h"

#define Dynamixel_NUM 6

class MyRobot : public hardware_interface::RobotHW
{
public:
    MyRobot();
    ~MyRobot();
    void init();
    void read();
    void write();
    void Dynamixel_init();

protected:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    double joint_position_[Dynamixel_NUM];
    double joint_velocity_[Dynamixel_NUM];
    double joint_effort_[Dynamixel_NUM];
    double joint_position_command_[Dynamixel_NUM];

private:
    DynamixelControl dynamixel_ctrl;
    uint32_t prof_acc[Dynamixel_NUM];
    uint32_t prof_vel[Dynamixel_NUM];
};