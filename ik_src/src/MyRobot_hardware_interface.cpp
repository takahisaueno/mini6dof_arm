#include "MyRobot_hardware_interface.h"

MyRobot::MyRobot(ros::NodeHandle &nh) : nh_(nh)
{

    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 10;
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

MyRobot::~MyRobot()
{
}

MyRobot::init()
{
    hardware_interface::JointStateHandle jointStateHandle1("arm1_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle1);
    hardware_interface::JointHandle jointPositionHandle1(jointStateHandle1, &joint_position_command_[0]);
    position_joint_interface_.registerHandle(jointPositionHandle1);
    joint_limits_interface::getJointLimits("arm1_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle1(jointPositionHandle1, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle1);

    hardware_interface::JointStateHandle jointStateHandle2("arm2_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandle2);
    hardware_interface::JointHandle jointPositionHandle2(jointStateHandle2, &joint_position_command_[1]);
    position_joint_interface_.registerHandle(jointPositionHandle2);
    joint_limits_interface::getJointLimits("arm2_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle2(jointPositionHandle2, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle2);

    hardware_interface::JointStateHandle jointStateHandle3("arm3_joint", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandle3);
    hardware_interface::JointHandle jointPositionHandle3(jointStateHandle3, &joint_position_command_[2]);
    position_joint_interface_.registerHandle(jointPositionHandle3);
    joint_limits_interface::getJointLimits("arm3_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle3(jointPositionHandle3, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle3);

    hardware_interface::JointStateHandle jointStateHandle4("arm4_joint", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
    joint_state_interface_.registerHandle(jointStateHandle4);
    hardware_interface::JointHandle jointPositionHandle4(jointStateHandle4, &joint_position_command_[3]);
    position_joint_interface_.registerHandle(jointPositionHandle4);
    joint_limits_interface::getJointLimits("arm4_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle4(jointPositionHandle4, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle4);

    hardware_interface::JointStateHandle jointStateHandle5("arm5_joint", &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]);
    joint_state_interface_.registerHandle(jointStateHandle5);
    hardware_interface::JointHandle jointPositionHandle5(jointStateHandle5, &joint_position_command_[4]);
    position_joint_interface_.registerHandle(jointPositionHandle5);
    joint_limits_interface::getJointLimits("arm5_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle5(jointPositionHandle5, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle5);

    hardware_interface::JointStateHandle jointStateHandle6("arm6_joint", &joint_position_[5], &joint_velocity_[5], &joint_effort_[5]);
    joint_state_interface_.registerHandle(jointStateHandle6);
    hardware_interface::JointHandle jointPositionHandle1(jointStateHandle6, &joint_position_command_[5]);
    position_joint_interface_.registerHandle(jointPositionHandle6);
    joint_limits_interface::getJointLimits("arm6_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandle6(jointPositionHandle6, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandle6);

    registerInterface(&position_joint_interface);
    registerInterface(&positionJointSaturationInterface);

    sub_arm_joint=nh_.subscribe<sensor_msgs::JointState>("presentJoint",&MyRobot::jointCallback,this);

}

MyRobot::update(const ros::TimerEvent &e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}


MyRobot::read(){
    for(int i=0;i<Dynamixel_NUM;i++){
        joint_position_[i]=joint_position_sub[i];
        joint_velocity_[i]=joint_velocity_sub[i];
        joint_effort_[i]=joint_effort_sub[i];
    }
}


MyRobot::jointCallback(const sensor_msgs::JointStateConstPtr& pose){
    for(int i=0;i<Dynamixel_NUM;i++){
        joint_position_sub[i]=*pose.position[i];
        joint_velocity_sub[i]=*pose.velocity[i];
        joint_effort_sub[i]=*pose.effort[i];
    }
}