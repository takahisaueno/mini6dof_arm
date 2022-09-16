#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <sensor_msgs/JointState.h>

class SimpleMoveItSubscriber
{
public:
    SimpleMoveItSubscriber();

private:
    ros::Subscriber joint_state_sub;
    void poseCallback(const sensor_msgs::JointState &pose);
};

SimpleMoveItSubscriber::SimpleMoveItSubscriber()
{
    ros::NodeHandle nh;
    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, &SimpleMoveItSubscriber::poseCallback, this);
}

void SimpleMoveItSubscriber::poseCallback(const sensor_msgs::JointState &pose)
{
    // ROS_INFO_STREAM(pose.position);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_moveit");

    SimpleMoveItSubscriber simple_moveit;
    ros::spin();
}