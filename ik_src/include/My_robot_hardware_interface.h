#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define Dynamixel_NUM 6

class MyRobot : public hardware_interface::RobotHW 
{
    public:
        MyRobot(ros::NodeHandle& nh);
        ~MyRobot();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        
    protected:
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        

        double joint_position_[6];
        double joint_velocity_[6];
        double joint_effort_[6];
        double joint_position_command_[6];

        double joint_position_sub[6];
        double joint_velocity_sub[6];
        double joint_effort_sub[6];

        
        ros::NodeHandle nh_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

    private:
        ros::Subscriber sub_arm_joint;

};