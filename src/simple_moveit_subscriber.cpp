#include <ros/ros.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <sensor_msgs/JointState.h>
#include "dynamixel_control.h"
#include <time.h>
#include "dynamixel_control/pid_parameter.h"

/*-------------------------------SimpleMoveItSubscriber----------------------------------------------*/
class SimpleMoveItSubscriber
{
public:
    SimpleMoveItSubscriber(bool mode);

private:
    ros::Subscriber joint_state_sub;
    ros::Publisher present_state_pub;
    void poseCallback(const sensor_msgs::JointStateConstPtr &pose);
    void send_initial_parameter();
    bool pid_parameter_changer(dynamixel_control::pid_parameter::Request &req, dynamixel_control::pid_parameter::Response &res);
    void set_pid_gain();
    DynamixelControl dynamixel_ctrl;
    float target_joint_angle[Dynamixel_NUM];
    float present_joint_angle[Dynamixel_NUM];
    sensor_msgs::JointState present_joint_msgs;

    uint16_t present_pwm[Dynamixel_NUM];
    uint16_t present_load[Dynamixel_NUM];

    uint32_t prof_acc[Dynamixel_NUM];
    uint32_t prof_vel[Dynamixel_NUM];

    void pubAngle();

    ros::ServiceServer PID_parameter_server;
    uint16_t target_p_gain[Dynamixel_NUM];
    uint16_t target_i_gain[Dynamixel_NUM];
    uint16_t target_d_gain[Dynamixel_NUM];

    uint16_t present_p_gain[Dynamixel_NUM];
    uint16_t present_i_gain[Dynamixel_NUM];
    uint16_t present_d_gain[Dynamixel_NUM];

    bool pid_parameter_toggle;
    bool initial_parameter_change;
    uint8_t changing_pid_parameter; // 0: nothing 1: intial_mode 2: setting PID mode

    ros::ServiceClient pid_parameter_initial_set_client;
};

/*--------------------------------SimpleMoveItSubscriber class method----------------------------------*/

SimpleMoveItSubscriber::SimpleMoveItSubscriber(bool mode) : pid_parameter_toggle(mode), initial_parameter_change(true), changing_pid_parameter(0)
{
    bool toggle;
    ros::NodeHandle nh;
    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("arm_robot/joint_states", 10, &SimpleMoveItSubscriber::poseCallback, this);
    present_state_pub = nh.advertise<sensor_msgs::JointState>("arm_robot/present_joint_states", 1);
    toggle = dynamixel_ctrl.toggleTorqueMode(1);
    for (int i = 0; i < Dynamixel_NUM; i++)
    {
        prof_acc[i] = 80;
        prof_vel[i] = 80;
    }
    dynamixel_ctrl.setProfile_acc(prof_acc);
    dynamixel_ctrl.setProfile_vel(prof_vel);

    if (pid_parameter_toggle)
    {
        PID_parameter_server = nh.advertiseService("arm_robot/pid_parameter", &SimpleMoveItSubscriber::pid_parameter_changer, this);
        pid_parameter_initial_set_client = nh.serviceClient<dynamixel_control::pid_parameter>("arm_robot/initial_pid_parameter");
        pid_parameter_initial_set_client.waitForExistence();
    }
}

void SimpleMoveItSubscriber::poseCallback(const sensor_msgs::JointStateConstPtr &pose)
{
    bool error_flag = false;
    present_joint_msgs = *pose;
    for (int i = 0; i < Dynamixel_NUM; i++)
    {
        target_joint_angle[i] = pose->position[i];
        if (target_joint_angle[i] == 0.0)
        {
            error_flag = true;
        }
    }
    if (!error_flag)
    {
        dynamixel_ctrl.moveToAngle(target_joint_angle);
        // ROS_INFO_STREAM("Target joint angle is:"
        //                 << "[" << pose->position[0] << "," << pose->position[1] << "," << pose->position[2] << "," << pose->position[3] << "," << pose->position[4] << "," << pose->position[5] << "]");
        dynamixel_ctrl.getAngle(present_joint_angle);
        pubAngle();
        set_pid_gain();

        // ROS_INFO_STREAM("Present joint angle is: [" << present_joint_angle[0] << "," << present_joint_angle[1] << "," << present_joint_angle[2] << "," << present_joint_angle[3] << "," << present_joint_angle[4] << "," << present_joint_angle[5] << "]");
        // dynamixel_ctrl.getPWM(present_pwm);
        // ROS_INFO_STREAM("Present joint PWM is: [" << present_pwm[0] << "," << present_pwm[1] << "," << present_pwm[2] << "," << present_pwm[3] << "," << present_pwm[4] << "," << present_pwm[5] << "]");
        // dynamixel_ctrl.getLoad(present_load);
        // ROS_INFO_STREAM("Present joint LOAD is: [" << present_load[0] << "," << present_load[1] << "," << present_load[2] << "," << present_load[3] << "," << present_load[4] << "," << present_load[5] << "]");
    }
}

bool SimpleMoveItSubscriber::pid_parameter_changer(dynamixel_control::pid_parameter::Request &req, dynamixel_control::pid_parameter::Response &res)
{
    ROS_INFO_STREAM("Got the service");

    if (initial_parameter_change)
    {

        // dynamixel_ctrl.getPosPID(present_p_gain, present_i_gain, present_d_gain);





        //この上のdynamixelのPIDゲインを取得する部分は直さないと行けない　問題：なぜか値を取得できない2022/10/9
        for (int i = 0; i < Dynamixel_NUM; i++)
        {
            present_p_gain[i] = 800;
            present_i_gain[i] = 0;
            present_d_gain[i] = 0;
        }
        ROS_INFO_STREAM(present_p_gain[0] << "," << present_p_gain[1] << "," << present_p_gain[2] << "," << present_p_gain[3] << "," << present_p_gain[4] << "," << present_p_gain[5]);
        ROS_INFO_STREAM(present_i_gain[0] << "," << present_i_gain[1] << "," << present_i_gain[2] << "," << present_i_gain[3] << "," << present_i_gain[4] << "," << present_i_gain[5]);
        ROS_INFO_STREAM(present_d_gain[0] << "," << present_d_gain[1] << "," << present_d_gain[2] << "," << present_d_gain[3] << "," << present_d_gain[4] << "," << present_d_gain[5]);

        changing_pid_parameter = 1;
    }
    else
    {

        for (int i = 0; i < Dynamixel_NUM; i++)
        {
            target_p_gain[i] = req.position_p_gain[i];
            target_i_gain[i] = req.position_i_gain[i];
            target_d_gain[i] = req.position_d_gain[i];
            ROS_INFO_STREAM(target_p_gain[0] << "," << target_p_gain[1] << "," << target_p_gain[2] << "," << target_p_gain[3] << "," << target_p_gain[4] << "," << target_p_gain[5]);
            ROS_INFO_STREAM(target_i_gain[0] << "," << target_i_gain[1] << "," << target_i_gain[2] << "," << target_i_gain[3] << "," << target_i_gain[4] << "," << target_i_gain[5]);
            ROS_INFO_STREAM(target_d_gain[0] << "," << target_d_gain[1] << "," << target_d_gain[2] << "," << target_d_gain[3] << "," << target_d_gain[4] << "," << target_d_gain[5]);
        }

        changing_pid_parameter = 2;
    }
    res.pid_change = true;
    return true;
}

void SimpleMoveItSubscriber::set_pid_gain()
{

    if (changing_pid_parameter == 2)
    {
        dynamixel_ctrl.setPosPID(target_p_gain, target_i_gain, target_d_gain);
        changing_pid_parameter = 0;
    }
    else if (changing_pid_parameter == 1)
    {
        send_initial_parameter();
        initial_parameter_change = false;
        changing_pid_parameter = 0;
    }
}

void SimpleMoveItSubscriber::pubAngle()
{
    bool error_flag = false;

    present_joint_msgs.header.stamp = ros::Time::now();
    for (int i = 0; i < Dynamixel_NUM; i++)
    {
        present_joint_msgs.position[i] = present_joint_angle[i];
        if (present_joint_msgs.position[i] == 0.0)
        {
            error_flag = true;
        }
    }
    if (!error_flag)
    {
        present_state_pub.publish(present_joint_msgs);
    }
}

void SimpleMoveItSubscriber::send_initial_parameter()
{
    dynamixel_control::pid_parameter parameter_msg;
    parameter_msg.request.position_p_gain.resize(Dynamixel_NUM);
    parameter_msg.request.position_i_gain.resize(Dynamixel_NUM);
    parameter_msg.request.position_d_gain.resize(Dynamixel_NUM);

    for (int i = 0; i < Dynamixel_NUM; i++)
    {
        parameter_msg.request.position_p_gain[i] = present_p_gain[i];
        parameter_msg.request.position_i_gain[i] = present_i_gain[i];
        parameter_msg.request.position_d_gain[i] = present_d_gain[i];

    }

    parameter_msg.request.present_time = ros::Time::now();

    if (pid_parameter_initial_set_client.call(parameter_msg))
    {
        if (parameter_msg.response.pid_change)
        {
            ROS_INFO_STREAM("Succeed to change the PID gain on rqt_reconfigure");
        }
        else
        {
            ROS_INFO_STREAM("Failed to change the PID gain on rqt_reconfigure");
        }
    }
    else
    {
        ROS_INFO_STREAM("Failed to call the service");
    }
}

/*-------------------------------------main------------------------------------------*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_moveit");
    std::unique_ptr<SimpleMoveItSubscriber> simple_moveit;
    if (argc < 2)
    {
        ROS_INFO_STREAM("This mode will use the initial pid parameter");
        simple_moveit.reset(new SimpleMoveItSubscriber(false));
    }
    else
    {
        std::string pid_flag_arg = argv[1];
        if (pid_flag_arg.compare("true") == 0)
        {
            ROS_INFO_STREAM("This mode will do the PID parameter tuning");
            simple_moveit.reset(new SimpleMoveItSubscriber(true));
        }
        else if (pid_flag_arg.compare("false") != 0)
        {
            ROS_INFO_STREAM("This mode will use the initial pid parameter");
            simple_moveit.reset(new SimpleMoveItSubscriber(false));
        }
        else
        {
            while (1)
            {
                ROS_INFO_STREAM("place proper argument");
                ROS_INFO_STREAM(argv[1]);
            }
        }
    }

    ros::AsyncSpinner spinner(3);
    spinner.start();
    while (ros::ok())
    {
    }
    spinner.stop();
    portHandler->closePort();
    return 0;
}