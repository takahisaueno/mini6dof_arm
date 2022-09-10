#include <dx2lib.h>
#include <dx2memmap.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>


#define INT_MAX 2147483647
#define BAUDRATE 115200
#define Dynamixel_NUM 6

uint8_t IDs[] = {0, 1, 2, 3, 4, 5};


DX2LIB dxif (false, &Serial1);
//DX2LIB dxif (true, NULL, 10, 9);/


struct {
  int32_t maxpos, minpos;
} poslimit[Dynamixel_NUM]; //maxposition & minposition of the dynamixel


class Dynamixel_6DOF
{

  private:
    uint8_t ID;
    bool initialize_succeed_flag = 1; // if the intialization succeed:1 fail:0
    uint32_t minpose;
    uint32_t maxpose;

    int32_t pos_angle;
    bool dir_rotate;

  public:
    static uint32_t JointStateInt[Dynamixel_NUM];
    static uint32_t nextJointStateInt[Dynamixel_NUM];
    static uint32_t targetJointStateInt[Dynamixel_NUM];
    static uint32_t beforeTargetJointStateInt[Dynamixel_NUM];
    static uint32_t beforeJointStateInt_target[Dynamixel_NUM];
    static int change_JointStateInt[Dynamixel_NUM];
    static float JointState[Dynamixel_NUM];
    static float nextJointState[Dynamixel_NUM];
    



    Dynamixel_6DOF(uint8_t ID_num);
    void Torque_enable(void);
    void Blink_LED(void);
    uint32_t getmaxpose();
    uint32_t getminpose();
    void ReadAngle(void);

};


void MoveArm();
void Torque_enable(Dynamixel_6DOF motor[Dynamixel_NUM]);
void messageCb(const std_msgs::Float32MultiArray& nextJoint);
//void PubInitialState(Dynamixel_6DOF motor[Dynamixel_NUM]);
void ReadAllJointangle(Dynamixel_6DOF motor[Dynamixel_NUM]);
