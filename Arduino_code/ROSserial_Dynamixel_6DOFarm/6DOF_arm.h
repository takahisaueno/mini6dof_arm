#include <dx2lib.h>
#include <dx2memmap.h>
#include <SoftwareSerial.h>
//#include <ros.h>
//#include <sensor_msgs/JointState.h>
//#include <std_msgs/String.h>
#define BAUDRATE 57600
#define Dynamixel_NUM 6
//#define PI 3.1415926535

DX2LIB dxif(false,&Serial1);


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
    static uint32_t angle_pos[Dynamixel_NUM];
    static uint32_t nextJointStateInt[Dynamixel_NUM];


    Dynamixel_6DOF(uint8_t ID_num);
    void Torque_enable(void);
    void Blink_LED(void);
    uint32_t getmaxpose(void);
    uint32_t getminpose(void);
    void ReadAngle(void);

};


//void NextJointState(const sensor_msgs::JointState& nextJoint);
//void PubInitialState(Dynamixel_6DOF motor[Dynamixel_NUM]);
//void ReadAllJointangle(Dynamixel_6DOF motor[Dynamixel_NUM]);
void MoveArm();
