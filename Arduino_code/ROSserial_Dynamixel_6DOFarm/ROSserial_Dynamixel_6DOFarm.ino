#include "6DOF_arm.h"




/*-------------------------------------------------------------------*/
uint8_t ID_list[] = {0, 1, 2, 3, 4, 5};


//
//sensor_msgs::JointState jointState;
//sensor_msgs::JointState nextJointState;
//std_msgs::String str_msg;

//ros::NodeHandle nh;/
//ros::Subscriber<sensor_msgs::JointState> sub("nextjoint", &NextJoi/ntState );
//ros::Publisher presentJointState("presentJoint", &jointState);/
//ros::Publisher chatter("chatter", &str_msg);

Dynamixel_6DOF motor[Dynamixel_NUM] = {0, 1, 2, 3, 4, 5};
//char hello[13] = "hello world!";/
/*-------------------------------------------------------------------*/

void setup() {

  dxif.begin(BAUDRATE);
  Serial.begin(9600);
  //  nh.getHardware()->setBaud(57600);
//  nh.initNode();/
  //  nh.advertise(presentJointState/);
  //  nh.advertise(chatter);
  //  nh.subscribe(sub);/
}

void loop() {


  //  PubInitialState(motor);/
//  while (1) {
////    ReadAllJointangle(motor);
//    
//    //    presentJointState.publish(&jointState);/
//    //    nh.spinOnce();
//    //    str_msg.data = hello;
//    //    chatter.publish( &str_msg );
//
//    delay(30);
//  }

int dir[6] = {50, 50, 50, 50, 50, 50};
  for (int i = 0; i < Dynamixel_NUM; i++) {
    motor[i].Torque_enable();
  }
  while (1) {
    for (int i = 0; i < Dynamixel_NUM; i++) {
      if (Dynamixel_6DOF::nextJointStateInt[i] > motor[i].getmaxpose() || Dynamixel_6DOF::nextJointStateInt[i] < motor[i].getminpose()) {
        dir[i] = -1 * dir[i];
      }

      Dynamixel_6DOF::nextJointStateInt[i] = Dynamixel_6DOF::nextJointStateInt[i] + dir[i];
    }
    MoveArm();
  }


}
