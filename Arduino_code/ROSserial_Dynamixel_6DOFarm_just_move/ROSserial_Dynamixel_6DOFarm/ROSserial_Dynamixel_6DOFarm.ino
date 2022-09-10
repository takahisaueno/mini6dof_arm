#include "6DOF_arm.h"

/*---------------------------------------*/

ros::NodeHandle nh;
sensor_msgs::JointState jointState;


/*---------------linear divide angle joint-----------------*/
unsigned long move_time_before=0;
unsigned long move_time_current=0;
int moving_counter_after_callback=0;
int moving_counter_limit=0;
unsigned long sub_time_before=0;
unsigned long sub_time_current=0;



ros::Subscriber<std_msgs::Float32MultiArray> sub("returning", messageCb );

bool subscriber_flag=0;
ros::Publisher presentJointState("presentJoint", &jointState);


/*---------------------------------------*/

void setup() {
  // put your setup code here, to run once:
  dxif.begin(BAUDRATE);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(presentJointState);
  nh.subscribe(sub);
}

void loop() {
  Dynamixel_6DOF motor[Dynamixel_NUM] = {0, 1, 2, 3, 4, 5};

  Torque_enable(motor);
  while (1) {
    ReadAllJointangle(motor);
    presentJointState.publish(&jointState);
    nh.spinOnce();
    MoveArm();
  }

}
