/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <mini6dof_arm/joint_state_msg.h>


ros::NodeHandle  nh;
std_msgs::Float32MultiArray str_msg;
//mini6dof_arm::joint_state_msg str_msg;/
//ros::Publisher chatter("chatter", &str_msg);/


void messageCb( const std_msgs::Float32MultiArray& toggle_msg){
//  str_msg.data=toggle_msg.data; /  // blink the led
  char data[150];
  sprintf(data,"%d,%d,%d,%d,%d,%d",(int)toggle_msg.data[0],(int)toggle_msg.data[1],(int)toggle_msg.data[2],(int)toggle_msg.data[3],(int)toggle_msg.data[4],(int)toggle_msg.data[5]);
  nh.loginfo(data);
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("returning", messageCb );

char letter[50];
void setup()
{
  
  nh.initNode();
//  nh.advertise(chatter);/
  nh.subscribe(sub);
}

void loop()
{
  
//  chatter.publish( &str_msg );/
//  nh.loginfo(letter);/
  nh.spinOnce();
  delay(500);
}
