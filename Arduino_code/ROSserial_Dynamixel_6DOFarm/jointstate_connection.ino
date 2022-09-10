//
//
void PubInitialState(Dynamixel_6DOF motor[Dynamixel_NUM]) {
  for (int i = 0; i < Dynamixel_NUM; i++) {
    motor[i].Torque_enable();
  }
  for (int i = 0; i < Dynamixel_NUM; i++) {
    jointState.position[i] = Dynamixel_6DOF::angle_pos[i];
   jointState.header.stamp = nh.now();
  }
}
//
//
//
void ReadAllJointangle(Dynamixel_6DOF motor[Dynamixel_NUM]) {
  for (int i = 0; i < Dynamixel_NUM; i++) {
    motor[i].ReadAngle();
  }
  for (int i = 0; i < Dynamixel_NUM; i++) {
    jointState.position[i] = Dynamixel_6DOF::angle_pos[i] / 4095.0*PI;
    Serial.print(jointState.position[i]);
  }
  Serial.println("");
  jointState.header.stamp = nh.now();
}
//
void MoveArm() {
  dxif.WriteSyncLongData (ID_list, ADDRESS_X_GOAL_POSITION, (uint32_t *)&Dynamixel_6DOF::nextJointStateInt, Dynamixel_NUM);
}


void NextJointState(const sensor_msgs::JointState &nextJoint) {
  for (int i = 0; i < Dynamixel_NUM; i++) {
    Dynamixel_6DOF::nextJointStateInt[i] = nextJoint.position[i]/PI*4095;
  }
  MoveArm();
//}
