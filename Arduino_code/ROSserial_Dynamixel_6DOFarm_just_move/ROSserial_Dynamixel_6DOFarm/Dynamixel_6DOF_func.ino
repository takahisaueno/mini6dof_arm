


uint32_t Dynamixel_6DOF::JointStateInt[Dynamixel_NUM] = {0};
float Dynamixel_6DOF::JointState[Dynamixel_NUM] = {0};
uint32_t Dynamixel_6DOF::targetJointStateInt[Dynamixel_NUM] = {0};
uint32_t Dynamixel_6DOF::beforeJointStateInt_target[Dynamixel_NUM] = {0};
uint32_t Dynamixel_6DOF::beforeTargetJointStateInt[Dynamixel_NUM] = {0};
uint32_t Dynamixel_6DOF::nextJointStateInt[Dynamixel_NUM] = {0};
int Dynamixel_6DOF::change_JointStateInt[Dynamixel_NUM] = {0};
float Dynamixel_6DOF::nextJointState[Dynamixel_NUM] = {0};



Dynamixel_6DOF::Dynamixel_6DOF(uint8_t ID_num) {

  ID = ID_num;
  delay(20);
  dxif.ReadBlockData (ID, ADDRESS_X_MAX_POSITION_LIMIT, (uint8_t *)&poslimit[ID], 4, NULL);
  minpose = poslimit[ID].minpos;
  maxpose = poslimit[ID].maxpos;


  Dynamixel_6DOF::nextJointStateInt[ID] = (minpose + maxpose) / 2;
  Dynamixel_6DOF::targetJointStateInt[ID] = (minpose + maxpose) / 2;
  ReadAngle();
}


void Dynamixel_6DOF::Torque_enable(void) {
  dxif.WriteByteData (ID, ADDRESS_X_TORQUE_ENABLE, 1, NULL);
}


void Dynamixel_6DOF::Blink_LED(void) {

  dxif.WriteByteData (ID, ADDRESS_X_LED_RED, 1, NULL);
  delay(500);
  dxif.WriteByteData (ID, ADDRESS_X_LED_RED, 0, NULL);
  delay(500);
}

uint32_t Dynamixel_6DOF::getminpose() {
  return minpose;
}
uint32_t Dynamixel_6DOF::getmaxpose() {
  return maxpose;
}

void Dynamixel_6DOF::ReadAngle(void) {
  dxif.ReadLongData (ID, ADDRESS_X_PRESENT_POSITION, (uint32_t *)&JointStateInt[ID], NULL);

}
