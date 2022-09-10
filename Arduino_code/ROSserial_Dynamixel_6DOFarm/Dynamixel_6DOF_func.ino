


uint32_t Dynamixel_6DOF::angle_pos[Dynamixel_NUM];
uint32_t Dynamixel_6DOF::nextJointStateInt[Dynamixel_NUM];

Dynamixel_6DOF::Dynamixel_6DOF(uint8_t ID_num) {

  ID = ID_num;
  Serial.print(ID_num);

  dxif.ReadBlockData (ID, ADDRESS_X_MAX_POSITION_LIMIT, (uint8_t *)&poslimit[ID], 4, NULL);
  minpose=poslimit[ID].minpos;
  maxpose=poslimit[ID].maxpos;
  

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

uint32_t Dynamixel_6DOF::getminpose(void){
  return minpose;
  }
uint32_t Dynamixel_6DOF::getmaxpose(void){
  return maxpose;
  }

void Dynamixel_6DOF::ReadAngle(void){
  dxif.ReadLongData (ID, ADDRESS_X_PRESENT_POSITION, (uint32_t *)&angle_pos[ID], NULL);
  }


  
