
void MoveArm() {
  for (int i = 0; i < Dynamixel_NUM; i++) {
    if (moving_counter_after_callback >= moving_counter_limit) {
      Dynamixel_6DOF::nextJointStateInt[i] = Dynamixel_6DOF::targetJointStateInt[i];
    }
    else {
      Dynamixel_6DOF::nextJointStateInt[i] = Dynamixel_6DOF::change_JointStateInt[i] * (moving_counter_after_callback + 1) + Dynamixel_6DOF::beforeJointStateInt_target[i];
    }

    if ((Dynamixel_6DOF::beforeTargetJointStateInt[i] < Dynamixel_6DOF::targetJointStateInt[i]) && (Dynamixel_6DOF::nextJointStateInt[i] > Dynamixel_6DOF::targetJointStateInt[i])) {
      Dynamixel_6DOF::nextJointStateInt[i] = Dynamixel_6DOF::targetJointStateInt[i];
    }
    else if ((Dynamixel_6DOF::beforeTargetJointStateInt[i] > Dynamixel_6DOF::targetJointStateInt[i]) && (Dynamixel_6DOF::nextJointStateInt[i] < Dynamixel_6DOF::targetJointStateInt[i])) {
      Dynamixel_6DOF::nextJointStateInt[i] = Dynamixel_6DOF::targetJointStateInt[i];
    }
    if (subscriber_flag == 0) {
      Dynamixel_6DOF::nextJointStateInt[i] = Dynamixel_6DOF::targetJointStateInt[i];
    }
    
  }
  dxif.WriteSyncLongData (IDs, ADDRESS_X_GOAL_POSITION, (uint32_t *)&Dynamixel_6DOF::nextJointStateInt, Dynamixel_NUM);
  move_time_before = move_time_current;
  move_time_current = millis();
  moving_counter_after_callback = moving_counter_after_callback + 1;
}


void Torque_enable(Dynamixel_6DOF motor[Dynamixel_NUM]) {
  for (int i = 0; i < Dynamixel_NUM; i++) {
    motor[i].Torque_enable();
  }
}


void messageCb(const std_msgs::Float32MultiArray& nextJoint) {
  moving_counter_after_callback = 0;


  if (subscriber_flag == 1) {
    for (int i = 0; i < Dynamixel_NUM; i++) {
      Dynamixel_6DOF::beforeJointStateInt_target[i] = Dynamixel_6DOF::JointStateInt[i];
    }
  }

  for (int i = 0; i < Dynamixel_NUM; i++) {
    if (i == 2 || i == 4) {
      Dynamixel_6DOF::targetJointStateInt[i] = (-1.0 * nextJoint.data[i] / (2 * PI) * 4095 + 2048);
    }
    else {
      Dynamixel_6DOF::targetJointStateInt[i] = (nextJoint.data[i] / (2 * PI) * 4095 + 2048);
    }
  }
  sub_time_before = sub_time_current;
  sub_time_current = millis();

  if (subscriber_flag == 1) {

    for (int i; i < Dynamixel_NUM; i++) {
      Dynamixel_6DOF::change_JointStateInt[i] = (int)Dynamixel_6DOF::targetJointStateInt[i] - (int)Dynamixel_6DOF::JointStateInt[i];
      Dynamixel_6DOF::change_JointStateInt[i] = Dynamixel_6DOF::change_JointStateInt[i] / ((int)(sub_time_current - sub_time_before) / (int)(move_time_current - move_time_before));
    }
    moving_counter_limit = (float)(sub_time_current - sub_time_before) / (float)(move_time_current - move_time_before);
  }
  else {
    moving_counter_limit = 0;
    subscriber_flag = 1;
  }
  if (moving_counter_limit > 5) {
    moving_counter_limit = 4;
  }
  char v[10];
  sprintf(v, "%d", moving_counter_limit);
  nh.loginfo(v);
}

void ReadAllJointangle(Dynamixel_6DOF motor[Dynamixel_NUM]) {
  for (int i = 0; i < Dynamixel_NUM; i++) {
    motor[i].ReadAngle();
  }
  jointState.position_length = Dynamixel_NUM;
  jointState.velocity_length = Dynamixel_NUM;
  jointState.effort_length = Dynamixel_NUM;

  for (int i = 0; i < Dynamixel_NUM; i++) {
    Dynamixel_6DOF::JointState[i] = ((int)Dynamixel_6DOF::JointStateInt[i] - 2048.0) / 4095.0 * 2 * PI;
    if (i == 2 || i == 4) {
      Dynamixel_6DOF::JointState[i] = -1.0 * Dynamixel_6DOF::JointState[i];
    }
  }

  jointState.position = Dynamixel_6DOF::JointState;

  jointState.header.stamp = nh.now();
}



//void PubInitialState(Dynamixel_6DOF motor[Dynamixel_NUM]) {
//  for (int i = 0; i < Dynamixel_NUM; i++) {
//    motor[i].Torque_enable();
//  }
//  for (int i = 0; i < Dynamixel_NUM; i++) {
//    jointState.position[i] = Dynamixel_6DOF::JointStateInt[i];
//    jointState.header.stamp = nh.now();
//  }
//}
