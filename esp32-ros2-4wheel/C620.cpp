#include "C620.h"

C620::C620(MCP2515* mcp2515, CalPID* pid, can_frame* sendMsg, int id) {
  mcp2515_ = mcp2515;
  pid_ = pid;
  id_ = id;
  data_receive_error_ = false;
  rpm_ = 0;
  current_ = 0;
  angle_ = 0;
  sendMsg_ = sendMsg;
  start_counter_ = 0;
  motor_stopping_ = false;
}

int C620::connectBytes(uint8_t upper, uint8_t lower) {
  return (int) (upper << 8) + lower;
}

void C620::setCurrent(can_frame *frame, int motor_id, float current) {
  int encoded_current = (int) (16384 * current / 20);
  uint8_t bytes[2];
  
  divideBytes(encoded_current, bytes);
  if(motor_id < 5) {
    for(int i=0; i<2; i++) {
      frame->data[(motor_id-1)*2+i] = bytes[i];
    }
  } else {
    for(int i=0; i<2; i++) {
      frame->data[(motor_id-5)*2+i] = bytes[i];
    }
  }
}

void C620::divideBytes(uint16_t val, uint8_t *data_array_ptr) {
  uint8_t *pointer = data_array_ptr;
  *pointer = 0xFF & (val >> 8);
  *(pointer+1) = 0xFF & val;
}

float C620::angle(can_frame *frame) {
  return (float) 360 * ((float) connectBytes(frame->data[0], frame->data[1]) / 8191);
}

int C620::RPM(can_frame *frame) {
  short read = connectBytes(frame->data[2], frame->data[3]);
  return (int) read;
}

float C620::current(can_frame *frame) {
  return (float) 20 * ((float) connectBytes(frame->data[4], frame->data[5]) / 16384);
}

void C620::stopMotor() {
  for(int i=0; i<8; i++) {
    sendMsg_->data[i] = 0;
  }
  mcp2515_->sendMessage(sendMsg_);
}

void C620::updatePID(int target_rpm) {
  /*if((target_rpm > STOP_THRESHOLD)&&(rpm_ < STOP_THRESHOLD)) {
    motor_stopping_ = true;
  }
  if(motor_stopping_){
    setCurrent(sendMsg_, id_, START_CURRENT);
    start_counter_++;
    if(start_counter_ > 59) {
      start_counter_ = 0;
      motor_stopping_ = false;
    }
  } else {*/
    float error = (float) target_rpm - rpm_;
    float new_current_ = pid_->calPID(error);
    setCurrent(sendMsg_, id_, new_current_);
  //}
}

void C620::updatePID_rad(float target_rad_s) {
  // rad/sで指定、出力軸の速度なのでギア比がかかっているのに注意
  float target_rpm = (int) target_rad_s * GEAR_RATIO * 60 / (2*PI);
  this->updatePID(target_rpm);
}

void C620::update() {
  angle_ = angle(&readMsg_);
  rpm_ = RPM(&readMsg_);
  current_ = current(&readMsg_);
}

void C620::setCANData(can_frame* frame) {
  //readMsg_ = *frame;
  for(int i=0; i<8; i++) {
    readMsg_.data[i] = frame->data[i];
  }
}

void C620::transfer() {
  mcp2515_->sendMessage(sendMsg_);
}

void C620::recoverError() {
  data_receive_error_ = false;
}

float C620::readRad_s() {
  float rad = (float) rpm_ * 2 * PI / (60*GEAR_RATIO);
  return rad;
}

int C620::readRPM() {
  return rpm_;
}
float C620::readAngle() {
  return angle_;
}
float C620::readCurrent() {
  return current_;
}