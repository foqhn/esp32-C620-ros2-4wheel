#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float64_multi_array.h>

// node
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
// rcl_timer_t timer;

#define ARRAY_LEN 2
// publisher1
rcl_publisher_t left_mt_pub;
std_msgs__msg__Float64MultiArray left_pub_msg;
rclc_executor_t executor_pub_left;

// publisher2
rcl_publisher_t right_mt_pub;
std_msgs__msg__Float64MultiArray right_pub_msg;
rclc_executor_t executor_pub_right;

// subscription1
rcl_subscription_t left_mt_sub;
std_msgs__msg__Float64MultiArray left_sub_msg;
rclc_executor_t executor_sub_left;

// subscription2
rcl_subscription_t right_mt_sub;
std_msgs__msg__Float64MultiArray right_sub_msg;
rclc_executor_t executor_sub_right;

#define LED_PIN 13

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

#define ESP
#include <SPI.h>
#include <mcp2515.h>
#include "CalPID.h"
#include "C620.h"

// #########################
struct can_frame readMsg;
struct can_frame sendMsg;
MCP2515 mcp2515(5);

void calOut();
hw_timer_t *hw_timer = NULL;
void IRAM_ATTR onTimer()
{
  calOut();
}
CalPID pid1(0.0025, 0.0018, 0, 0.05, 10);
C620 driver1(&mcp2515, &pid1, &sendMsg, 1);
C620 driver2(&mcp2515, &pid1, &sendMsg, 2);
C620 driver3(&mcp2515, &pid1, &sendMsg, 3);
C620 driver4(&mcp2515, &pid1, &sendMsg, 4);
float target_rad1;
float target_rad2;
float target_rad3;
float target_rad4;
int pid_cnt = 0;
int output_cnt = 0;
float rad1;
float rad2;
float rad3;
float rad4;
// #########################
void left_sub_cb(const void *msgin)
{
  const std_msgs__msg__Float64MultiArray *left_sub_msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  target_rad1 = left_sub_msg->data.data[0];
  target_rad2 = left_sub_msg->data.data[1];

  driver1.updatePID_rad(target_rad1);
  driver2.updatePID_rad(target_rad2);


  //RPMフィードバックがここに来るのは本来避けたい　が、現状これ以外動作しない
  left_pub_msg.data.size = ARRAY_LEN;
  left_pub_msg.data.data[0] = rad1;
  left_pub_msg.data.data[1] = rad2;
  RCSOFTCHECK(rcl_publish(&left_mt_pub, &left_pub_msg, NULL););
}

void right_sub_cb(const void *msgin)
{
  const std_msgs__msg__Float64MultiArray *right_sub_msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  target_rad3 = right_sub_msg->data.data[0];
  target_rad4 = right_sub_msg->data.data[1];

  driver3.updatePID_rad(target_rad3);
  driver4.updatePID_rad(target_rad4);

  //RPMフィードバックがここに来るのは本来避けたい　が、現状これ以外動作しない
  right_pub_msg.data.size = ARRAY_LEN;
  right_pub_msg.data.data[0] = rad3;
  right_pub_msg.data.data[1] = rad4;
  RCSOFTCHECK(rcl_publish(&right_mt_pub, &right_pub_msg, NULL););
}




void calOut()
{
  if (mcp2515.readMessage(&readMsg) == MCP2515::ERROR_OK)
  {

    if (readMsg.can_id == 0x201)
    {
      driver1.setCANData(&readMsg);
    }
    else if (readMsg.can_id == 0x202)
    {
      driver2.setCANData(&readMsg);
    }
    else if (readMsg.can_id == 0x203)
    {
      driver3.setCANData(&readMsg);
    }
    else if (readMsg.can_id == 0x204)
    {
      driver4.setCANData(&readMsg);
    }
    driver1.update();
    driver2.update();
    driver3.update();
    driver4.update();
  }
  pid_cnt++;
  if (pid_cnt >= 10)
  {
    driver1.updatePID_rad(target_rad1);
    driver2.updatePID_rad(target_rad2);
    driver3.updatePID_rad(target_rad3);
    driver4.updatePID_rad(target_rad4);
    mcp2515.sendMessage(&sendMsg);
    pid_cnt = 0;
  }
  output_cnt++;
  if (output_cnt >= 40)
  {
    rad1 = driver1.readRad_s();
    rad2 = driver2.readRad_s();
    rad3 = driver3.readRad_s();
    rad4 = driver4.readRad_s();
    // HWタイマーの割り込み内に入れると、再起動ループに入る
    // pub_msg1.data.size=ARRAY_LEN;
    // pub_msg1.data.data[0] = rad1;
    // pub_msg1.data.data[1] = rad2;
    // RCSOFTCHECK(rcl_publish(&mt_pub1, &pub_msg1, NULL););
    output_cnt = 0;
  }
}

void setup()
{
  Serial.begin(115200);
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  left_pub_msg.data.data = (double *)malloc(sizeof(double) * ARRAY_LEN);
  left_pub_msg.data.size = 0;
  left_pub_msg.data.capacity = ARRAY_LEN;

  right_pub_msg.data.data = (double *)malloc(sizeof(double) * ARRAY_LEN);
  right_pub_msg.data.size = 0;
  right_pub_msg.data.capacity = ARRAY_LEN;
  for (int i = 0; i < ARRAY_LEN; i++)
  {
    left_pub_msg.data.data[i] = 0.0;
    right_pub_msg.data.data[i] = 0.0;
  }

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  //タイマーを使うと、ノードが起動しない
  // create ros timer
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  // create publisher1
  RCCHECK(rclc_publisher_init_best_effort(
      &left_mt_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/C620/actual_rad1"));

  // create publisher2
  RCCHECK(rclc_publisher_init_best_effort(
      &right_mt_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/C620/actual_rad2"));


  // create subscription1
  RCCHECK(rclc_subscription_init_default(
      &left_mt_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/C620/target_rad1"));

  // create subscription2
  RCCHECK(rclc_subscription_init_default(
      &right_mt_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/C620/target_rad2"));

  // create executor
  RCCHECK(rclc_executor_init(&executor_sub_left, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_sub_right, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_pub_left, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_pub_right, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor_sub_left, &left_mt_sub, &left_sub_msg, &left_sub_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_sub_right, &right_mt_sub, &right_sub_msg, &right_sub_cb, ON_NEW_DATA));



  left_sub_msg.data.data = (double *)malloc(sizeof(double) * ARRAY_LEN);
  left_sub_msg.data.size = 0;
  left_sub_msg.data.capacity = ARRAY_LEN;

  right_sub_msg.data.data = (double *)malloc(sizeof(double) * ARRAY_LEN);
  right_sub_msg.data.size = 0;
  right_sub_msg.data.capacity = ARRAY_LEN;
  

  sendMsg.can_id = 0x200;
  sendMsg.can_dlc = 8;
  for (int i = 0; i < 8; i++)
  {
    sendMsg.data[i] = 0;
  }
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  delay(1000);

  // start timer
  hw_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(hw_timer, &onTimer, true);
  timerAlarmWrite(hw_timer, 5000, true);
  timerAlarmEnable(hw_timer);

  // Serial.println("start");
}

void loop()
{
  delay(100);

  rclc_executor_spin_some(&executor_sub_left, 100);
  rclc_executor_spin_some(&executor_sub_right, 100);
  rclc_executor_spin_some(&executor_pub_left, 100);
  rclc_executor_spin_some(&executor_pub_right, 100);

}
