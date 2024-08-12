#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

// node
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
// rcl_timer_t timer;

#define ARRAY_LEN 4
// publisher
rcl_publisher_t mt_pub1;
std_msgs__msg__Float32MultiArray pub_msg1;
rclc_executor_t executor_pub;

// subscription
rcl_subscription_t mt_sub1;
std_msgs__msg__Float32MultiArray sub_msg;
rclc_executor_t executor_sub;

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
void sub1_cb(const void *msgin)
{
  const std_msgs__msg__Float32MultiArray *sub_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  target_rad1 = sub_msg->data.data[0];
  target_rad2 = sub_msg->data.data[1];
  target_rad3 = sub_msg->data.data[2];
  target_rad4 = sub_msg->data.data[3];

  driver1.updatePID_rad(target_rad1);
  driver2.updatePID_rad(target_rad2);
  driver3.updatePID_rad(target_rad3);
  driver4.updatePID_rad(target_rad4);
  pub_msg1.data.size = ARRAY_LEN;
  pub_msg1.data.data[0] = rad1;
  pub_msg1.data.data[1] = rad2;
  pub_msg1.data.data[2] = rad3;
  pub_msg1.data.data[3] = rad4;
  RCSOFTCHECK(rcl_publish(&mt_pub1, &pub_msg1, NULL););
}

// void sub1_cb(const void *msgin){
//   const std_msgs__msg__Float32 * sub_msg = (const std_msgs__msg__Float32 *)msgin;
//   target_rad1 = sub_msg->data;
//   driver1.updatePID_rad(target_rad1);
//   pub_msg1.data = target_rad1;
//   rcl_publish(&mt_pub1, &pub_msg1, NULL);
// }

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL) {
//     pub_msg1.data.size=ARRAY_LEN;
//     pub_msg1.data.data[0] = rad1;
//     pub_msg1.data.data[1] = rad2;
//     RCSOFTCHECK(rcl_publish(&mt_pub1, &pub_msg1, NULL););
//   }
// }

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

  pub_msg1.data.data = (float *)malloc(sizeof(float) * ARRAY_LEN);
  pub_msg1.data.size = 0;
  pub_msg1.data.capacity = ARRAY_LEN;
  for (int i = 0; i < ARRAY_LEN; i++)
  {
    pub_msg1.data.data[i] = 0.0;
  }

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create ros timer
  // const unsigned int timer_timeout = 1000;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &mt_pub1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/C620/actual_rad1"));

  // create subscription
  RCCHECK(rclc_subscription_init_default(
      &mt_sub1,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/C620/target_rad1"));

  // create executor
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &mt_sub1, &sub_msg, &sub1_cb, ON_NEW_DATA));

  sub_msg.data.data = (float *)malloc(sizeof(float) * ARRAY_LEN);
  sub_msg.data.size = 0;
  sub_msg.data.capacity = ARRAY_LEN;

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
  RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}
