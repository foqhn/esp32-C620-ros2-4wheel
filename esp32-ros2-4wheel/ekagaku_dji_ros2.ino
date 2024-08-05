
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/Float32.h>

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t ros_timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

#define ESP
#include <SPI.h>
#include <mcp2515.h>
#include "CalPID.h"
#include "C620.h"


//#########################
struct can_frame readMsg;
struct can_frame sendMsg;
MCP2515 mcp2515(5);

void calOut();
hw_timer_t * hw_timer = NULL;
void IRAM_ATTR onTimer(){
    calOut();
}
CalPID pid1(0.0025, 0.0018, 0, 0.05, 10);
C620 driver1(&mcp2515, &pid1, &sendMsg,1);
float target_rad1=0;
int pid_cnt=0;
float rad1=0;
//#########################

void calOut(){
    long start = micros();
    if(mcp2515.readMessage(&readMsg)==MCP2515::ERROR_OK){
      
        if(readMsg.can_id==0x201){
            driver1.setCANData(&readMsg);
        }
        driver1.update();
    }
    pid_cnt++;
    if(pid_cnt>=10){
        driver1.updatePID_rad(target_rad1);
        mcp2515.sendMessage(&sendMsg);
        pid_cnt=0;
    }
}

void timer_callback(rcl_timer_t * ros_timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (ros_timer != NULL) {
    msg.data=rad1;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    Serial.println(rad1);
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_arduino_node_publisher"));

    // create timer,
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
        &ros_timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &ros_timer));
    msg.data = 0;


    Serial.begin(115200);
    sendMsg.can_id = 0x200;
    sendMsg.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
        sendMsg.data[i] = 0;
    }
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    Serial.println("ready");
    delay(1000);

    //start timer
    hw_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(hw_timer, &onTimer, true);
    timerAlarmWrite(hw_timer,5000, true);
    timerAlarmEnable(hw_timer);


    Serial.println("start");
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
