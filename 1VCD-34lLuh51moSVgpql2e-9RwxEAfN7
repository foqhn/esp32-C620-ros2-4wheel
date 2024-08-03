#define ESP
#include <SPI.h>
#include <mcp2515.h>
#include "CalPID.h"
#include "C620.h"

struct can_frame readMsg;
struct can_frame sendMsg;
MCP2515 mcp2515(5);

#ifdef ESP
hw_timer_t * timer = NULL;
void IRAM_ATTR onTimer(){
    calOut();
}
#else

#endif

CalPID pid1(0.0025, 0.0018, 0, 0.05, 10);
CalPID pid2(0.0025, 0.0018, 0, 0.05, 10);
CalPID pid3(0.0025, 0.0018, 0, 0.05, 10);
CalPID pid4(0.0025, 0.0018, 0, 0.05, 10);

C620 driver1(&mcp2515, &pid1, &sendMsg,1);
C620 driver2(&mcp2515, &pid2, &sendMsg,2);
C620 driver3(&mcp2515, &pid3, &sendMsg,3);
C620 driver4(&mcp2515, &pid4, &sendMsg,4);

float target_rad1=0;
float target_rad2=0;
float target_rad3=0;
float target_rad4=0;

int output_cnt=0;
int pid_cnt=0;

void calOut(){
    long start = micros();
    if(mcp2515.readMessage(&readMsg)==MCP2515::ERROR_OK){
      
        if(readMsg.can_id==0x201){
            driver1.setCANData(&readMsg);
        }else if(readMsg.can_id==0x202){
            driver2.setCANData(&readMsg);
        }else if(readMsg.can_id==0x203){
            driver3.setCANData(&readMsg);
        }else if(readMsg.can_id==0x204){
            driver4.setCANData(&readMsg);
        }

        driver1.update();
        driver2.update();
        driver3.update();
        driver4.update();
    }
    pid_cnt++;
    if(pid_cnt>=10){
        driver1.updatePID_rad(target_rad1);
        driver2.updatePID_rad(target_rad2);
        driver3.updatePID_rad(target_rad3);
        driver4.updatePID_rad(target_rad4);
        mcp2515.sendMessage(&sendMsg);
        pid_cnt=0;
    }
    output_cnt++;
    if(output_cnt>=40){
        float rad1 = driver1.readRad_s();
        float rad2 = driver2.readRad_s();
        float rad3 = driver3.readRad_s();
        float rad4 = driver4.readRad_s();

        Serial.print(rad1);
        Serial.print(",");
        Serial.print(rad2);
        Serial.print(",");
        Serial.print(rad3);
        Serial.print(",");
        Serial.println(rad4);
        output_cnt=0;

    }
    
}
//ros include
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32multiarray.h>

rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
    while(1){
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

void subscription_callback(const void * msgin)
{  
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    target_rad1 = msg->data.data[0];
    target_rad2 = msg->data.data[1];
    target_rad3 = msg->data.data[2];
    target_rad4 = msg->data.data[3];
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

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "micro_ros_arduino_subscriber"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // add subscriber to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));


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
    #ifdef STM
    my_timer.pause();
    my_timer.setPrescaleFactor(1680);
    my_timer.setOverflow(500);
    my_timer.attachInterrupt(calOut);
    my_timer.refresh();
    my_timer.resume();
    #endif
    #ifdef ESP
    //definition: timerBegin(uint32_t frequency
    // 5ms period
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer,5000, true);
    timerAlarmEnable(timer);
    #else
    MsTimer2::set(25, calOut);
    MsTimer2::start();
    #endif

    Serial.println("start");
}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
