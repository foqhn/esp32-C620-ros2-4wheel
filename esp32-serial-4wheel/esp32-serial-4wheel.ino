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

void setup() {
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
    // put your main code here, to run repeatedly:
    //serial input example: 0.1,0.2,0.3,0.4
  if(Serial.available()){
    String str = Serial.readStringUntil('\n');
    int index = str.indexOf(",");
    target_rad1 = str.substring(0,index).toFloat();
    str = str.substring(index+1);
    index = str.indexOf(",");
    target_rad2 = str.substring(0,index).toFloat();
    str = str.substring(index+1);
    index = str.indexOf(",");
    target_rad3 = -1*str.substring(0,index).toFloat();
    target_rad4 = -1*str.substring(index+1).toFloat();
  }

    // if(Serial.available()>0){
    //     String input = Serial.readString();
    //     float sum = 0;
    //     int comma = 0;
    //     for(int i=0; i<input.length(); i++){
    //         if(input[i]==','){
    //             comma++;
    //         }
    //     }
    //     if(comma==3){
    //         float f1 = input.substring(0, input.indexOf(',')).toFloat();
    //         input = input.substring(input.indexOf(',')+1);
    //         float f2 = input.substring(0, input.indexOf(',')).toFloat();
    //         input = input.substring(input.indexOf(',')+1);
    //         float f3 = input.substring(0, input.indexOf(',')).toFloat();
    //         input = input.substring(input.indexOf(',')+1);
    //         float f4 = input.toFloat();
    //         target_rad1 = f1;
    //         target_rad2 = f2;
    //         target_rad3 = f3;
    //         target_rad4 = f4;
    //     }

}
