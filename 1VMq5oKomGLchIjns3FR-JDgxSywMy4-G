#ifndef C620_H
#define C620_H

#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>
#include "CalPID.h"
//#include <MsTimer2.h>

#define GEAR_RATIO 19 //モーターの減速比 19*ベベルの減速比2
#define STOP_THRESHOLD 5 //5RPM以下のときは止まっているとみなす 回り始め制御用
#define START_CURRENT 0.5 //回り始めは回転と反対方向に50mA流す

class C620
{
private:
    MCP2515 *mcp2515_;
    CalPID *pid_;
    
    float angle_;
    int rpm_;
    float current_;
    float target_rpm_;
    int id_;
    int start_counter_;
    bool data_receive_error_;
    bool motor_stopping_;
    int connectBytes(uint8_t upper, uint8_t lower);
    void setCurrent(can_frame *frame, int motor_id, float current);
    void divideBytes(uint16_t val, uint8_t *data_array_ptr);
    float angle(can_frame *frame);
    int RPM(can_frame *frame);
    float current(can_frame *frame);
public:
    struct can_frame readMsg_;
    struct can_frame* sendMsg_;
    C620(MCP2515* mcp2515, CalPID* pid, can_frame* sendMsg, int id);
    int readRPM();
    float readRad_s();
    float readAngle();
    float readCurrent();

    //以下、どちらかのPID計算関数をCalPIDのインスタンス作成時に指定した周期で実行する
    void updatePID(int target_rpm);
    void updatePID_rad(float target_rad_s);
    
    void transfer();
    void recoverError();
    void stopMotor();
    void setCANData(can_frame* frame);

    //モーターへの出力はせずにセンサーの読み取りデータのみ更新する場合
    void update();
};

#endif