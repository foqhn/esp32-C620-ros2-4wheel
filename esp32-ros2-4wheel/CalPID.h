#ifndef CALPID_H
#define CALPID_H

class CalPID
{
private:
    double kp, ki, kd;
    double delta_t;
    double deviation_old;
    double integral;
    double value_PID;
    double max_pid;

public:
    CalPID(double kp_, double ki_, double kd_,double dt, double max);//PIDの係数３つ（PDの場合でもI=0とかにして３つ）制御周期、、最大値
    void setParameter(double kp_, double ki_, double kd_);
    void setMaxValue(double max);
    void setDELTA_T(double delta_time);
    double calPID(double devia_present);
    double calPI(double devia_present);
    double calPD(double devia_present);
    double calPI_D(double devia_present,double diff_value);//微分先行型PID
    double calP_D(double devia_present,double diff_value);//微分先行型PD
    void resetIntegral();
};

#endif