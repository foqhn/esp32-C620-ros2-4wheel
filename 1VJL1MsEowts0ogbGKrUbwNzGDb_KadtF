#include "CalPID.h"

CalPID::CalPID(double kp_, double ki_, double kd_, double dt, double max)
{
    setParameter(kp_, ki_, kd_);
    deviation_old = 0;
    integral = 0;
    delta_t = dt;
    setMaxValue(max);
};
void CalPID::setParameter(double kp_, double ki_, double kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}
void CalPID::setMaxValue(double max)
{
    max_pid = max;
}
void CalPID::setDELTA_T(double delta_time)
{
    delta_t = delta_time;
}
double CalPID::calPID(double devia_present)
{
    double differential = (deviation_old - devia_present) / delta_t;
    if (value_PID != max_pid && value_PID != -max_pid)
    { //アンチワインドアップ（積分項固定式）
        integral += (devia_present + deviation_old) * delta_t / 2;
    }
    value_PID = kp * devia_present + ki * integral - kd * differential;

    if (value_PID > max_pid)
    {
        value_PID = max_pid;
    }
    else if (value_PID < -max_pid)
    {
        value_PID = -max_pid;
    }

    deviation_old = devia_present;

    return value_PID;
}
double CalPID::calPI(double devia_present)
{
    if (value_PID != max_pid && value_PID != -max_pid)
    { //アンチワインドアップ（積分項固定式）
        integral += (devia_present + deviation_old) * delta_t / 2;
    }
    value_PID = kp * devia_present + ki * integral;

    if (value_PID > max_pid)
    {
        value_PID = max_pid;
    }
    else if (value_PID < -max_pid)
    {
        value_PID = -max_pid;
    }

    deviation_old = devia_present;

    return value_PID;
}
double CalPID::calPD(double devia_present)
{
    double differential = (deviation_old - devia_present) / delta_t;
    double value_PD = kp * devia_present - kd * differential;

    if (value_PD > max_pid)
    {
        value_PD = max_pid;
    }
    else if (value_PD < -max_pid)
    {
        value_PD = -max_pid;
    }

    deviation_old = devia_present;

    return value_PD;
}
double CalPID::calP_D(double devia_present, double diff_value)
{
    double value_P_D = kp * devia_present - kd * diff_value;

    if (value_P_D > max_pid)
    {
        value_P_D = max_pid;
    }
    else if (value_P_D < -max_pid)
    {
        value_P_D = -max_pid;
    }

    return value_P_D;
}
double CalPID::calPI_D(double devia_present, double diff_value)
{
    if (value_PID != max_pid && value_PID != -max_pid)
    { //アンチワインドアップ（積分項固定式）
        integral += (devia_present + deviation_old) * delta_t / 2;
    }
    value_PID = kp * devia_present + ki * integral - kd * diff_value;

    if (value_PID > max_pid)
    {
        value_PID = max_pid;
    }
    else if (value_PID < -max_pid)
    {
        value_PID = -max_pid;
    }

    deviation_old = devia_present;

    return value_PID;
}
void CalPID::resetIntegral()
{
    integral = 0;
}