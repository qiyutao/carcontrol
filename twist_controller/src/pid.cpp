/*
 * @Author: J.Q.Wang [wjq_z@qq.com]
 * @Date: 2018-07-20 15:36:20 
 * @Last Modified by: mikey.zhaopeng
 * @Last Modified time: 2018-07-20 15:36:52
 */
// MIN_NUM = float('-inf')
// MAX_NUM = float('inf')
#include "pid.h"

PID::PID()
{
}

PID::~PID(){}

void PID::init_pid(double kp, double ki, double kd, double mn, double mx)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    min_ = mn;
    max_ = mx;
    int_val_ = 0.0;
    last_error_ = 0.0;
}

double PID::reset()
{
    int_val_ = 0.0;
}

double PID::step(double error, ros::Duration sample_time)
{
    double _sample_time = sample_time.toSec();
    double integral = int_val_ + error * _sample_time;
    double derivative = (error - last_error_) / _sample_time;

    double val = kp_ * error + ki_ * integral + kd_ * derivative;
    if (val > max_){
        val = max_;
    }
    else if (val < min_){
        val = min_;
    }
    else{
        int_val_ = integral;
    }
    last_error_ = error;

    return val;
}
