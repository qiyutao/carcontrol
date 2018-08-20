/*
 * @Author: J.Q.Wang [wjq_z@qq.com]
 * @Date: 2018-07-20 15:04:07 
 * @Last Modified by: J.Q.Wang
 * @Last Modified time: 2018-07-20 16:19:00
 */

#include "lowpass.h"

LowPassFilter::LowPassFilter() {}

void LowPassFilter::set(double tau, double ts)
{
    a = 1. / (tau / ts + 1.);
    b = tau / ts / (tau / ts + 1.);
    last_val = 0;
    ready = false;
}

double LowPassFilter::get()
{
    return last_val;
}

double LowPassFilter::filt(double val)
{
    if (ready)
    {
        val = a * val + b * last_val;
    }
    else
    {
        ready = true;
    }
    last_val = val;
    return val;
}
