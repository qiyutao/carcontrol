#pragma once

#include <ros/ros.h>

class PID
{
  private:
    double kp_, ki_, kd_, mn_, mx_, min_, max_, int_val_, last_error_;

  public:
    PID();
    ~PID();
    void init_pid(double kp, double ki, double kd, double mn, double mx);

    double reset();

    double step(double error, ros::Duration sample_time);
};