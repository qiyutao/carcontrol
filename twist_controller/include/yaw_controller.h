#pragma once

#include "cmath"
#include "cstdlib"
#include "CONS.h"
#include "pid.h"

#define __max(a, b) (((a) > (b)) ? (a) : (b))
#define __min(a, b) (((a) < (b)) ? (a) : (b))

//偏离控制
class YawController
{
private:
  double wheel_base;
  double steer_ratio;
  double min_speed;
  double max_lat_accel;
  double min_angle;
  double max_angle;

  double steer_cmd;
  ros::Time update_time;


  PID steering_controller_0_10, steering_controller_10_20, steering_controller_20_30, steering_controller_30_40;

public:
  YawController();
  ~YawController();
  void init_parameter(double wheel_base, double steer_ratio, double min_speed,
                      double max_lat_accel, double max_steer_angle);

  double get_angle(double wheel_angle);

  double get_steering(double linear_velocity, double angular_velocity, double current_velocity,
                      double dis_l, double dis_r, double dis_c, ros::Time dis_time);
};