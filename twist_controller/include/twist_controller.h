#pragma once

#include "pid.h"
#include <ros/ros.h>
#include <iostream>
#include "yaw_controller.h"
#include "lowpass.h"

class Controller
{

private:
  double kp, ki, kd, mn, mx, tau, ts;
  double vehicle_mass_, decel_limit_, accel_limit_, wheel_radius_;
  ros::Time last_time_;

  PID throttle_controller;
  YawController yaw_controller;
  LowPassFilter throttle_lpf;

public:
  double throttle_ = 0;
  double brake_ = 0;
  double steel_ = 0;

  Controller();
  ~Controller();

  void init_controller(double vehicle_mass, double decel_limit, double accel_limit,
                       double wheel_radius, double wheel_base, double steer_ratio, double max_lat_accel,
                       double max_steer_angle);

  //  Control throttle, brake and steel by linear velocity, angular velocity.
  void control(bool dbw_enabled, double linear_vel, double angular_vel, double current_vel,
               double dis_l, double dis_r, double dis_c, ros::Time dis_time);
};