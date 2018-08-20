/*
 * @Author: J.Q.Wang [wjq_z@qq.com]
 * @Date: 2018-07-20 15:04:37 
 * @Last Modified by: J.Q.Wang
 * @Last Modified time: 2018-07-20 16:18:53
 */

#include "twist_controller.h"
#include <algorithm>
#include <iostream>

using namespace std;

Controller::Controller() {}
Controller::~Controller() {}

void Controller::init_controller(double vehicle_mass, double decel_limit, double accel_limit,
                                 double wheel_radius, double wheel_base, double steer_ratio,
                                 double max_lat_accel, double max_steer_angle)
{
    kp = 0.27;
    ki = 0.3;
    kd = 0.0;
    mn = 0.0;
    mx = 1.0;
    throttle_controller.init_pid(kp, ki, kd, mn, mx);
    yaw_controller.init_parameter(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle);
    tau = 1;
    ts = 1.8;

    throttle_lpf.set(tau, ts);

    this->vehicle_mass_ = vehicle_mass;
    this->decel_limit_ = decel_limit;
    this->accel_limit_ = accel_limit;
    this->wheel_radius_ = wheel_radius;

    last_time_ = ros::Time::now();
}

//  Control throttle, brake and steel by linear velocity, angular velocity.
void Controller::control(bool dbw_enabled, double linear_vel, double angular_vel, double current_vel,
                         double dis_l, double dis_r, double dis_c, ros::Time dis_time)
{

    steel_ = yaw_controller.get_steering(linear_vel, angular_vel, current_vel, dis_l, dis_r, dis_c, dis_time);
    double vel_error = linear_vel - current_vel;

    ros::Time current_time = ros::Time::now();
    ros::Duration sample_time = current_time - last_time_;
    last_time_ = current_time;

    throttle_ = throttle_controller.step(vel_error, sample_time);
    throttle_ = throttle_lpf.filt(throttle_);
    brake_ = 0;

    if (fabs(linear_vel - 0.) < 0.05 && current_vel < 0.1)
    {
        throttle_ = 0;
        brake_ = decel_limit_;
    }
    else if (throttle_ < .1 && vel_error < -2.)
    {
        throttle_ = 0;
        double decel = std::max(vel_error / 3.0, decel_limit_);
        brake_ = fabs(decel);
    }
}
