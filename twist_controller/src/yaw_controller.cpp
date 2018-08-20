/*
 * @Author: J.Q.Wang [wjq_z@qq.com]
 * @Date: 2018-07-20 15:04:50 
 * @Last Modified by: J.Q.Wang
 * @Last Modified time: 2018-07-20 17:39:26
 */

#include "yaw_controller.h"
#include <iostream>
using namespace std;
YawController::YawController() {}
YawController::~YawController() {}
void YawController::init_parameter(double wheel_base, double steer_ratio, double min_speed, double max_lat_accel, double max_steer_angle)
{
    this->wheel_base = wheel_base;
    this->steer_ratio = steer_ratio;
    this->min_speed = min_speed;
    this->max_lat_accel = max_lat_accel;

    this->min_angle = -max_steer_angle;
    this->max_angle = max_steer_angle;

    update_time = ros::Time::now();

    steering_controller_0_10.init_pid(STEER_0_10_KP, STEER_0_10_KI, STEER_0_10_KD, STEER_MIN_10, STEER_MAX_10);
    steering_controller_10_20.init_pid(STEER_10_20_KP, STEER_10_20_KI, STEER_10_20_KD, STEER_MIN_20, STEER_MAX_20);
    steering_controller_20_30.init_pid(STEER_20_30_KP, STEER_20_30_KI, STEER_20_30_KD, STEER_MIN_30, STEER_MAX_30);
    steering_controller_30_40.init_pid(STEER_30_40_KP, STEER_30_40_KI, STEER_30_40_KD, STEER_MIN_40, STEER_MAX_40);
}

double YawController::get_angle(double wheel_angle)
{
    double angle = wheel_angle * steer_ratio;
    return angle;
}

double YawController::get_steering(double linear_velocity, double angular_velocity, double current_velocity,
                                   double dis_l, double dis_r, double dis_c, ros::Time dis_time)
{
    double current_v_km = current_velocity * 3.6;
    steer_cmd = 0;

    // let try to use the dis to center as cte
    double cte_to_lane = dis_c;

    ros::Duration sample_time = dis_time - update_time;

    if (current_v_km < 10)
    {
        // for the steer, left is +, for the distance: in the left of center is  + , the output is wheel angle
        steer_cmd = -steering_controller_0_10.step(cte_to_lane, sample_time);
    }
    else if (current_v_km < 20)
    {
        steer_cmd = -steering_controller_10_20.step(cte_to_lane, sample_time);
    }
    else if (current_v_km < 30)
    {
        steer_cmd = -steering_controller_20_30.step(cte_to_lane, sample_time);
    }
    else if (current_v_km < 40)
    {
        steer_cmd = -steering_controller_30_40.step(cte_to_lane, sample_time);
    }
    else
    {
        std::cerr << "The speed is too high!!!! It's dangerous to test!!!" << std::endl;
        steer_cmd = -steering_controller_30_40.step(cte_to_lane, sample_time);
    }

    if (fabs(current_v_km) > 0.0)
        return get_angle(steer_cmd);
    else
        return 0.0;
}
