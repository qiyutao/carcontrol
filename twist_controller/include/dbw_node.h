#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include "can_node_msg/break_report.h"
#include <std_msgs/String.h>

#include "twist_controller.h"

#include "control_node_msgs/SteeringCmd.h"
#include "control_node_msgs/BrakeCmd.h"
#include "control_node_msgs/ThrottleCmd.h"
#include "control_node_msgs/lane_distance.h"


class DBWNode
{
  private:
  
    Controller controller;

    double current_linear_velocity = 0;
    double target_linear_velocity = 0;
    double target_angular_veloctiy = 0;
    bool dbw_enabled = true;

    double vehicle_mass = 1736.35;
    double decel_limit = -3;
    double accel_limit = 1.;
    double wheel_radius = 0.2413;

    double wheel_base = 2.677;
    double steer_ratio = 14.46;
    double max_lat_accel = 3.;
    double max_steer_angle = 450.0;

    double dis_left, dis_right, dis_center;
    ros::Time dis_time;

    ros::Subscriber brake_sub, break_msg_sub, lane_distance_sub_;
    
    ros::Publisher steer_pub, throttle_pub, brake_pub;

  public:
    DBWNode(ros::NodeHandle &nh_);

    void loop();

    void current_velocity_cb(const can_node_msg::break_report::ConstPtr &msg);

    void twist_cmd_cb(const geometry_msgs::TwistStamped::ConstPtr &twist_cmd);

    void lane_dis_cb(const control_node_msgs::lane_distance::ConstPtr &lane_dis_msg);

    void publish(double throttle, double brake, double steer);
};