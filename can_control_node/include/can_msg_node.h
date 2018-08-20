#pragma once

#include "ros/ros.h"
#include "controlcan.h"
#include "can_node_msg/can.h"
#include "can_node_msg/break_report.h"
#include "can_node_msg/steel_report.h"
#include "can_node_msg/throttle_report.h"

#include "control_node_msgs/SteeringCmd.h"
#include "control_node_msgs/BrakeCmd.h"
#include "control_node_msgs/ThrottleCmd.h"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>
#include "pid.h"
#include "coms.h"
#include <iostream>
#include <fstream>
#include <signal.h>

class CanMsgNode
{
  private:
    VCI_CAN_OBJ can_obj_[100];
    VCI_CAN_OBJ send_frame[3];

    VCI_CAN_OBJ steerling_frame, throttle_frame, break_frame;

    double steel_manual_, break_manual_, throttle_manual_;
    
    // 是否有人踩刹车踏板，如果人踩刹车踏板，所有线控释放！
    bool break_applied_ = true;

    //最大转角角速度  degree/s
    unsigned int max_steer_speed;
    double current_steer_angle_;
    int freq_counter;

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;

    int vci_device_type_;
    int vci_device_ind_;
    unsigned int can_port_idx_;
    unsigned int can_port_idx_2_;

    int wait_time; // when receive buffer is empty, wait for 20 ms

    std::string can_msg_pub_topic_;

    ros::Publisher eps_msg_pub_, break_msg_pub_, throttle_msg_pub_;
    ros::Subscriber steer_msg_sub_, brake_msg_sub_, throttle_msg_sub_, closest_waypoint_sub_;

    double steering_cmd_;
    double brake_cmd_;
    double throttle_cmd_;

    void SteerCmdCb(const control_node_msgs::SteeringCmd::ConstPtr &msg);
    void BrakeCmdCb(const control_node_msgs::BrakeCmd::ConstPtr &msg);
    void ThrottleCmdCb(const control_node_msgs::ThrottleCmd::ConstPtr &msg);

    int PrepareCan();

    void SendCanMsg(double steerling_value, double break_value, double throttle_value, bool auto_enable);
    void SendSteerMsg(double steerling_value, unsigned int w_mode);
    void SendThrottleMsg(double throttle_value, unsigned int man_control, unsigned int adas_state);
    void SendBreakMsg(double break_value);
    void closest_waypointCb(const std_msgs::Int32ConstPtr& msg);

    void ProcessBreakMsg(VCI_CAN_OBJ &can_obj);
    void ProcessThrottleMsg(VCI_CAN_OBJ &can_obj);
    void ProcessSteelMsg(VCI_CAN_OBJ &can_obj);

    void ReceiveCanMsg();

    void EndSignalHandler(int sig);

    void process();

    can_node_msg::can can_obj2msg(const VCI_CAN_OBJ &can_obj);

  public:
    CanMsgNode(ros::NodeHandle &nh);
    virtual ~CanMsgNode();

    void Spin(int freq);
};