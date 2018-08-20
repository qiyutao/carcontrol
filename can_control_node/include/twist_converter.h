//
// Created by abang on 18-6-15.
//

#ifndef PROJECT_TWIST_CONVERTER_H
#define PROJECT_TWIST_CONVERTER_H

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include "can_node_msg/break_report.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <coms.h>
#include <dbw_mkz_msgs/SteeringReport.h>
#include <dbw_mkz_msgs/FuelLevelReport.h>

/*To convert ros twist message's anglular velocity (rad/s) to desired steering angle in degree*/
/*To convert ros twist message's linear velocity (m/s) to desird car velocity in m/s*/

class twist_converter {

public:
    twist_converter(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate);
    virtual ~twist_converter();

    void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist_msg);
    void break_report_cb(const can_node_msg::break_report::ConstPtr & msg);

    double convert_trans_rot_vel_to_wheel_angle(double linear_v, double angular_v, double wheelbase);

    virtual void spin();
    virtual void spinonce();
    void process();

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhPrivate_;

    //parameters
    std::string twist_topic_;
    std::string desired_steerangle_topic_;
    std::string desired_throttle_topic_;
    std::string desired_break_topic_;

    //published topics
    std_msgs::Float32 desired_steerangle_;  // degree
    std_msgs::Float32 desired_velocity_;    // m/s

    //subscriber
    ros::Subscriber sub_twist_, sub_break_report_;

    //publisher
    ros::Publisher desired_steerangle_publisher, desired_throttle_publisher, desired_break_publisher;

    //car paramters
    double wheelbase;
    double track;
    double steerratio;
    double maxsteerangle;
    double minsteerangle;
    double maxvelocity;
    double minvelocity;

private:

    bool sys_enable_;  // is in autonomous mode?
    geometry_msgs::TwistStamped twist_;
    can_node_msg::break_report current_break_report_;

    static const double GAS_DENSITY = 2.858; // for kg/gal transform
    static double mphToMps(double mph) { return mph * 0.44704;} //mph to m/s
    static double mpsToKmph(double mps) { return mps * 3.6;} //m/s to km ph

};


#endif //PROJECT_TWIST_CONVERTER_H
