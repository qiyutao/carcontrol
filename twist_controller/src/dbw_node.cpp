
#include "dbw_node.h"
#include <iostream>
using namespace std;
DBWNode::DBWNode(ros::NodeHandle &nh_)
{
    nh_.param<double>("vehicle_mass", vehicle_mass, 1736.35);
    nh_.param<double>("decel_limit", decel_limit, -3.);
    nh_.param<double>("accel_limit", accel_limit, 1.);
    nh_.param<double>("wheel_radius", wheel_radius, 0.2413);
    nh_.param<double>("wheel_base", wheel_base, 2.677);
    nh_.param<double>("steer_ratio", steer_ratio, 14.46);
    nh_.param<double>("max_lat_accel", max_lat_accel, 3.);
    nh_.param<double>("max_steer_angle", max_steer_angle, 450.0);

    steer_pub = nh_.advertise<control_node_msgs::SteeringCmd>("/vehicle/steering_cmd", 50);
    throttle_pub = nh_.advertise<control_node_msgs::ThrottleCmd>("/vehicle/throttle_cmd", 50);
    brake_pub = nh_.advertise<control_node_msgs::BrakeCmd>("/vehicle/brake_cmd", 50);

    break_msg_sub = nh_.subscribe("/break_msg", 50, &DBWNode::current_velocity_cb, this);
    brake_sub = nh_.subscribe("/twist_cmd", 50, &DBWNode::twist_cmd_cb, this);
    lane_distance_sub_ = nh_.subscribe("/lane_distance", 50, &DBWNode::lane_dis_cb, this);

    controller.init_controller(vehicle_mass, decel_limit, accel_limit, wheel_radius,
                               wheel_base, steer_ratio, max_lat_accel, max_steer_angle);

    //loop();
}

void DBWNode::loop()
{
    ros::Rate r(50); //50Hz

    while (ros::ok())
    {
        ros::spinOnce();
        control_node_msgs::ThrottleCmd tcmd;
        // let set the target v at 2 m/s for test
        target_linear_velocity = 2.0;
        controller.control(dbw_enabled, target_linear_velocity, target_angular_veloctiy, current_linear_velocity,
                           dis_left, dis_right, dis_center, dis_time);

        double throttle = controller.throttle_;
        double brake = controller.brake_;
        double steering = controller.steel_;

        publish(throttle, brake, steering);
        r.sleep();
    }
}

void DBWNode::current_velocity_cb(const can_node_msg::break_report::ConstPtr &msg)
{
    // the speed in msg is km/h, converter it to m/s
    current_linear_velocity = msg->speed / 3.6;
}

void DBWNode::twist_cmd_cb(const geometry_msgs::TwistStamped::ConstPtr &twist_cmd)
{
    target_linear_velocity = twist_cmd->twist.linear.x;
    target_angular_veloctiy = twist_cmd->twist.angular.z;
}

void DBWNode::lane_dis_cb(const control_node_msgs::lane_distance::ConstPtr &dis_msg)
{
    dis_center = dis_msg->dis_to_center;
    dis_left = dis_msg->dis_to_left;
    dis_right = dis_msg->dis_to_right;
    dis_time = dis_msg->header.stamp;
}

void DBWNode::publish(double throttle, double brake, double steer)
{
    control_node_msgs::ThrottleCmd throttle_cmd;

    throttle_cmd.header.stamp = ros::Time::now();
    throttle_cmd.throttle_cmd = throttle;
    throttle_pub.publish(throttle_cmd);

    control_node_msgs::SteeringCmd steering_cmd;
    steering_cmd.header.stamp = ros::Time::now();
    steering_cmd.steering_cmd = steer;
    steer_pub.publish(steering_cmd);

    control_node_msgs::BrakeCmd brake_cmd;
    brake_cmd.header.stamp = ros::Time::now();
    brake_cmd.brake_cmd = brake;
    brake_pub.publish(brake_cmd);
    std::cout << "throttle brake steer:" << throttle << " " << brake << " " << steer << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dbw_node");
    ros::NodeHandle nh("~");

    DBWNode dbwNode(nh);
    dbwNode.loop();
    return 0;
}
