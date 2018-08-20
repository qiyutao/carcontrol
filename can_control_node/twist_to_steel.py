#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
from control_node_msgs.msg import SteeringCmd, BrakeCmd, ThrottleCmd


class TwistStampedToSteel(object):
    def __init__(self):
        rospy.init_node('twist_to_steel')

        self.SteelingPublisher = rospy.Publisher(
            '/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.BrakePublisher = rospy.Publisher(
            '/vehicle/brake_cmd', BrakeCmd, queue_size=1)
        self.ThrottlePublisher = rospy.Publisher(
            '/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)

        rospy.Subscriber(
            '/twist_cmd', TwistStamped, self.steering_cmd_cb, queue_size=1)
        rospy.spin()

    def steering_cmd_cb(self, t):

        smsg = SteeringCmd()
        smsg.header.stamp = t.header.stamp
        smsg.steering_cmd = t.twist.angular.z * 100.0

        # the rqt max linear.x should be 4.0
        bmsg = BrakeCmd()
        tmsg = ThrottleCmd()
        bmsg.header.stamp = t.header.stamp
        tmsg.header.stamp = t.header.stamp
        if t.twist.linear.x < 0.0:
            # speed is smaller than 0, stop throttle and break
            tmsg.throttle_cmd = 0.0
            bmsg.brake_cmd = -t.twist.linear.x
        else:
            bmsg.brake_cmd = 0.0
            # the % of throttle should be 0 ~ 100
            tmsg.throttle_cmd = t.twist.linear.x / 4.0

        self.SteelingPublisher.publish(smsg)
        self.BrakePublisher.publish(bmsg)
        self.ThrottlePublisher.publish(tmsg)


if __name__ == '__main__':
    TwistStampedToSteel()
