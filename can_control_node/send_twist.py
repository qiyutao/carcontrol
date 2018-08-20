#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped,Twist

class TwistToTwistStamped(object):
	def __init__(self):
		rospy.init_node('send_twist')
		
		self.TwistStamped = rospy.Publisher(
            '/twist_cmd', TwistStamped, queue_size=1)

		rospy.Subscriber(
            '/cmd_vel', Twist, self.twist_cmd_cb, queue_size=1)
		rospy.spin()

	def twist_cmd_cb(self, twist):
		tmsg = TwistStamped()
		tmsg.header.stamp = rospy.Time.now()
		tmsg.twist = twist
		self.TwistStamped.publish(tmsg)


if __name__ == '__main__':
	TwistToTwistStamped()