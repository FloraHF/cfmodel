#!/usr/bin/env python3

import rospy
import numpy as np
from math import cos, pi

from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped, Twist


def get_time():
	t = rospy.Time.now()
	return t.secs + t.nsecs * 1e-9

class CFmodel(object):

	def __init__(self, cf_id='cf0'):

		self.cf = cf_id
		self.state = 'hover'
		self.goal_msg = PoseStamped()
		self.pqrt_msg = Twist()
		self.setGoal(0., 0., .5) 
		self.setPQRT(0., 0., 0., 0.) 
		self.A = {'roll':1., 'pitch':1., 'yaw':10., 'thrust':500.}

		self.goal_pub = rospy.Publisher('/'+self.cf+'/goal', PoseStamped, queue_size=1) 
		self.pqrt_pub = rospy.Publisher('/'+self.cf+'/pqrt', Twist, queue_size=1) 

		self.takeoff = self.service_client(self.cf, '/cftakeoff') 
		self.land = self.service_client(self.cf, '/cfland') 

		self.identservice = {'roll':  self.service_client(self.cf, '/cmdroll'),
							 'pitch': self.service_client(self.cf, '/cmdpitch'),
							 'yaw':   self.service_client(self.cf, '/cmdyaw'),
							 'thrust':self.service_client(self.cf, '/cmdthrust')}

		rospy.Service('cf0/identthrust', Empty, self.thrust)
		rospy.Service('cf0/identroll', Empty, self.roll)
		rospy.Service('cf0/identpitch', Empty, self.pitch)
		rospy.Service('cf0/identyaw', Empty, self.yaw)

		self.t0 = self.get_time()

	def get_time(self):
		t = rospy.Time.now()
		return t.secs + t.nsecs * 1e-9

	def square_signal(self):
		t = self.get_time() - self.t0
		# print(t)
		y = cos(0.25*pi*t)
		if y >= 0:
			return 1.
		else:
			return -1.
		
	# def service_server(self, cf, name, callback):
	# 	srv_name = '/' + cf + name
	# 	rospy.Service(srv_name, Empty, getattr(self, callback))
	# 	rospy.wait_for_service(srv_name)
	# 	rospy.loginfo('found ' + srv_name + 'service')

	def service_client(self, cf, name):
		srv_name = '/' + cf + name
		rospy.wait_for_service(srv_name)
		rospy.loginfo('found ' + srv_name + 'service')
		return rospy.ServiceProxy(srv_name, Empty)

	def setGoal(self, x, y, z):
		self.goal_msg.header.seq = 0
		self.goal_msg.header.frame_id = '/world'
		self.goal_msg.header.stamp = rospy.Time.now()
		self.goal_msg.pose.position.x = x
		self.goal_msg.pose.position.y = y
		self.goal_msg.pose.position.z = z
		self.goal_msg.pose.orientation.x = 0
		self.goal_msg.pose.orientation.y = 0
		self.goal_msg.pose.orientation.z = 0
		self.goal_msg.pose.orientation.w = 1

	def setPQRT(self, p, q, r, T):
		self.pqrt_msg.linear.x = -q
		self.pqrt_msg.linear.y = p
		self.pqrt_msg.linear.z = T
		self.pqrt_msg.angular.z = r

	def roll(self, req):
		# self.setGoal(0., 0., .5) 
		self.state = 'roll'
		self.identservice['roll']()
		return EmptyResponse()

	def pitch(self, req):
		# self.setGoal(0., 0., .5) 
		self.state = 'pitch'
		self.identservice['pitch']()
		return EmptyResponse()

	def yaw(self, req):
		# self.setGoal(0., 0., .5) 
		self.state = 'yaw'
		self.identservice['yaw']()
		return EmptyResponse()

	def thrust(self, req):
		# self.setGoal(0., 0., .5) 
		self.state = 'thrust'
		self.identservice['thrust']()
		return EmptyResponse()

	def iteration(self, event): 
		# print(self.state)
		self.goal_pub.publish(self.goal_msg)
		if self.state == 'yaw':
			self.setPQRT(0., 0., self.A['yaw']*self.square_signal(), 0.)
			self.pqrt_pub.publish(self.pqrt_msg)
		elif self.state == 'pitch':
			self.setPQRT(0., self.A['pitch']*self.square_signal(), 0., 0.)
			self.pqrt_pub.publish(self.pqrt_msg)
		elif self.state == 'roll':
			p = self.A['roll']*self.square_signal()
			self.setPQRT(p, 0., 0., 0.)
			self.pqrt_pub.publish(self.pqrt_msg)
		elif self.state == 'thrust':
			self.setPQRT(0., 0., 0., self.A['thrust']*self.square_signal())
			self.pqrt_pub.publish(self.pqrt_msg)

if __name__ == '__main__':

	rospy.init_node('CFmodel', anonymous=True)
	cf_id = rospy.get_param("~cf_frame", "/cf0")
	cf = CFmodel(cf_id)
	rospy.Timer(rospy.Duration(1.0/20), cf.iteration)
	rospy.spin()
