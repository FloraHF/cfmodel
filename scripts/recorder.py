#!/usr/bin/env python3

import os
import numpy as np
from math import atan2, asin
import rospy
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from std_msgs.msg import Float32MultiArray, String, Float32
from geometry_msgs.msg import PoseStamped, Twist
from crazyflie_game.msg import Mocap

class DataRecorder(object):
	def __init__(self, max_size=1e3):
		self.max_size = max_size
		self.cur_size = 0
		self.data = []
		self.time = []

	def record(self, t, data):
		if self.cur_size >= self.max_size:
			self.data.pop(0)
			self.time.pop(0)
			self.cur_size -= 1
		self.data.append(data)
		self.time.append(t)
		self.cur_size += 1


class PlayerRecorder(object):

	def __init__(self, cf_id="/cf2",
				 cmd_vel='cf2/cmd_vel',
				 mocap="/cf2/mocap",
				 max_size=1e4,
				 rate=10,
				 logger_dir='res1'):

		self._cf_id = cf_id
		self._init_time = self._get_time()

		script_dir = os.path.dirname(__file__)
		self._data_dir = os.path.join(script_dir, logger_dir+'/'+self._cf_id+'/data/')
		if not os.path.isdir(self._data_dir):
			os.makedirs(self._data_dir)

		self._location_dirc = os.path.join(self._data_dir, 'location.csv')
		print(self._location_dirc)
		self._euler_dirc = os.path.join(self._data_dir, 'euler.csv')
		self._cmd_vel_dirc = os.path.join(self._data_dir, 'cmd_vel.csv')

		if os.path.exists(self._location_dirc):
			os.remove(self._location_dirc)
		if os.path.exists(self._euler_dirc):
			os.remove(self._euler_dirc)
		if os.path.exists(self._cmd_vel_dirc):
			os.remove(self._cmd_vel_dirc)

		self.rate = rospy.Rate(rate)
		self._mocap_sub = rospy.Subscriber(mocap, Mocap, self._update_mocap)
		self._cmd_vel_sub = rospy.Subscriber(cmd_vel, Twist, self._update_cmd_vel)

	def _get_time(self):
		t = rospy.Time.now()
		return t.secs + t.nsecs * 1e-9

	def _qt_to_euler(self, mocap):
		quat = np.zeros(4)
		quat[0] = mocap.quaternion[0]
		quat[1] = mocap.quaternion[1]
		quat[2] = mocap.quaternion[2]
		quat[3] = mocap.quaternion[3]
		ans = np.zeros(3)
		ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
					   1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]))
		ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]))
		ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]),
					   1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]))
		return ans

	def _update_mocap(self, mocap):
		t = self._get_time() - self._init_time
		
		with open(self._location_dirc, 'a') as f:
			f.write('%.3f, %.3f, %.3f, %.3f\n'%(t, mocap.position[0], mocap.position[1], mocap.position[2]))

		euler = self._qt_to_euler(mocap)
		with open(self._euler_dirc, 'a') as f:
			f.write('%.3f, %.3f, %.3f, %.3f\n'%(t, euler[0], euler[1], euler[2]))

	def _update_cmd_vel(self, cmd):
		x = cmd.linear.x
		y = cmd.linear.y
		z = cmd.linear.z
		yaw = cmd.angular.z
		t = self._get_time() - self._init_time

		with open(self._cmd_vel_dirc, 'a') as f:
			f.write('%.3f, %.3f, %.3f, %.3f, %.3f\n'%(t, x, y, z, yaw))


if __name__ == '__main__':
	rospy.init_node('recorder.py', anonymous=True)
	cf_id = rospy.get_param("~cf_frame", "/cf0")
	logger_dir = rospy.get_param("~logger_dir",'')

	recorder = PlayerRecorder(cf_id=cf_id,
					  cmd_vel='/'+cf_id+'/cmd_vel',
					  mocap='/'+cf_id+'/mocap',
					  logger_dir=logger_dir)
	rospy.spin()