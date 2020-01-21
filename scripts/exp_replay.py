import os
import numpy as np
from math import sin, cos, acos, atan2, sqrt, pi
from scipy.interpolate import interp1d

class ReplayPool(object):

	def __init__(self, role, res_dir='res1/'):

		self._frame = role
		self._script_dir = os.path.dirname(__file__)
		self._data_dir = os.path.join(self._script_dir, res_dir + self._frame + '/data/')

		self.cmd_roll, self.cmd_pitch, self.cmd_yaw, self.cmd_thrust, self.t_start, self.t_end = self._read_cmd()
		self.roll, self.pitch, self.yaw = self._read_euler()
		self.z = self._read_z()

	# read command
	def _read_cmd(self):

		t, roll, pitch, thrust, yaw = [], [], [], [], []
		
		with open(self._data_dir + 'cmd_vel.csv') as f:
			data = f.readlines()
			for line in data:
				datastr = line.split(',')
				t.append(float(datastr[0]))
				roll.append(float(datastr[2]))   # vy is roll
				pitch.append(-float(datastr[1])) # vx is -pitch
				yaw.append(float(datastr[4]))    # msg.angular.z is yaw
				thrust.append(float(datastr[3])) # vz is thrust
		t = np.asarray(t)
		roll = np.asarray(roll)
		pitch = np.asarray(pitch)
		yaw = np.asarray(yaw)
		thrust = np.asarray(thrust)

		return  interp1d(t, roll, fill_value='extrapolate'),\
				interp1d(t, pitch, fill_value='extrapolate'),\
				interp1d(t, yaw, fill_value='extrapolate'),\
				interp1d(t, thrust, fill_value='extrapolate'),\
				min(t), max(t)

	# read euler angles
	def _read_euler(self):
		t, roll, pitch, yaw = [], [], [], []
		
		with open(self._data_dir + 'euler.csv') as f:
			data = f.readlines()
			for line in data:
				datastr = line.split(',')
				t.append(float(datastr[0]))
				roll.append(float(datastr[1]))   
				pitch.append(-float(datastr[2])) 
				yaw.append(float(datastr[3])) 
		t = np.asarray(t)   
		roll = np.asarray(roll)
		pitch = np.asarray(pitch)
		yaw = np.asarray(yaw)

		return  interp1d(t, roll, fill_value='extrapolate'),\
				interp1d(t, pitch, fill_value='extrapolate'),\
				interp1d(t, yaw, fill_value='extrapolate')

	# read height
	def _read_z(self):
		t, z = [], []
		
		with open(self._data_dir + 'location.csv') as f:
			data = f.readlines()
			for line in data:
				datastr = line.split(',')
				t.append(float(datastr[0]))
				z.append(float(datastr[3]))
		t = np.asarray(t)   
		z = np.asarray(z)

		return  interp1d(t, z, fill_value='extrapolate')