#!/usr/bin/env python3
import RPi.GPIO as io
import rospy
import numpy as np

from geometry_msgs.msg import Twist

import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from std_msgs.msg import Float64MultiArray

sign = lambda x: (1,-1)[x<0]

class Ouijabot():
	def __init__(self):
		self.pwm_nos = [26,27,18,24]
		self.dir_pins = [17,22,23,25]
		self.pwm_pins = []
		for i in range(0,4):
			io.setup(self.pwm_nos[i],io.OUT)
			io.setup(self.dir_pins[i],io.OUT)
			self.pwm_pins.append(io.PWM(self.pwm_nos[i], 1e3))
			self.pwm_pins[i].start(0)
			io.output(self.dir_pins[i],io.HIGH)

		self.wd = np.array([0,0,0,0])

		#subscribers
		self.cmd_callback = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

		#parameters
		self.vel_A =(1./3)*np.array([[-1,-1,1],[-1,1,1],[1,1,1],[1,-1,1]]) #translate desired vels to motor commands
		self.maxDelay = rospy.get_param('~maxDelay')
		self.cmdFreq = rospy.get_param('~cmdFrq')
		self.currFreq = rospy.get_param('~currFrq')
		self.cmdMode = rospy.get_param('~cmdMode','equal')
		self.cmdRate = rospy.Rate(self.cmdFreq)
		self.cmdTime = rospy.get_time()

		self.curr_pub =  rospy.Publisher('current',Float64MultiArray,queue_size=0)
		self.i2c = busio.I2C(board.SCL, board.SDA)

		self.ads = ADS.ADS1015(self.i2c)

		self.chan0 = AnalogIn(self.ads,ADS.P0)
		self.chan1 = AnalogIn(self.ads,ADS.P1)
		self.chan2 = AnalogIn(self.ads,ADS.P2)
		self.chan3 = AnalogIn(self.ads,ADS.P3)

		self.channels = [self.chan0,self.chan1,self.chan2,self.chan3]

		self.cmdTimer = rospy.Timer(rospy.Duration(1/self.cmdFreq),self.run)
		self.currTimer = rospy.Timer(rospy.Duration(1/self.currFreq),self.current_callback)


	def cmd_vel_callback(self,data):
		vx = data.linear.x
		vy = data.linear.y
		omega = data.angular.z

		self.cmdTime = rospy.get_time()
		vd = np.array([vx,vy,omega])

		if self.cmdMode == 'equal':
			self.wd = 100*np.matmul(self.vel_A,vd)
		elif self.cmdMode == 'max':
			if np.sum(np.abs(vd)) > 1:
				vd = vd / np.sum(np.abs(vd))
			self.wd = 3*100*np.matmul(self.vel_A,vd)
		#print(self.wd)

	def current_callback(self,event):
		data = []
		for i in range(0,4):
			data.append(sign(self.wd[i])*self.channels[i].voltage)
		msg = Float64MultiArray(data=data)
		self.curr_pub.publish(msg)

	def stop_bot(self):
		for i in range(0,4):
			self.pwm_pins[i].ChangeDutyCycle(0)

	def bot_shutdown(self):
		self.stop_bot()
		for i in range(0,4):
			self.pwm_pins[i].stop()
		io.cleanup()

	def run(self,event):
		if (rospy.get_time() - self.cmdTime) > self.maxDelay:
			self.stop_bot()
		else:
			for i in range(0,4):
				if self.wd[i] > 0:
					io.output(self.dir_pins[i],io.HIGH)
				else:
					io.output(self.dir_pins[i],io.LOW)
				self.pwm_pins[i].ChangeDutyCycle(abs(self.wd[i]))

if __name__=="__main__":
	rospy.init_node('ouijabot')
	io.setmode(io.BCM)
	io.setwarnings(False)
	try:
		Ouijabot()
		rospy.logwarn("Starting up.")
		rospy.spin()
	except rospy.ROSException as e:
		rospy.logwarn("Shutting down.")
		self.bot_shutdown()
		raise e
