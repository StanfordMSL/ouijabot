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

		self.throttles = [0.,0.,0.,0.] #list of throttles for motors (in percent, [-100.,100.])
		self.curr_des = np.array([0.,0.,0.,0.]) #list of desired currents for motors
		self.currents = np.array([0.,0.,0.,0.]) #list of measured currents

		self.err_prev = np.array([0.,0.,0.,0.]) #last error -- used for 'd' term
		self.err_int = np.array([0.,0.,0.,0.]) #integral of error -- used for 'i' term

		#parameters
		self.maxDelay = rospy.get_param('~maxDelay')
		self.cmdFreq = rospy.get_param('~cmdFrq')
		self.currFreq = rospy.get_param('~currFrq')
		self.cmdRate = rospy.Rate(self.cmdFreq)
		self.cmdTime = rospy.get_time()

		#physical constants
		self.rW = rospy.get_param('/rW') #wheel radius, m
		self.rB = rospy.get_param('/rB') #body radius, m
		self.kMotor = rospy.get_param('/kMotor')*rospy.get_param('/fudgefactor') #motor constant, N*m/Amp
		self.kF = kF = np.sqrt(2)*self.kMotor/(2*self.rW) #N/Amp
		self.kT = kT = self.kF*self.rB #N*m/Amp

		#controller params
		self.kP = rospy.get_param('/kP') #proportional constant 
		self.kI = rospy.get_param('/kI') #integral constant
		self.kD = rospy.get_param('/kD') #derivative constant
		self.kFF = rospy.get_param('/kFF') #feed-forward constant
		self.iDec = rospy.get_param('/iDec') #factor to wind down integral term
		self.filt = rospy.get_param('/currFilt') #filtering factor for current measurements
		self.db = rospy.get_param('/currDB') #current measurement deadband

		#setup allocation matrix
		self.A = A = np.array([[-kF,-kF,kF,kF],[-kF,kF,kF,-kF],[kT,kT,kT,kT]])
		self.A_wrench = A.T@np.linalg.inv(A @ A.T)

		#setup current i/o
		self.curr_pub =  rospy.Publisher('current',Float64MultiArray,queue_size=0)
		self.cd_pub = rospy.Publisher('curr_des',Float64MultiArray,queue_size=0)
		self.i2c = busio.I2C(board.SCL, board.SDA)
		self.ads = ADS.ADS1015(self.i2c)

		self.chan0 = AnalogIn(self.ads,ADS.P0)
		self.chan1 = AnalogIn(self.ads,ADS.P1)
		self.chan2 = AnalogIn(self.ads,ADS.P2)
		self.chan3 = AnalogIn(self.ads,ADS.P3)

		self.channels = [self.chan0,self.chan1,self.chan2,self.chan3]

		#setup timers for control read/write
		self.cmdTimer = rospy.Timer(rospy.Duration(1/self.cmdFreq),self.run)
		self.currTimer = rospy.Timer(rospy.Duration(1/self.currFreq),self.current_callback)
		self.cdTimer = rospy.Timer(rospy.Duration(1/self.currFreq),self.cd_callback)

		#subscribers
		self.cmd_callback = rospy.Subscriber("cmd_wrench", Twist, self.cmd_wrench_callback)

		#TODO: implement 'active' functionality

	def cmd_wrench_callback(self,data):
		fx = data.linear.x
		fy = data.linear.y
		torque = data.angular.z

		self.cmdTime = rospy.get_time()

		Wd = np.array([fx,fy,torque]) #desired wrench

		self.curr_des = self.A_wrench @ Wd #change wrench to motor currents

	def cd_callback(self,event):
		msg = Float64MultiArray(data=self.curr_des)
		self.cd_pub.publish(msg)

	def current_callback(self,event):
		data = []
		for i in range(0,4):
			data.append(sign(self.throttles[i])*self.channels[i].voltage)
		self.currents = self.currents + self.filt*(np.array(data)-self.currents)
		#self.currents[np.where(np.abs(self.currents)<=self.db)] = 0.0
		msg = Float64MultiArray(data=self.currents)
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
		else: #TODO: ifactive
			#calculate error terms
			e = self.currents - self.curr_des #current error
			de = e - self.err_prev #error difference
			#a thought: if clipping, stop winding up integrator
			self.err_int = err_int = self.iDec*self.err_int + e #running error (integral term)

			input = self.kFF*self.curr_des - self.kP*e - self.kD*de - self.kI*err_int
			self.throttles = np.clip(input,-100.,100.)
			self.throttles[np.where(np.abs(self.throttles)<=1.0)] = 0.

			for i in range(0,4):
				if self.throttles[i] > 0:
					io.output(self.dir_pins[i],io.HIGH)
				else:
					io.output(self.dir_pins[i],io.LOW)
				self.pwm_pins[i].ChangeDutyCycle(abs(self.throttles[i]))

			self.err_prev = e #store previous error term for derivative term

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
