#!/usr/bin/env python
import RPi.GPIO as io
import rospy
import numpy as np

from geometry_msgs.msg import Twist

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
		self.cmdFreq = rospy.get_param('~writeFrq')
		self.cmdRate = rospy.Rate(self.cmdFreq)

		self.cmdTime = rospy.get_time()

	def cmd_vel_callback(self,data):
		vx = data.linear.x
		vy = data.linear.y
		omega = data.angular.z

		self.cmdTime = rospy.get_time()

		vd = np.array([vx,vy,omega])

		self.wd = 100*np.matmul(self.vel_A,vd)
		print(self.wd)

	def stop_bot(self):
		for i in range(0,4):
			self.pwm_pins[i].ChangeDutyCycle(0)

	def bot_shutdown(self):
		self.stop_bot()
		for i in range(0,4):
			self.pwm_pins[i].stop()
		io.cleanup()

	def run(self):
		while not rospy.is_shutdown():
			if (rospy.get_time() - self.cmdTime) > self.maxDelay:
				self.stop_bot()
			else:
				for i in range(0,4):
                       			if self.wd[i] > 0:
                                		io.output(self.dir_pins[i],io.HIGH)
                        		else:
                                		io.output(self.dir_pins[i],io.LOW)
                        		self.pwm_pins[i].ChangeDutyCycle(abs(self.wd[i]))
			self.cmdRate.sleep()
		self.bot_shutdown()

if __name__=="__main__":
	rospy.init_node('ouijabot')
	io.setmode(io.BCM)
	io.setwarnings(False)
	bot = Ouijabot()
	try:
		bot.run()
	except rospy.ROSException as e:
		rospy.logwarn("Shutting down.")
		raise e
