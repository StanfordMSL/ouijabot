#!/usr/bin/env python
import RPi.GPIO as io
import rospy

class Ouijabot():
	def __init__(self):
		self.pwm_pin = 4
		self.dir_pin = 17
		io.setup(self.pwm_pin,io.OUT)
		io.setup(self.dir_pin,io.OUT)
		self.p = io.PWM(self.pwm_pin, 1e3)
		self.p.start(50)
		io.output(self.dir_pin,io.HIGH)

if __name__=="__main__":
	rospy.init_node('ouijabot')
	io.setmode(io.BCM)
	bot = Ouijabot()
	rate = rospy.Rate(1/5.)
	fast = False
	while not rospy.is_shutdown():
		if fast:
			bot.p.ChangeDutyCycle(20)
			fast = False
		else:
			bot.p.ChangeDutyCycle(80)
			fast = True
		rate.sleep()
	bot.p.stop()
	io.cleanup()

