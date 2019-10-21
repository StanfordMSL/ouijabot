#!/usr/bin/python3
import rospy
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from std_msgs.msg import Float64MultiArray

i2c = busio.I2C(board.SCL, board.SDA)

ads = ADS.ADS1015(i2c)

chan0 = AnalogIn(ads,ADS.P0)
chan1 = AnalogIn(ads,ADS.P1)
chan2 = AnalogIn(ads,ADS.P2)
chan3 = AnalogIn(ads,ADS.P3)

if __name__=="__main__":
	rospy.init_node('current')
	pub = rospy.Publisher('current',Float64MultiArray,queue_size=10)
	r = rospy.Rate(100)
	while not rospy.is_shutdown():
		msg = Float64MultiArray(data=[chan0.voltage,chan1.voltage,chan2.voltage,chan3.voltage])
		pub.publish(msg)
		r.sleep()
