#!/usr/bin/python3

"""ROS node which reads and publishes current sensor values.

Module uses adafruit packages to read current sensors from motor control boards
over I2C. Uses a try-except block to catch bad I2C reads. Publishes current
values to the 'current' topic under the node's namespace.
"""

import rospy
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from std_msgs.msg import Float64MultiArray

i2c = busio.I2C(board.SCL, board.SDA) #create i2c object
ads = ADS.ADS1015(i2c) #create ads object

#setup channels for each current sensor
chan0 = AnalogIn(ads,ADS.P0)
chan1 = AnalogIn(ads,ADS.P1)
chan2 = AnalogIn(ads,ADS.P2)
chan3 = AnalogIn(ads,ADS.P3)

if __name__=="__main__":
	rospy.init_node('current')
	pub = rospy.Publisher('current',Float64MultiArray,queue_size=10)
	r = rospy.Rate(100) #TODO(Preston): make this a ROS parameter
	while not rospy.is_shutdown():
        try: #try to read current sensor
            msg = Float64MultiArray(data=[chan0.voltage,chan1.voltage,chan2.voltage,chan3.voltage])
            pub.publish(msg)
        except: #this read can fail; 
            rospy.logwarn('current read error')
		r.sleep()
