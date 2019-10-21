#!/usr/bin/python3
import rospy
import board
import busio
import adafruit_fxos8700
import adafruit_fxas21002c
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

i2c = busio.I2C(board.SCL, board.SDA)

fxos = adafruit_fxos8700.FXOS8700(i2c)
fxas = adafruit_fxas21002c.FXAS21002C(i2c)

if __name__=="__main__":
	rospy.init_node('imu')
	pub = rospy.Publisher('imu',Imu,queue_size=10)
	r = rospy.Rate(100)
	while not rospy.is_shutdown():
		msg = Imu()
		#msg.orientation = None #TODO: sensor fusion for orientation est
		msg.angular_velocity = Vector3(*fxas.gyroscope)
		msg.linear_acceleration = Vector3(*fxos.accelerometer)
		pub.publish(msg)
		r.sleep()
