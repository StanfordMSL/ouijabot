#!/usr/bin/env python

### The MIT License (MIT)
### Copyright (c) 2015 Zijian Wang

import struct
import serial # for testing

class Param:
	enable_imu, enable_current = range(2) # start from 0

def Wrap_Msg_A(vel_mag, vel_dir, omega):
	'''
	Wrap up message for command type A
	Input: the linear and angular velocity of the robot
	Return value: a string that can be directly written to serial port using ser.write()  
	'''
	# convert float to hex
	x = struct.pack('f', vel_mag)
	y = struct.pack('f', vel_dir)
	z = struct.pack('f', omega)	
	
	# calculate checksum, excluding start byte ('@')
	checksum = 0
	checksum += ord('a')
	for cc in x:
		checksum += ord(cc)
	for cc in y:
		checksum += ord(cc)
	for cc in z:
		checksum += ord(cc)
	checksum = checksum % 256
	
	# concatenate message
	return '@a'+x+y+z+chr(checksum)

def Wrap_Msg_P(param_name, param_val):
	'''
	Wrap up message for setting the value of a parameter in MCU
	message format: "@p + param_name(1-byte-uchar) + param_val(4-byte-float) + checksum(1-byte) "
	Input: param_name is a string
		   param_val is a float number. For boolean: > 0 means true, < 0 means false, typical value: +1 or -1
	'''
	msg_param_name = 0
	msg_param_val = struct.pack('f', param_val)
	if param_name == "enable_imu":
		msg_param_name = Param.enable_imu
	elif param_name == "enable_current":
		msg_param_name = Param.enable_current
	else:
		print "[Error] can't recognize param name in Wrap_Msg_P"
		return None

	checksum = 0
	checksum += ord('p')
	checksum += msg_param_name
	for cc in msg_param_val:
		checksum += ord(cc)
	checksum = checksum % 256

	return '@p' + chr(msg_param_name) + msg_param_val + chr(checksum)

def Wrap_Msg_Z(msg): 
	'''
	Wrap up message for command type Z (plain ASCII text)
	Input: string with a length less than or equal to 17
	Return value: a string that can be directly written to serial port using ser.write()  
	'''	
	if len(msg) > 17:
		msg = msg[0:17]
	if len(msg) < 17:
		msg = msg + '?????????????????'
		msg = msg[0:17]
	checksum = ord('z')
	for cc in msg:
		checksum += ord(cc)
		checksum = checksum % 256

	return '@z'+msg+chr(checksum)
	

################ Test code
#ser = serial.Serial('/dev/ttyAMA0',115200)	
#ser.write(Wrap_Msg_A(100.98, 2, -1.598))
#sleep(0.01)
#ser.write(Wrap_Msg_Z('helloworldhelloworldhelloworld'))

#ser.write(Wrap_Msg_P("enable_imu", -1.0))
    
