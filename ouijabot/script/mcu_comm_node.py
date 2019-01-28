#!/usr/bin/env python

### The MIT License (MIT)
### Copyright (c) 2015 Zijian Wang

import rospy
import std_msgs.msg 
import serial
from time import sleep
from WrapMsg import *
import math
import signal # for catching keyboard interrupt ctrl+c
import sys
import time
#numpy not installed 

# Standard ROS message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

# Custom ROS message
from ouijabot.msg import Wheel_Spd # custom ROS msg for wheel spd (4 floats)
from ouijabot.msg import Current # custom ROS msg for motor current (4 floats)
from ouijabot.srv import Enable_IMU # enable uploading IMU measurement
from ouijabot.srv import Enable_Current # enable uploading current measurement

class Ouijabot(object):
    """docstring for Ouijabot"""
    def __init__(self):
        #main serial comm
        self.ser = serial.serial('/dev/ttyAMA0',115200)  
        #ros node
        rospy.init_node('ouijabot', anonymous=True)

        #parms
        self.maxDelay= rospy.get_param('~maxDelay')
        self.maxVel= rospy.get_param('~maxVel')
        self.readFrq= rospy.get_param('~readFrq')
        self.writeFrq= rospy.get_param('~writeFrq')

        #pub 
        self.pub_wheel_spd = rospy.Publisher('wheel_spd', Wheel_Spd, queue_size=15)
        self.pub_current = rospy.Publisher('current', Current, queue_size=15)
        self.pub_acc = rospy.Publisher('imu', Imu, queue_size=15)

        #subs
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        #timers
        self.read_timer = rospy.Timer(1./self.readFrq, self.read_serial)


        #srv
        rospy.service('enable_imu', Enable_IMU, self.service_enable_imu)
        rospy.service('enable_current', Enable_Current, self.service_enable_current)


        #clean 
        self.ser.flushInput();



        
    # Exit safely: will run when ctrl+c is pressed
    def exit_handler(self, signal, frame):
        print 'Program ended by user!'
        sys.exit(0)

        signal.signal(signal.SIGINT, exit_handler) # register the exit interrupt function

    def sensor_msg_handler(self, feed_stream):
        '''
        Extract the sensor message sent by the PCB. 
        Input: feed_stream is the new received data from the serial port
        Output: publish corresponding data through ROS
        ''' 
        if not hasattr(Sensor_Msg_Handler, "prev_stream_left_over"):
            Sensor_Msg_Handler.prev_stream_left_over = ""  # add an attribute to the function (equivalent to static var in C)
        # combine the stream left over previously to the new stream 
        stream = Sensor_Msg_Handler.prev_stream_left_over + feed_stream

        # check the existence of packet head @
        if '@' not in stream:
            Sensor_Msg_Handler.prev_stream_left_over = stream
            #print "no @ in stream"
            return
        while '@' in stream:
            # locate the first head, discard all the information before the head
            head_idx = stream.index('@')
            if len(stream[head_idx: ]) < 2: # @ is the last byte
                Sensor_Msg_Handler.prev_stream_left_over = stream[head_idx: ]
                print "stream trancated: @ is the last byte"
                return  
            data_len = 24; # initialize this var
            if stream[head_idx+1] == 'A': # Wheel_Spd, format: "@+type+s1+s2+s3+s4+checksum"
                data_len = 16 # number of bytes of data, excluding @, type, checksum
            elif stream[head_idx+1] == 'B': # Motor current
                data_len = 16
            elif stream[head_idx+1] == 'C': # Acceleration
                data_len = 12 # only 3 bytes for 3-axis Acc
            elif stream[head_idx+1] == 'E': # 3-axis Acc + z-axis Gyro
                data_len = 16 
            #
            # Reserve for more types
            #
            else: # unknown type, discard the whole package
                Sensor_Msg_Handler.prev_stream_left_over = stream[head_idx+1: ] # remove the '@' of packet is equivalent to discard this entire wrong packet
                print "unknown sensor packet type"
                return
            if len(stream[head_idx+2:-1]) < data_len: # haven't yet received the full packet
                Sensor_Msg_Handler.prev_stream_left_over = stream[head_idx: ]
                #print "stream trancated: haven't yet received the full packet", stream
                #print "left:", Sensor_Msg_Handler.prev_stream_left_over
                return

            ### Retrieve data
            #checksum
            checksum = 0
            for bb in stream[head_idx+1:head_idx+2+data_len]:
                checksum += ord(bb)
            checksum = checksum % 256
            if checksum != ord(stream[head_idx+2+data_len]):
                print "[Warning] checksum unmatch in sensor message"
                rospy.logwarn("[Warning] checksum unmatch in sensor message")
                Sensor_Msg_Handler.prev_stream_left_over = stream[head_idx+1: ] # remove the '@' of packet is equivalent to discard this entire wrong packet
                return
            # checksum matched, extract every kind of sensor message
            if stream[head_idx+1] == 'A': # Wheel_Spd, format: "@+type+s1+s2+s3+s4+checksum"
                # print stream[head_idx:head_idx+2], len(stream), len(stream[head_idx:]), stream[head_idx:]
                wheel_spd = [0.0,0.0,0.0,0.0]
                data_start_idx = head_idx+2
                temp = struct.unpack('f', stream[data_start_idx: data_start_idx+4])
                wheel_spd[0] = temp[0]
                temp =  struct.unpack('f', stream[data_start_idx+4: data_start_idx+8])
                wheel_spd[1] = temp[0]
                temp = struct.unpack('f', stream[data_start_idx+8: data_start_idx+12])
                wheel_spd[2] = temp[0]
                temp = struct.unpack('f', stream[data_start_idx+12: data_start_idx+16])
                wheel_spd[3] = temp[0]
                # publish to ROS using custom msg
                a = Wheel_Spd()
                a.wheel_spd = wheel_spd
                self.pub_wheel_spd.publish(a)
                # print "[WheelSpd] %.3f, %.3f, %.3f, %.3f" % (wheel_spd[0], wheel_spd[1], wheel_spd[2], wheel_spd[3])
                stream = stream[head_idx+3+data_len: ] # delete the extracted data from the stream
            elif stream[head_idx+1] == 'B': # Motor current
                current = [0.0,0.0,0.0,0.0]
                data_start_idx = head_idx+2
                temp = struct.unpack('f', stream[data_start_idx: data_start_idx+4])
                current[0] = temp[0]
                temp =  struct.unpack('f', stream[data_start_idx+4: data_start_idx+8])
                current[1] = temp[0]
                temp = struct.unpack('f', stream[data_start_idx+8: data_start_idx+12])
                current[2] = temp[0]
                temp = struct.unpack('f', stream[data_start_idx+12: data_start_idx+16])
                current[3] = temp[0]
                # publish to ROS using custom msg
                a = Current()
                a.current = current
                self.pub_current.publish(a)
                # print "[Current] %.3f, %.3f, %.3f, %.3f" % (current[0], current[1], current[2], current[3])
                stream = stream[head_idx+3+data_len: ] # delete the extracted data from the stream
            elif stream[head_idx+1] == 'C': # Acceleration
                acc = [0.0,0.0,0.0]
                data_start_idx = head_idx+2
                temp = struct.unpack('f', stream[data_start_idx: data_start_idx+4])
                acc[0] = temp[0]
                temp =  struct.unpack('f', stream[data_start_idx+4: data_start_idx+8])
                acc[1] = temp[0]
                temp = struct.unpack('f', stream[data_start_idx+8: data_start_idx+12])
                acc[2] = temp[0]
                # publish to ROS using custom msg
                a = Imu()
                a.linear_acceleration.x = acc[0]
                a.linear_acceleration.y = acc[1]
                a.linear_acceleration.z = acc[2]
                self.pub_acc.publish(a)
                #print "[Acc] %.3f, %.3f, %.3f" % (acc[0], acc[1], acc[2])
                stream = stream[head_idx+3+data_len: ] # delete the extracted data from the stream
            elif stream[head_idx+1] == 'E': # 3-axis Acc + z-axis Gyro
                acc = [0.0,0.0,0.0]
                wz = 0.0
                data_start_idx = head_idx+2
                temp = struct.unpack('f', stream[data_start_idx: data_start_idx+4])
                acc[0] = temp[0]
                temp =  struct.unpack('f', stream[data_start_idx+4: data_start_idx+8])
                acc[1] = temp[0]
                temp = struct.unpack('f', stream[data_start_idx+8: data_start_idx+12])
                acc[2] = temp[0]
                temp = struct.unpack('f', stream[data_start_idx+12: data_start_idx+16])
                wz = temp[0]
                # publish to ROS using custom msg
                a = Imu()
                a.linear_acceleration.x = acc[0]
                a.linear_acceleration.y = acc[1]
                a.linear_acceleration.z = acc[2]
                a.angular_velocity.z = wz
                self.pub_acc.publish(a)
                stream = stream[head_idx+3+data_len: ] # delete the extracted data from the stream
            else:
                continue
            #
            # Reserve for more types
            #

        Sensor_Msg_Handler.prev_stream_left_over = stream  # update the stream_left_over after the while loop
                        
    # enable_imu service
    def service_enable_imu(self, req):
        if True == req.is_enable:
            self.ser.write(Wrap_Msg_P("enable_imu", 1.0))
        else:
            self.ser.write(Wrap_Msg_P("enable_imu", -1.0))

        # TODO: check whether the parameter has been successfully modified on MCU
        return True

    # enable_current service
    def service_enable_current(self, req):
        if True == req.is_enable:
            self.ser.write(Wrap_Msg_P("enable_current", 1.0))
        else:
            self.ser.write(Wrap_Msg_P("enable_current", -1.0))

        # TODO: check whether the parameter has been successfully modified on MCU
        return True


    def get_vec_ang(self, vec):
        '''
        Get the angle of a vector in [0,360).
        Input: a 2D vector
        Output: [0, 360)
        '''
        if (0 == vec[0]): # vx = 0
            if (vec[1] > 0):
                return 90
            elif (vec[1] < 0):
                return 270
            else:
                return 0
        else: # vx != 0
            if(0 == vec[1]): # vy = 0
                if(vec[0] > 0):
                    return 0
                else:
                    return 180
            else: # vx != 0, vy != 0
                temp = math.fabs(vec[1]/vec[0])
                if ( vec[0]>0 and vec[1]>0 ): # 1st quadrant 
                    return math.atan(temp) * 180 / math.pi
                elif ( vec[0]<0 and vec[1]>0 ): # 2
                    return (math.pi-math.atan(temp)) * 180 / math.pi
                elif ( vec[0]<0 and vec[1]<0 ): # 3
                    return (math.pi+math.atan(temp)) * 180 / math.pi
                else: # 4
                    return (2*math.pi-math.atan(temp)) * 180 / math.pi

    def cmd_vel_callback(self, data):
        vx = data.linear.x
        vy = data.linear.y

        #repackage output information
        self.vel_mag = min(self.maxVel, math.sqrt(vx*vx + vy*vy))
        self.vel_dir = get_vec_ang((vx, vy))
        self.omega = data.angular.z
        self.cmdTime = time.time()
        # print vel_mag, vel_dir, omega
        #self.ser.write(Wrap_Msg_A(vel_mag, vel_dir, omega))

    def read_serial(self):
        #checking for inbound mesasgs and handles them
        if self.ser.inWaiting() > 0:
            self.sensor_msg_handler(self.ser.read(n));

    def stop_bot(self):
        #stops the robot 
        self.ser.write(Wrap_Msg_A(0.0, 0.0, 0.0))

    def run(self)
    cmdRate = rospy.rate(self.writeFrq)
    while not rospy.is_shutdown():
        if (time.time()- self.cmdTime)>self.maxDelay:
            #if exceeds the delay stop robot 
            self.stop_bot()
        else:
        #serial out velocity cmd
            self.ser.write(Wrap_Msg_A(self.vel_mag, self.vel_dir, self.omega))
        cmdRate.sleep()





if __name__ == '__main__':
        bot = Ouijabot()
        try:
            bot.run()
        except rospy.ROSInterruptException as e:
            print(e)
            bot.stop_bot()





    
