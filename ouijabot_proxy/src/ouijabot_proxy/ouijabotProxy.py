#!/usr/bin/env python

'''
The MIT License (MIT)
Copyright (c) 2019 Kunal Shah
                kshah.kunal@gmail.com
'''
#std lib imports 

import sys
import time
import numpy as np
import numpy.linalg as la

# Standard ROS message
import rospy
import std_msgs.msg 
from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import Imu
import tf.transformations as transf 


# Custom ROS message
from ouijabot.msg import Wheel_Spd # custom ROS msg for wheel spd (4 floats)
from ouijabot.msg import Current # custom ROS msg for motor current (4 floats)
from ouijabot.srv import Enable_IMU # enable uploading IMU measurement
from ouijabot.srv import Enable_Current # enable uploading current measurement

def Rot2d(alpha):
    #2d rotation matrix 
    return np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]])


class OuijabotProxy(object):
    """This class implements a proxy for the comunication beteween the user and the ouijabot 
        purposes/usage: 
            handle position/location information from motion capture
            velocity (twist) set point following with respect to frame transforms


        the final output will always be a twist to the ouijabot onboard process
    """
    def __init__(self, postionTopic, velocityTopic, params=None, mode="vel"):
        #postionTopic optitrack or filtered position data 
        #velocityTopic ouijabot output topic 

        #parms
        if params is None:
            self.velMax_l= rospy.get_param('~velMax_l')
            self.velMax_a= rospy.get_param('~velMax_a')
            self.cmdFrq= rospy.get_param('~cmdFrq')
            self.ID= rospy.get_param('~id')
        else:
            self.velMax_l= params['velMax_l']
            self.velMax_a= params['velMax_a']
            self.cmdFrq= params['cmdFrq']
            self.ID = params['ID']

        self.modes = ['vel', 'pos']
        self.mode = mode 

        #pubs
        self.cmdPub = rospy.Publisher(velocityTopic, Twist, queue_size=15)
        #subs
        rospy.Subscriber(postionTopic, PoseStamped, self.poseCB)

        #intialize
        self.pose = None
        self.poseTarget = None
        self.twistCmd = Twist()
        self.enable = False
        #pose check
        counter = 0 
        while self.pose is None:
            #wait for pose 
            rospy.loginfo("Attempting to Establish Position Lock")
            rospy.sleep(1)
            counter +=1
            if counter > 5:
                raise RuntimeError("Position Lock Failure")

        rospy.loginfo("Ouijabot Position Established")
        #timers
        self.controlTimer = rospy.Timer(rospy.Duration(1./self.cmdFrq), self.controlLoop)
        rospy.loginfo("Ouijabot Initialization Exited Successfully: Clear for Launch")

    # call back from optirack data
    def poseCB(self, msg):
        #get pose form topic 
        self.pose=msg.pose

    def setMode(self, mode):
        if mode not in self.modes:
            self.stop()
            self.enable=False
            raise RuntimeError("mode not found. terminating")
        self.mode= mode

    def getPose(self, full=False):
        #return the pose of the robot, if full = True return the full Pose message struct 
        if full:
            return self.pose
        else:
            Q= [self.pose.orientation.x, 
                self.pose.orientation.y, 
                self.pose.orientation.z, 
                self.pose.orientation.w]

            euler =  transf.euler_from_quaternion(Q)
            return (self.pose.position.x, self.pose.position.y, euler[2])

    def setPoseTarget(self, pose):
        #sets the target pose (world frame) of the robot
        #pose is a (x, y, theta) NOT A ROS MSG POSE
        self.mode="pose"

        #wrap angle
        self.poseTarget = (pose[0], pose[1], self.wrapPi(pose[2])) #pose is x, y, theta 


    def poseControl(self):
        kp_lin = 1
        kp_ang = .08

        poseCurrent= self.getPose()
        #get velocity in world frame
        dx = self.poseTarget[0] - poseCurrent[0]
        dy = self.poseTarget[1] - poseCurrent[1]
        dtheta = self.circularDist(self.poseTarget[2], poseCurrent[2])

        v_world = [dx, dy] #2D planar velocity 
        #convert to body frame
        v_body = self.frameTransformR(v_world, poseCurrent[2])


        vel = [v_body[0]*kp_lin,
               v_body[1]*kp_lin, 
               dtheta*kp_ang] 

        self.setVelocityTarget(vel)


    def wrapPi(self, a):
        # returns the angle wrapped -pi to pi
        if a < -np.pi or a > np.pi:
            a = ((a + np.pi) % (2 * np.pi)) - np.pi
        return a

    def circularDist(self, a1, a2):
        #returns the difference between two angles, accounting for wrapping
        #assumes the angles are in [-pi, pi]
        a1, a2 = map(self.wrapPi, (a1, a2))
        print a1, a2

        diff = ( a1 - a2 + np.pi ) % (2*np.pi) - np.pi;
        return diff + (2*np.pi) if diff < -np.pi else diff 

    def getPoseTargetDist(self):
        if self.poseTarget is None:
            raise RuntimeError("goal position not set")

        poseCurrent= self.getPose()
        dx = self.poseTarget[0] - poseCurrent[0]
        dy = self.poseTarget[1] - poseCurrent[1]

        return (dx**2+ dy**2)**.5


    def frameTransformR(self, v, a, reverse=False):
        #transforms the vector v (in frame A) by the transformaton given by angle a (A to B) into frame B
        #assumes that v is a list
        R=Rot2d(a)
        if reverse:
            return np.dot(R, np.array(v)) 
        else:
            return np.dot(R.T, np.array(v))

    def frameTransformQ(self, v, q, reverse=False):

        #NOT YET TESTED
        #transforms the vector v (in frame A) by the transformaton given by quaternion q (A to B) into frame B
        #assumes that v and q are lists

        v_ = [v[0], v[1], 0.0, 0.0] # "real" component of quaternions in ros is w, which is the 4th element. turn vector into a pure (no real part) quaternion

        # "conjugate" vector v1 by quaternion q1 (i.e. apply the rotation to v1) - https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
        v_ = tf.transformations.quaternion_multiply(
                     tf.transformations.quaternion_multiply(q, v_),
                     tf.transformations.quaternion_inverse(q) # should be same as tf.transformations.quaternion_conjugate(q1), assuming q1 is a unit quaternion
                    )[:3]
        return v_

    def setVelocityTarget(self, vel, frame='R'):
        #frame 'R" sets the target velocity of the robot (v_x, v_y, v_theta) in robot frame
        if frame == 'R':

            self.twistCmd.linear.x= vel[0]
            self.twistCmd.linear.y= vel[1]
            self.twistCmd.angular.z= vel[2]
        #frame 'W" sets the target velocity of the robot (v_x, v_y, v_theta) in world frame 
        elif frame =='W':
            poseCurrent= self.getPose()
            #get velocity in world frame
            dx = vel[0]
            dy = vel[1]
            v_world = [dx, dy] #2D planar velocity 
            #convert to body frame
            v_body = self.frameTransformR(v_world, poseCurrent[2])
            self.twistCmd.linear.x= v_body[0]
            self.twistCmd.linear.y= v_body[1]
            self.twistCmd.angular.z= vel[2]
        else:
            self.stop()
            raise RuntimeError('invalid frame for velocity control')

        #clip to tresholds
        self.clipVelocity()

    def clipVelocity(self):
        velMag_l = la.norm([self.twistCmd.linear.x, self.twistCmd.linear.y])
        velMag_a = abs(self.twistCmd.angular.z)
        if velMag_l > self.velMax_l:
            self.twistCmd.linear.x *= (self.velMax_l/velMag_l)
            self.twistCmd.linear.y *= (self.velMax_l/velMag_l)

        if velMag_a > self.velMax_a:
            self.twistCmd.angular.z *= (self.velMax_a/velMag_a)
            
    def getVelocityTarget(self):
        #returns the current velocity setpoint
        return (self.twistCmd.linear.x,
                self.twistCmd.linear.y,
                self.twistCmd.angular.z)

    def setEnable(self, enb):
        #enb is bool
        self.enable = enb

    def stop(self):
        #stops the robot 
        self.twistCmd = Twist() #resets the robot

    def controlLoop(self, event):
        if self.enable:
            if self.mode=="pose":
                self.poseControl()
        else:
            self.stop()
        self.cmdPub.publish(self.twistCmd)

def testVelocity(bot):
    vels=[(1, 0, 0 ),
          (0, -1, 0 ),
          (-1, 0, 0 ),
          (0, 1, 0 ),
          (0, 0, .05 ),
          (0, 0, -.05 )]
    times= [1, 1, 1, 1, 1, 1 ]

    for vel, t in zip(vels, times):
        startTime = time.time()
        while (time.time()- startTime)< t:
            bot.setVelocityTarget(vel,  frame ='W')
            position = bot.getPose()
            velocity = bot.getVelocityTarget()
            print("\ncurrent pose: {:2.4f}, {:2.4f}; {:2.4f} ".format(position[0],position[1], position[2]))
            print("cmd velocity: {:2.4f}, {:2.4f}; {:2.4f} ".format(velocity[0],velocity[1], velocity[2]))
            time.sleep(.5)     
    return 0

   
if __name__ == '__main__':
    """
    This tests the ouijabot proxy class for high level control
    this will simply move the robot around to test each directon"""
    rospy.init_node('ouijabotProxy', anonymous=True)
    ouijabotTest = OuijabotProxy(rospy.get_param('~poseTopic'), 
                                 rospy.get_param('~velTopic'))
    readInput = ""
    while readInput != "x":
        readInput = raw_input("enter x to start: ")
    ouijabotTest.setEnable(True)
    #run test
    testVelocity(ouijabotTest)
    sys.exit()


