#!/usr/bin/env python

### The MIT License (MIT)
### Copyright (c) 2019 Kunal

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

#custom ROS clases 

from ouijabot_proxy.ouijabotProxy import OuijabotProxy as Prox


def posTest(bot):
    goals = [(0, 0, 0),
             (3, 1, np.pi),
             (3, -1, 0),
             (-3,-1, np.pi),
             (-3, 2.6, 0)]

    #goals = [(-3, 2, 0*np.pi)]
    
    for goal in goals:
        bot.setPoseTarget(goal)
        goalDist = bot.getPoseTargetDist()
        while (goalDist> .1):
            position = bot.getPose()
            velocity = bot.getVelocityTarget()
            goalDist = bot.getPoseTargetDist()
            print("\ncurrent pose: {:2.4f}, {:2.4f}; {:2.4f} ".format(position[0],position[1], position[2]))
            print("cmd velocity: {:2.4f}, {:2.4f}; {:2.4f} ".format(velocity[0],velocity[1], velocity[2]))
            print("dist from goal {:2.4f}".format(goalDist))
            rospy.sleep(1)


        print("goal reached: ")
        print(goal)
        rospy.sleep(.5)
    return 0




   
if __name__ == '__main__':
    """
    This tests the ouijabot proxy class for high level control
    this will simply move the ouibot around a series of waypoints"""
    rospy.init_node('ouijabot_pos_test', anonymous=True)
    ID = str(rospy.get_param('~id')) #getting ID
    ouijabotTest = Prox("/vrpn_client_node/ouijabot"+ID+"/pose", "cmd_vel")

    readInput = ""
    while readInput != "x":
        readInput = raw_input("enter x to start: ")
    ouijabotTest.setEnable(True)

    posTest(ouijabotTest)


    sys.exit()