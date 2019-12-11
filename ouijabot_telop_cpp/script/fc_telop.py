#!/usr/bin/env python
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3, Twist
import rospy

class ForceTelop():
    """node to generate reference trajectories for ac experiments"""
    def __init__(self):
        self.joy_sub = rospy.Subscriber('/joy',Joy,self.joy_callback,
            queue_size=1)
        self.ref_pub = rospy.Publisher('vel_out',Twist,queue_size=1)
        self.control_freq = rospy.get_param('~control_freq')
        self.ref_timer = rospy.Timer(rospy.Duration(1./self.control_freq),self.ref_callback)
        self.force_scale = rospy.get_param('~force_scale')
        self.torque_scale = rospy.get_param('~torque_scale')
        self.msg = Twist()

    def joy_callback(self,data):
        self.msg = Twist()
        if abs(data.axes[0]) > 0.01:
            self.msg.linear.x = -self.force_scale * data.axes[0] #x-axis backwards
        if abs(data.axes[1]) > 0.01:
            self.msg.linear.y = self.force_scale * data.axes[1]
        if abs(data.axes[2]) > 0.01:
            self.msg.angular.z = self.torque_scale * data.axes[2]

    def ref_callback(self,event):
        self.ref_pub.publish(self.msg)

def main():
    rospy.init_node('fc_telop')
    try:
        ForceTelop()
        rospy.spin()
    except rospy.ROSException as e:
        print('closing force telop')
        raise e

if __name__ == "__main__":
    main()
