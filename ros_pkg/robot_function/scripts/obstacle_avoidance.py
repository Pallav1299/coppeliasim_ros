#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def laser_callback(msg):
    #m2wr#
    #"""
    regions = {
          "right":  min(min(msg.ranges[:143]), 10),
    "front_right":  min(min(msg.ranges[144:287]), 10),
          "front": 	min(min(msg.ranges[288:431]), 10),
     "front_left":  min(min(msg.ranges[432:575]), 10),
           "left":  min(min(msg.ranges[576:713]), 10),
    }
    #"""

    #turtlebot3#
    """
    regions = {
          "right":  min(min(msg.ranges[:71]), 10),
    "front_right":  min(min(msg.ranges[72:143]), 10),
          "front":  min(min(msg.ranges[144:215]), 10),
     "front_left":  min(min(msg.ranges[216:287]), 10),
           "left":  min(min(msg.ranges[288:359]), 10),
    }
    """
    generate_op(regions)

def generate_op(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state = ""
    d = 0.5

    if regions['front']>d and regions["front_left"]>d and regions["front_right"]>d :
    	state = "Case 1 - no obstacle"
    	linear_x = 0.6
    	angular_z = 0
    elif regions['front']<d and regions["front_left"]>d and regions["front_right"]>d :
        state = "Case 2 - obstacle at front"
        linear_x = 0
        angular_z = -0.3
    elif regions['front']>d and regions["front_left"]<d and regions["front_right"]>d :
        state = "Case 3 - obstacle at front_left"
        linear_x = 0
        angular_z = 0.3
    elif regions['front']>d and regions["front_left"]>d and regions["front_right"]<d :
        state = "Case 4 - obstacle at front_right"
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < d and regions["front_left"] > d and regions["front_right"] < d:
        state = 'case 5 - front and front_right'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < d and regions["front_left"] < d and regions["front_right"] > d:
        state = 'case 6 - front and front_left'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] < d and regions["front_left"] < d and regions["front_right"] < d:
        state = 'case 7 - front and front_left and front_right'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] > d and regions["front_left"] < d and regions["front_right"] < d:
        state = 'case 8 - front-left and front_right'
        linear_x = 0
        angular_z = -0.3
    else:
        state = 'unknown case'
        rospy.loginfo(regions)
    rospy.loginfo(state)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('read_laser')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('m2wr/laser/scan', LaserScan, laser_callback)
    rospy.spin()