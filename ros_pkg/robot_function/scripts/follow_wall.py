#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import * 

import math

active_ = False
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done'
    return res

def clbk_laser(msg):
    global regions_
    # laserPoints_ = []
    regions = {
    'right': [],
    'fright': [],
    'front': [],
    'fleft': [],
    'left': [],
    }

    i =0
    while (i<len(msg.ranges)):
        # laserPoints_.append([msg.ranges[i], msg.ranges[i+1]])
        ang = math.atan2(msg.ranges[i+1], msg.ranges[i])
        if ang<=-1.256:
            regions["right"].append(findRange([msg.ranges[i], msg.ranges[i+1]]))
        elif ang>-1.256 and ang<=-0.4186:
            regions["fright"].append(findRange([msg.ranges[i], msg.ranges[i+1]]))
        elif ang>-0.4186 and ang<=0.4186:
            regions["front"].append(findRange([msg.ranges[i], msg.ranges[i+1]]))
        elif ang>0.4186 and ang<=1.256:
            regions["fleft"].append(findRange([msg.ranges[i], msg.ranges[i+1]]))
        elif ang>=1.256:
            regions["left"].append(findRange([msg.ranges[i], msg.ranges[i+1]]))
        i+=3
    
    regions_ = {
        'right': 5 if len(regions["right"])==0 else min(min(regions["right"]), 5),
        'fright': 5 if len(regions["fright"])==0 else min(min(regions["fright"]), 5),
        'front': 5 if len(regions["front"])==0 else min(min(regions["front"]), 5),
        'fleft': 5 if len(regions["fleft"])==0 else min(min(regions["fleft"]), 5),
        'left': 5 if len(regions["left"])==0 else min(min(regions["left"]), 5),
        }

    take_action()

def findRange(pt):
	return math.sqrt(pt[0]**2 + pt[1]**2)

def findAngle(pt):
	return math.atan2(pt[1],pt[0])


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 1 #1.5
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)    
        
def find_wall():
    msg = Twist()
    msg.linear.x = 2.0 #0.2
    msg.angular.z = -1.0 #-0.3
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 1.0 #-0.3
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 1.0 #0.5
    return msg

def main():
    global pub_, active_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('sim_ros_interface/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('sim_ros_interface/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue

        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()