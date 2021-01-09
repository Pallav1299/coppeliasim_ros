#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from tf import transformations

from std_srvs.srv import *
import math

active_ = False

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
position_ = Point()
desired_position_ = None #Point()
# desired_position_.x = rospy.get_param('des_pos_x')
# desired_position_.y = rospy.get_param('des_pos_y')
# desired_position_.x = 0
# desired_position_.y = 0
# desired_position_.z = 0
regions_ = None

state_desc_ = ['Go to point', 'wall following', "waiting"]
state_ = 0
# 0 - go to point
# 1 - wall following

# callbacks
def clbk_odom(msg):
    global position_, yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    global regions_, laserPoints_
    laserPoints_ = []
    regions = {
    'right': [],
    'fright': [],
    'front': [],
    'fleft': [],
    'left': [],
    }

    i =0
    while (i<len(msg.ranges)):
        laserPoints_.append([msg.ranges[i], msg.ranges[i+1]])
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
    # print(regions_)

def desired_pose_callback(msg):
    global desired_position_
    desired_position_ = msg
    
def findRange(pt):
    return math.sqrt(pt[0]**2 + pt[1]**2)

def findAngle(pt):
    return math.atan2(pt[1],pt[0])

def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 2:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
        

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def go_to_goal_safely(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message  = 'Done'
    return res

def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_, active_
    global srv_client_go_to_point_, srv_client_wall_follower_
    
    rospy.init_node('bug0')
    
    sub_laser = rospy.Subscriber('sim_ros_interface/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('sim_ros_interface/odom', Odometry, clbk_odom)
    desiredPoseSub = rospy.Subscriber('sim_ros_interface/desired_pose', Point, desired_pose_callback)
    
    srv_client_go_to_point_ = rospy.ServiceProxy('go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('wall_follower_switch', SetBool)
    srv = rospy.Service('go_to_goal_safely', SetBool, go_to_goal_safely)

    # initialize going to the point
    change_state(0)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        print("active_: ", active_)
        if not active_:
            change_state(2)
            continue
        else:
            if regions_ == None:
                continue
            if desired_position_ == None:
                continue
            
            if state_ == 0:
                print(state_)
                if regions_['front'] > 0.15 and regions_['front'] < 1:
                    change_state(1)
            
            elif state_ == 1:
                print(state_)
                desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
                err_yaw = normalize_angle(desired_yaw - yaw_)
                
                if math.fabs(err_yaw) < (math.pi / 6) and \
                regions_['front'] > 1.5:
                    change_state(0)
                
                if err_yaw > 0 and \
                math.fabs(err_yaw) > (math.pi / 4) and \
                math.fabs(err_yaw) < (math.pi / 2) and \
                regions_['left'] > 1.5:
                    change_state(0)
                    
                if err_yaw < 0 and \
                math.fabs(err_yaw) > (math.pi / 4) and \
                math.fabs(err_yaw) < (math.pi / 2) and \
                regions_['right'] > 1.5:
                    change_state(0)
            
        rate.sleep()

if __name__ == "__main__":
    main()