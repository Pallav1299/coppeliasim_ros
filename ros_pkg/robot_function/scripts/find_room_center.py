#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations


from std_srvs.srv import *
import math
import numpy as np
import matplotlib.pyplot as plt

room_center_found_ = True
active_ = False
current_position_ = Point()
yaw_ = 0
room_center_ = Point()

def clbk_odom(msg):
    global current_position_, yaw_
    
    # position
    current_position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    # global regions_, laserPoints_
    global laserPoints_
    laserPoints_ = []
    corners = []
    # regions = {
    # 'right': [],
    # 'fright': [],
    # 'front': [],
    # 'fleft': [],
    # 'left': [],
    # }

    i =0
    while (i<len(msg.ranges)):
        laserPoints_.append([msg.ranges[i], msg.ranges[i+1]])
        # ang = math.atan2(msg.ranges[i+1], msg.ranges[i])
        # if ang<=-1.256:
        #     regions["right"].append(findRange([msg.ranges[i], msg.ranges[i+1]]))
        # elif ang>-1.256 and ang<=-0.4186:
        #     regions["fright"].append(findRange([msg.ranges[i], msg.ranges[i+1]]))
        # elif ang>-0.4186 and ang<=0.4186:
        #     regions["front"].append(findRange([msg.ranges[i], msg.ranges[i+1]]))
        # elif ang>0.4186 and ang<=1.256:
        #     regions["fleft"].append(findRange([msg.ranges[i], msg.ranges[i+1]]))
        # elif ang>=1.256:
        #     regions["left"].append(findRange([msg.ranges[i], msg.ranges[i+1]]))
        i+=3
    
    ang_thresh = (60/180)*math.pi
    sampling_ratio = 10
    indexes = np.linspace(0, len(laserPoints_),int(len(laserPoints_)/sampling_ratio))
    # print(indexes[0])
    for i in range(1, len(indexes)-2):
        if pointDist(laserPoints_[int(indexes[i])])<4:
            prev_idx = math.ceil(indexes[i-1])
            idx = int(indexes[i])
            next_idx = math.floor(indexes[i+1])

            # print(prev_idx, next_idx, len(laserPoints_))
            # dist1 = findDist(laserPoints[i-1], laserPoints_[i])
            ang1 = findSlope(laserPoints_[prev_idx], laserPoints_[idx])

            # dist2 = findDist(laserPoints[i], laserPoints_[i+1])
            ang2 = findSlope(laserPoints_[idx], laserPoints_[next_idx])
            
            ang_diff = ang2-ang1
            
            # print(ang1, ang2, ang_diff, ang_thresh)
            if math.fabs(ang_diff)>=ang_thresh:
                # corners.append(laserPoints_[prev_idx])
                corners.append(laserPoints_[idx])
                # corners.append(laserPoints_[next_idx])
    # print(len(corners))
    corners = np.array(corners)
    laserPoints_ = np.array(laserPoints_)
    plt.plot(0,0,"bo")
    plt.plot(laserPoints_[:,0], laserPoints_[:,1], "yo")
    plt.plot(corners[:,0], corners[:,1], "ro")

    # plt.show()
        
    
    # regions_ = {
    #     'right': 5 if len(regions["right"])==0 else min(min(rFalseegions["right"]), 5),
    #     'fright': 5 if len(regions["fright"])==0 else min(min(regions["fright"]), 5),
    #     'front': 5 if len(regions["front"])==0 else min(min(regions["front"]), 5),
    #     'fleft': 5 if len(regions["fleft"])==0 else min(min(regions["fleft"]), 5),
    #     'left': 5 if len(regions["left"])==0 else min(min(regions["left"]), 5),
    #     }
    # print(regions_)

def findSlope(pt1, pt2):
    return math.atan2(pt2[1]-pt1[1], pt2[0]-pt1[0])

def pointDist(pt):
    return math.sqrt(pt[1]**2 + pt[0]**2)

def find_room_center(req):
	global active_
	active_ = req.data
	res = SetBoolResponse()
	res.success = True
	res.message  = 'Done'
	return res

def main():
    global room_center_, room_center_found_, active_
    rospy.init_node('find_room_center')

    # sub_laser = rospy.Subscriber('sim_ros_interface/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('sim_ros_interface/odom', Odometry, clbk_odom)
    desiredPosePub = rospy.Publisher('sim_ros_interface/desired_pose', Point, queue_size=1)

    srv = rospy.Service('find_room_center', SetBool, find_room_center)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if room_center_found_:
                room_center_.x = 0
                room_center_.y = -0.5
                room_center_.z = 0
                desiredPosePub.publish(room_center_)
        rate.sleep()

            
if __name__ == "__main__":
    main()