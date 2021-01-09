#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from tf import transformations

from std_srvs.srv import *
import math

srv_client_find_room_center_ = None
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None

state_desc_= ["find_room_center", "go_to_point_1", "follow_wall_1", "correct_yaw", "explore_and_find_beacon", "go_to_point_2", "follow_wall_2", "final_state"]
state_ = 0
completed_ = False

regions_ = None

yaw_ = 0
current_position_ = Point()
desired_position_ = Point()
beacon_found_ = False
room_center_ = None
distance_precision_ = 0.05 #0.1

def room_center_clbk(msg):
    global room_center_
    room_center_ = msg

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

def findRange(pt):
    return math.sqrt(pt[0]**2 + pt[1]**2)

def findAngle(pt):
    return math.atan2(pt[1],pt[0])

def clbk_beacon(msg):
    global beacon_found_
    beacon_found_ = msg.data

def change_state(state):
    global state_, state_desc_
    global srv_client_find_room_center_, srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    # if state_ == 0:
    #     resp = srv_client_go_to_point_(True)
    #     resp = srv_client_wall_follower_(False)
    # if state_ == 1:
    #     resp = srv_client_go_to_point_(False)
    #     resp = srv_client_wall_follower_(True)
    if state_ == 0:
        resp = srv_client_find_room_center_(True)
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(False)
    if (state_ == 1) or (state_ == 5):
        resp = srv_client_find_room_center_(False)
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if (state_ == 2) or (state_ == 6):
        resp = srv_client_find_room_center_(False)
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
    if state_ == 4:
        resp = srv_client_find_room_center_(False)
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
    
def correct_yaw():
    global yaw_, laserPoints_
    pass

def move_forward(vel):
    global twist_pub
    twist = Twist()
    twist.linear.x = vel
    twist_pub.publish(twist)

def stop_robot():
    global twist_pub
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    twist_pub.publish(twist)

def main():
    global completed_, state_, beacon_found_, current_position_, desired_position_, room_center_, distance_precision_
    global srv_client_find_room_center_, srv_client_go_to_point_, srv_client_wall_follower_
    global twist_pub, desiredPosePub

    rospy.init_node("master_node")

    sub_odom = rospy.Subscriber('sim_ros_interface/odom', Odometry, clbk_odom)
    sub_beacon = rospy.Subscriber('sim_ros_interface/beaconTrig', Int8, clbk_beacon)
    sub_laser = rospy.Subscriber('sim_ros_interface/scan', LaserScan, clbk_laser)
    desiredPoseSub = rospy.Subscriber('sim_ros_interface/desired_pose', Point, room_center_clbk)
    desiredPosePub = rospy.Publisher('sim_ros_interface/desired_pose', Point, queue_size=1)


    twist_pub = rospy.Publisher('sim_ros_interface/cmd_vel', Twist, queue_size=1)

    srv_client_find_room_center_ = rospy.ServiceProxy('find_room_center', SetBool)
    srv_client_go_to_point_ = rospy.ServiceProxy('go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('wall_follower_switch', SetBool)

    change_state(0)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
        if completed_:
            print("COMPLETED")
            break
        else:
            print(state_)
            if state_ == 0:
                if not room_center_ is None:
                    change_state(1)     #go to center
                    print('room center found: ', room_center_)
                # else:
                    # room_center_ = Point()
                    # room_center_.x = 0
                    # room_center_.y = 0
                    # room_center_.z = 0
                    # desired_position_ = room_center_
                    # desiredPosePub(desired_position_)

            if state_ == 1:
                dist = math.sqrt((room_center_.x-current_position_.x)**2 + (room_center_.y-current_position_.y)**2)
                if dist<distance_precision_:
                    change_state(3)     #reached center. Correct yaw
                    print("reached room center")

                elif regions_['front'] > 0.15 and regions_['front'] < 1:
                    change_state(2)
                
            elif state_ == 2:
                desired_yaw = math.atan2(room_center_.y - current_position_.y, room_center_.x - current_position_.x)
                err_yaw = normalize_angle(desired_yaw - yaw_)
                
                if math.fabs(err_yaw) < (math.pi / 6) and \
                regions_['front'] > 1.5:
                    change_state(1)
                    print("a")
                
                if err_yaw > 0 and \
                math.fabs(err_yaw) > (math.pi / 4) and \
                math.fabs(err_yaw) < (math.pi / 2) and \
                regions_['left'] > 1.5:
                    change_state(1)
                    print("b")

                    
                if err_yaw < 0 and \
                math.fabs(err_yaw) > (math.pi / 4) and \
                math.fabs(err_yaw) < (math.pi / 2) and \
                regions_['right'] > 1.5:
                    change_state(1)
                    print("c")


            elif state_ == 3:
                correct_yaw()
                print("aligned robot front to doorway")
                # move_forward(1)
                change_state(4) 

            elif state_ == 4:
                if beacon_found_ == 1:
                    stop_robot()
                    desired_position_ = room_center_
                    desiredPosePub.publish(desired_position_)
                    change_state(5)     #go to room center
                else:
                    print("looking for beacon")
            elif state_ == 5:
                # print(room_center_)
                dist = math.sqrt((room_center_.x-current_position_.x)**2 + (room_center_.y-current_position_.y)**2)
                if dist<distance_precision_:
                    change_state(7)     #reached center. Correct yaw
                    print("reached room center finally")

                elif regions_['front'] > 0.15 and regions_['front'] < 1:
                    change_state(6)
            elif state_ == 6:
                desired_yaw = math.atan2(room_center_.y - current_position_.y, room_center_.x - current_position_.x)
                err_yaw = normalize_angle(desired_yaw - yaw_)
                
                if math.fabs(err_yaw) < (math.pi / 6) and \
                regions_['front'] > 1.5:
                    change_state(5)
                
                if err_yaw > 0 and \
                math.fabs(err_yaw) > (math.pi / 4) and \
                math.fabs(err_yaw) < (math.pi / 2) and \
                regions_['left'] > 1.5:
                    change_state(5)
                    
                if err_yaw < 0 and \
                math.fabs(err_yaw) > (math.pi / 4) and \
                math.fabs(err_yaw) < (math.pi / 2) and \
                regions_['right'] > 1.5:
                    change_state(5)

            elif state_ == 7:
                stop_robot()
                completed_ = True
            else:
                rospy.logerr('Unknown state')
                pass

        rate.sleep()

    
if __name__ == "__main__":
    main()