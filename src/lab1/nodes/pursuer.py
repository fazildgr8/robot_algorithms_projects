#!/usr/bin/python
import roslib
roslib.load_manifest('lab1')
import rospy
import tf
import math
import random
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
import numpy as np
import time
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler



left = 0
right =  0
front =  0

prev_location = [0,0] #x,y
pursuer_location = [3,3] #x,y
pursuer_rot = [0,0,0] #roll, pitch, yaw
distance = 1000
prev_heading = 0



def callback(msg):
    global front, right, left
    range_arr = np.array(msg.ranges)
    left  = np.mean(range_arr[241:]) # Mean range of left 120 values
    front = np.mean(range_arr[121:240]) # Mean range of front 120 values
    right = np.mean(range_arr[0:120]) # Mean range of right 120 values
    # print('robot_1 Laser-',(left,front,right))

def callback_odom(msg):
    global pursuer_location, pursuer_rot
    pursuer_location = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation)
    pursuer_rot = [roll, pitch, yaw]

# Persuer Velocity Computation
def vel_compute(trans,rot,goal_loc):
    global distance, theta_d, prev_heading
    # Propotional Controller
    # Parameters
    d_tolerence = 0.01
    v_max  = 10
    kl = 1 # Linear V tune
    ka = 4 # Angular V tune

    X = goal_loc[0]
    Y = goal_loc[1]
    x = trans[0]
    y = trans[1]
    theta = rot[2]
    v_x = 0
    theta_d = 0
    v_theta = 0
    err_theta = 0
    d = math.sqrt(((X-x)**2)+((Y-y)**2)) # Distance
    quad = quadrant_check(trans,goal_loc) # Quadrant Find

    if(quad == 3):
        theta = theta + math.pi
    if(quad == 2):
        theta = theta - math.pi


    if((X-x)==0):
        theta_d = 0
        err_theta = 0
    else:
        theta_d = math.atan((Y-y)/(X-x))
        err_theta = theta_d-theta

    err_theta = theta_d-theta
    distance = d
    if(d>=d_tolerence):
        # Linear Velocity
        v_x = kl*d
        if(v_x>v_max):
            v_x = v_max
        # Angular Velocity
        v_theta = ka*err_theta
    
    print('v_x - ',v_x,' v_theta - ',v_theta,'Distance -',distance,' error -',err_theta)
    print('Current Heading-',math.degrees(theta),'Goal Heeading -',math.degrees(theta_d))
    print('Current - ',trans,'Goal -',goal_loc )
    print('Quad -',quad)
    print('\n')
    prev_heading = theta_d
    return v_x,v_theta # linear velocity Angular Velocity

def Obstacle_avoid(vel_1):
    print('Obstacle Pursuer')
    k_a = 1 # Obstacle avoidance turn factor
    cmd = geometry_msgs.msg.Twist()
    if(left < right):
        angular = -1*k_a* right
        cmd.angular.z = angular
        vel_1.publish(cmd)
    if(right < left):
        angular = k_a* left
        cmd.angular.z = angular
        vel_1.publish(cmd)
        # 360 degree turn
    def turn():
        begin=rospy.Time.now()
        angular = random.choice([3.14,-3.14]) # 180 degrees in 0.5 seconds
        linear = 0
        cmd.angular.z = angular
        cmd.linear.x = linear
        # print('U Turn')
        cmd.angular.z = angular
        cmd.linear.x = linear
        while((rospy.Time.now()-begin) < rospy.Duration(0.5)):
            vel_1.publish(cmd)
    t = 1
    if(front < t or right < t or left < t):
        turn()
    if(front < t or right < t):
        turn()
    if(front < t or left < t):
        turn()

# Check the Quadrant of Goal Pose wrt Robot
def quadrant_check(robot_pose,goal_pose):
    x = robot_pose[0]
    y = robot_pose[1]
    X = goal_pose[0]
    Y = goal_pose[1]
    if(X-x > 0 and Y-y > 0):
        return 1
    if(X-x < 0 and Y-y > 0):
        return 2
    if(X-x < 0 and Y-y < 0):
        return 3
    if(X-x > 0 and Y-y < 0):
        return 4
    if(X-x == 0 and Y-y == 0):
        return 0


def uturn(vel_1):
    cmd = geometry_msgs.msg.Twist()
    begin=rospy.Time.now()
    angular = random.choice([6.3,-6.3]) # 360 degrees in 0.5 seconds
    linear = 0
    cmd.angular.z = angular
    cmd.linear.x = linear
    # print('U Turn')
    cmd.angular.z = angular
    cmd.linear.x = linear
    while((rospy.Time.now()-begin) < rospy.Duration(0.5)):
        vel_1.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('pursuer')
    listener = tf.TransformListener() # Listening to transformation msg from stage
    sub = rospy.Subscriber('/robot_1/base_scan', LaserScan, callback) # Receive Laser Msg from /stage
    sub_odom = rospy.Subscriber('/robot_1/odom', Odometry, callback_odom) 
    vel_1 = rospy.Publisher('/robot_1/cmd_vel', geometry_msgs.msg.Twist,queue_size=10) # Publish Command to robot_1
    rate = rospy.Rate(10.0)
    begin = rospy.get_time()

    while not rospy.is_shutdown():
        try:
            (trans_0,rot_0) = listener.lookupTransform('world', '/robot_0/odom', rospy.Time(0)) # robot_0 translation and rotation
            (trans_1,rot_1) = listener.lookupTransform('world', '/robot_1/odom', rospy.Time(0)) # robot_1 translation and rotation
            # print('robot_0 Trans-',trans_0,' Rot-',rot_0)
            # print('robot_1 Trans-',trans_1,' Rot-',rot_1)
            # print('\n')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Update Previous Location every second
        if((rospy.get_time()-begin) > 0.5):
            prev_location = [trans_0[0],trans_0[1]]
            begin = rospy.get_time()
        
        # Pursuer Obstacle Avoidance System
        obstacle_tolerence = 1
        if(front < obstacle_tolerence or right < obstacle_tolerence or left < obstacle_tolerence):
            Obstacle_avoid(vel_1)

        # Persuer Velocity Computation
        cmd = geometry_msgs.msg.Twist()
        linear,angular = vel_compute(pursuer_location,pursuer_rot,prev_location)
        cmd.linear.x = linear
        cmd.angular.z = angular          
        vel_1.publish(cmd)
        rate.sleep()
        # rospy.spin()


