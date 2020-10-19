#!/usr/bin/python
import roslib
roslib.load_manifest('lab2')
import rospy
import tf
import math
import random
import geometry_msgs.msg
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np
import time
global laser_ranges, robot_location, robot_rotation, laser_intensities
laser_ranges = np.zeros((361,))
laser_intensities = np.zeros((361,))
robot_location = [0,0,0]
robot_rotation = [0,0,0,0]

# Broadcast to /odom to a world frame
def tf_broadcaster(msg, robot_name):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0,msg.pose.pose.orientation.z),
                     rospy.Time.now(),
                     robot_name,
                     "world")  

def callback_laser(msg):
    global laser_ranges, laser_intensities
    laser_ranges = np.array(msg.ranges)
    laser_intensities = np.array(msg.intensities)

def callback_odom(msg):
    global robot_location, robot_rotation
    location = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    robot_location = location
    orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion (orientation)
    rot = [roll, pitch, yaw]
    robot_rotation = rot
    robot_cube_publisher(location,rot)

def robot_cube_publisher(trans,rot):
    marker_pub = rospy.Publisher('robot_marker', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.CUBE
    marker_data.pose.position.x = trans[0]
    marker_data.pose.position.y = trans[1]
    qot = tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
    marker_data.pose.orientation.x = qot[0]
    marker_data.pose.orientation.y = qot[1]
    marker_data.pose.orientation.z = qot[2]
    marker_data.pose.orientation.w = qot[3]
    marker_data.header.frame_id = '/odom'
    marker_data.scale.x = 0.35
    marker_data.scale.y = 0.35
    marker_data.scale.z = 0.25
    marker_data.color.a = 1
    marker_data.color.r = 0
    marker_data.color.g = 0
    marker_data.color.b = 255
    marker_pub.publish(marker_data)

def obstacle_avoider(laser_ranges):
    left  = np.mean(laser_ranges[121:]) # Mean range of left 120 values
    front = np.mean(laser_ranges[61:120]) # Mean range of front 120 values
    right = np.mean(laser_ranges[0:60]) # Mean range of right 120 values
    vel_0 = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=1) # Publish Command to robot_0
    cmd = geometry_msgs.msg.Twist()
    k_a = 1.5 # Obstacle avoidance turn factor
    tolerence = 2
    v_max = 2

    linear = v_max
    angular = 0
    cmd.angular.z = angular
    cmd.linear.x = linear

    if(front < tolerence or right < tolerence or left < tolerence):
        if(left < right):
            angular = -1*k_a* right
            cmd.angular.z = angular
            vel_0.publish(cmd)

        if(right < left):
            angular = k_a* left
            cmd.angular.z = angular
            vel_0.publish(cmd)

    def uturn():
        begin=rospy.Time.now()
        angular = random.choice([3.14,-3.14]) # 180 degrees in 0.5 seconds
        linear = 0
        cmd.angular.z = angular
        cmd.linear.x = linear
        # print('U Turn')
        cmd.angular.z = angular
        cmd.linear.x = linear
        while((rospy.Time.now()-begin) < rospy.Duration(0.5)):
            vel_0.publish(cmd)

    t = 1
    if(front < t or right < t or left < t):
        uturn()
    if(front < t or right < t):
        uturn()
    if(front < t or left < t):
        uturn()

    linear = v_max
    angular = 0
    cmd.angular.z = angular
    cmd.linear.x = linear
    vel_0.publish(cmd)

def polar_to_cartesian(r,theta):
    x = r*math.cos(theta)
    y = r*math.sin(theta)
    return x,y

def points_publisher(points_list):
    marker_pub = rospy.Publisher('points_list', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.POINTS
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/odom'

    marker_data.scale.x = 0.02 # width
    marker_data.scale.y = 0.02 # Height

    marker_data.pose.position.x = robot_location[0]
    marker_data.pose.position.y = robot_location[1]
    qot = tf.transformations.quaternion_from_euler(robot_rotation[0], robot_rotation[1], robot_rotation[2])
    marker_data.pose.orientation.x = qot[0]
    marker_data.pose.orientation.y = qot[1]
    marker_data.pose.orientation.z = qot[2]
    marker_data.pose.orientation.w = qot[3]

    marker_data.color.a = 1
    marker_data.color.r = 0
    marker_data.color.g = 1
    marker_data.color.b = 0

    for p in points_list:
        marker_data.points.append(Point(p[0],p[1],p[2]))
    marker_pub.publish(marker_data)

def laser_to_points(laser_ranges):
    points_list = []
    theta_list = np.arange(180.5,step=0.5)
    for i in range(len(laser_ranges)):
        ranges = laser_ranges
        theta = math.radians(theta_list[i])-(math.pi/2)
        r = ranges[i]
        x,y = polar_to_cartesian(r,theta)
        # angle = robot_rotation[2]-(math.pi/2)
        # xn = x*math.cos(angle)-y*math.sin(angle)
        # yn = x*math.sin(angle)+y*math.cos(angle)
        # points_list.append((xn+robot_location[0],yn+robot_location[1],0))
        if(r<3):
            points_list.append([x,y,0])
    points_publisher(points_list)
    return points_list

def lines_publisher(points_list):
    marker_pub = rospy.Publisher('lines_list', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.LINE_LIST
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/odom'

    marker_data.scale.x = 0.02 # width


    marker_data.pose.position.x = robot_location[0]
    marker_data.pose.position.y = robot_location[1]

    qot = tf.transformations.quaternion_from_euler(robot_rotation[0], robot_rotation[1], robot_rotation[2])
    marker_data.pose.orientation.x = qot[0]
    marker_data.pose.orientation.y = qot[1]
    marker_data.pose.orientation.z = qot[2]
    marker_data.pose.orientation.w = qot[3]

    marker_data.color.a = 1
    marker_data.color.r = 1
    marker_data.color.g = 1
    marker_data.color.b = 0

    for p in points_list:
        marker_data.points.append(Point(p[0],p[1],p[2]))
    marker_pub.publish(marker_data)

def line_param(p1,p2):
    m = 0
    if((p1[0]-p2[0]) != 0):
        m =  (p1[1]-p2[1])/(p1[0]-p2[0])
    b = p1[1] - m*p1[0]
    return m,b

def distance(line_points,point):
    p1 = np.array(line_points[0])
    p2 = np.array(line_points[1])
    m,b = line_param(p1,p2)
    d = abs(m*point[0]-point[1]+b)/math.sqrt((m**2)+1)
    # d = np.linalg.norm(np.cross(p2-p1, p1-point))/np.linalg.norm(p2-p1)
    return d

def ransac_lines(points_list):
    all_line_points = []
    all_line_points_pair = []
    k = 300 # iterations 
    d_thresh = 0.1 # Distance threshold
    min_inliers = 40
    max_inliers = 0
    score_list = []
    t = 0
    while t < k:
        inlier_points = []
        outlier_points = []
        score = 0
        p1 = random.choice(points_list)
        p2 = random.choice(points_list)
        for point in points_list:
            d = distance((p1,p2),point)
            
            if d<d_thresh:
                inlier_points.append(point)
                score = score + d
            else:
                outlier_points.append(point)
        t = t+1
        score = score/len(inlier_points)
        
        if(len(inlier_points)>min_inliers):
            # all_line_points.append([p1,p2])
            all_line_points.append(p1)
            all_line_points.append(p2)
            all_line_points_pair.append([p1,p2])
            score_list.append(score)
            if(len(inlier_points)>max_inliers):
                max_inliers = len(inlier_points)

    # return max_line, all_line_points
    return all_line_points_pair, score_list


if __name__ == '__main__':
    
    rospy.init_node('perception')
    listener = tf.TransformListener()
    sub = rospy.Subscriber('/base_scan', LaserScan, callback_laser) # Receive Laser Msg from /stage
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom) # Receive Laser Msg from /stage
    robot_name = '/odom' # TF Broadcast robot
    # rospy.Subscriber(robot_name,Odometry,tf_broadcaster,robot_name)
    # rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        global robot_location, robot_rotation
        # obstacle_avoider(laser_ranges)
        if(np.mean(np.array(laser_ranges))<3):
            points_list = laser_to_points(laser_ranges)
            all_line_points_pair, score_list = ransac_lines(points_list)
            sorted_lines = [x for _,x in sorted(zip(score_list,all_line_points_pair))] # Sort the formed lines According to the line length scores
            line_list = []
            n_lines = 100 # Top N lines to display
            # sorted_lines.reverse()
            for points in sorted_lines[0:n_lines]:
                line_list.append(points[0])
                line_list.append(points[1])
            print('Lines - ',len(line_list)/2)

            lines_publisher(line_list)





        
        
