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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import time
global laser_ranges, robot_location, robot_rotation, laser_intensities, final_goal_location, robot_start_location
laser_ranges = np.zeros((361,))
laser_intensities = np.zeros((361,))
robot_start_location = [-8,-2,0]
robot_location = [-8,-2,0]
robot_rotation = [0,0,1.57]
robot_orientation = quaternion_from_euler(robot_rotation[0],robot_rotation[1],robot_rotation[2])
final_goal_location = [4.5,9,0]
Distance_goal = 10**5
heading_theta = 0

##################### Perception Part #####################

def goal_location_marker(final_goal_location):
    marker_pub = rospy.Publisher('goal_location_marker', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.TEXT_VIEW_FACING
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/odom'


    marker_data.scale.z = 1 # Height

    marker_data.pose.position.x = final_goal_location[0]
    marker_data.pose.position.y = final_goal_location[1]


    marker_data.color.a = 1
    marker_data.color.r = 1
    marker_data.color.g = 1
    marker_data.color.b = 1
    p = final_goal_location
    marker_data.text = 'GOAL'

    # for p in points_list:
    #     marker_data.points.append(Point(p[0],p[1],p[2]))
    marker_pub.publish(marker_data)

def callback_laser(msg):
    global laser_ranges, laser_intensities
    laser_ranges = np.array(msg.ranges)
    laser_intensities = np.array(msg.intensities)

def callback_odom(msg):
    global robot_location, robot_rotation, robot_orientation
    location = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    robot_location = location
    orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation)
    rot = [roll, pitch, yaw]
    robot_rotation = rot
    robot_orientation = orientation
    robot_cube_publisher(location,robot_orientation)

def robot_cube_publisher(trans,rot):
    marker_pub = rospy.Publisher('robot_marker', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.CUBE
    marker_data.pose.position.x = trans[0]
    marker_data.pose.position.y = trans[1]
    qot = rot
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

def polar_to_cartesian(r,theta):
    x = r*math.cos(theta)
    y = r*math.sin(theta)
    return x,y

def points_publisher(points_list):
    global robot_location, robot_rotation, robot_orientation
    marker_pub = rospy.Publisher('points_list', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.POINTS
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/odom'

    marker_data.scale.x = 0.02 # width
    marker_data.scale.y = 0.02 # Height

    marker_data.pose.position.x = robot_location[0]
    marker_data.pose.position.y = robot_location[1]
    
    marker_data.pose.orientation.x = robot_orientation[0]
    marker_data.pose.orientation.y = robot_orientation[1]
    marker_data.pose.orientation.z = robot_orientation[2]
    marker_data.pose.orientation.w = robot_orientation[3]

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
        # points_list.append([x,y,0])
    points_publisher(points_list)
    return points_list

def lines_publisher(points_list):
    global robot_orientation, robot_rotation, robot_location
    marker_pub = rospy.Publisher('lines_list', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.LINE_LIST
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/odom'

    marker_data.scale.x = 0.02 # width


    marker_data.pose.position.x = robot_location[0]
    marker_data.pose.position.y = robot_location[1]

    
    marker_data.pose.orientation.x = robot_orientation[0]
    marker_data.pose.orientation.y = robot_orientation[1]
    marker_data.pose.orientation.z = robot_orientation[2]
    marker_data.pose.orientation.w = robot_orientation[3]

    marker_data.color.a = 1
    marker_data.color.r = 1
    marker_data.color.g = 1
    marker_data.color.b = 0

    for p in points_list:
        marker_data.points.append(Point(p[0],p[1],p[2]))
    marker_pub.publish(marker_data)

def goal_line_publisher(points_list):
    global robot_orientation, robot_rotation, robot_location
    marker_pub = rospy.Publisher('goal_line', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.LINE_LIST
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/odom'

    marker_data.scale.x = 0.3 # width


    # marker_data.pose.position.x = robot_location[0]
    # marker_data.pose.position.y = robot_location[1]

    
    # marker_data.pose.orientation.x = robot_orientation[0]
    # marker_data.pose.orientation.y = robot_orientation[1]
    # marker_data.pose.orientation.z = robot_orientation[2]
    # marker_data.pose.orientation.w = robot_orientation[3]

    marker_data.color.a = 0.5
    marker_data.color.r = 0
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

# refference - https://stackoverflow.com/questions/55230528/find-point-where-altitude-meets-base-python
def orthoProjection(p1, p2, p3):
    ax, ay = p1[0],p1[1]
    bx, by = p2[0],p2[1]
    cx, cy = p3[0],p3[1]
    abx = bx - ax
    aby = by - ay
    acx = cx - ax
    acy = cy - ay
    t = (abx * acx + aby * acy) / (abx * abx + aby * aby)
    px = ax + t * abx
    py = ay + t * aby
    return [px, py, 0]

def ransac_lines(points_list):
    all_line_points = []
    all_line_points_pair = []
    k = 50 # iterations 
    d_thresh = 0.2 # Distance threshold
    min_inliers = 2
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
        score = len(inlier_points)
        
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

def get_sorted_lines(laser_ranges, n_lines):
    line_list = []
    points_list = laser_to_points(laser_ranges)
    all_line_points_pair, score_list = ransac_lines(points_list)
    sorted_lines = [x for _,x in sorted(zip(score_list,all_line_points_pair))] # Sort the formed lines According to the line length scores
    sorted_lines.reverse()
    for points in sorted_lines[0:n_lines]:
        line_list.append(points[0])
        line_list.append(points[1])
    return sorted_lines,line_list

def laser_range_direction(laser_ranges):
    left  = np.mean(laser_ranges[289:289+31]) # Mean range of left 120 values
    slight_left  = np.mean(laser_ranges[217:288-31]) # Mean range of left 120 values
    front = np.mean(laser_ranges[145:216-31]) # Mean range of front 120 values
    slight_right = np.mean(laser_ranges[73:144-31]) # Mean range of right 120 values
    right = np.mean(laser_ranges[0:60-31]) # Mean range of right 120 values



    return left,slight_left,front,right,slight_right

#################### Planning Part #########################

def wall_follow(line_points,laser_ranges):
    left,slight_left, front, right,slight_right = laser_range_direction(laser_ranges)
    line_points = line_points[0]
    p1 = orthoProjection(line_points[0],line_points[1],robot_location)
    # p1 = line_points[0]
    p2 = line_points[1]
    D_wall = 15 #10
    wal_lead = 3  #0.1
    angle_wall = math.atan2(p2[1]-D_wall,p2[0]+wal_lead-p1[0])
    linear, angular = vel_compute_withAngle(robot_rotation, angle_wall)
    move(linear, angular)

def wall_follow2(laser_ranges):
    global Distance_goal, final_goal_location
    left,slight_left, front, right,slight_right = laser_range_direction(laser_ranges)
    d_arr = np.array([left,slight_left, front, right,slight_right])
    d_thresh = 2.5
    k = 4
    slack = 1.2
    print('Wall Distance -', d_thresh-np.min(d_arr))
    # t = abs(d_thresh-np.min(d_arr))*k
    t = math.atan(d_thresh/front) - math.atan(np.min(d_arr)/front)

    if(Distance_goal<1.8):
        go_to_goal(final_goal_location)
    elif(left<d_thresh and front >= d_thresh):
        begin=rospy.Time.now()
        while((rospy.Time.now()-begin) < rospy.Duration(slack)):       
            move(0.5,-0.1)
            print('Wall - Go Straight')

    elif(left > d_thresh and slight_left < d_thresh and right < left):
        t = abs(t)*k
        move(0.5,t)
        print('Wal - Turn Left')
    elif(left<d_thresh and front < d_thresh and right > left):
        t = abs(t)*k
        move(0.5,-1.5*t)
        print('Wall - Turn Right')
    elif(slight_left < 0.8 and left > d_thresh):
        move(0.5,-1.2)
    elif(slight_right < d_thresh or slight_left < d_thresh or front < d_thresh):
        if(slight_left > slight_right):
            move(1,abs(slight_right-slight_left))
        if(slight_left < slight_right):
            move(1,-abs(slight_right-slight_left))

def go_to_goal(final_goal_location):
    global robot_location, robot_rotation
    linear,angular = vel_compute2(robot_location,robot_rotation,final_goal_location)
    move(linear,angular)

def isRobot_goal_line(robot_location):
    global robot_start_location,final_goal_location
    m,b = line_param(robot_location,final_goal_location)
    # y = mx + b
    lhs = robot_start_location[1]
    rhs = m*robot_start_location[0] + b
    e = 1
    if(abs(lhs-rhs)<=e):
        return True
    else:
        return False
    

################## Motion Controll Part ####################

def Heading_angle(goal_loc,Current_loc):
    return math.atan2(goal_loc[1]-Current_loc[1],goal_loc[0]-Current_loc[0])

def Distance_compute(pos1,pos2):
    X = pos1[0]
    Y = pos1[1]
    x = pos2[0]
    y = pos2[1]
    d = math.sqrt(((X-x)**2)+((Y-y)**2))
    return d

def vel_compute(robot_location,robot_rotation,goal_loc):
    trans = robot_location 
    rot = robot_rotation 
    global Distance_goal, heading_theta
    # Propotional Controller
    # Parameters
    d_tolerence = 1
    v_max  = 10
    kl = 1 # Linear V tune
    ka = 4 # Angular V tune

    theta = rot[2]
    v_x = 0
    theta_d = 0
    v_theta = 0
    err_theta = 0
    d = Distance_compute(goal_loc,trans)  # Distance

    theta_d = Heading_angle(goal_loc,trans)
    heading_theta = theta_d
    err_theta = theta_d-theta
    if(d>d_tolerence):
        # Linear Velocity
        v_x = kl*d
        if(v_x>v_max):
            v_x = v_max
        # Angular Velocity
        v_theta = ka*err_theta
    v_x = 2
    # print('v_x - ',v_x,' v_theta - ',v_theta,'Distance -',distance,' error -',err_theta)
    # print('Current Heading-',math.degrees(theta),'Goal Heeading -',math.degrees(theta_d))
    # print('Current - ',trans,'Goal -',goal_loc )
    # print('\n')
    return v_x,v_theta # linear velocity Angular Velocity

def vel_compute2(robot_location,robot_rotation,goal_loc):
    trans = robot_location 
    rot = robot_rotation 
    global Distance_goal, heading_theta
    # Propotional Controller
    # Parameters
    d_tolerence = 0.01
    v_max  = 2
    kl = 1 # Linear V tune
    ka = 4 # Angular V tune

    theta = rot[2]
    v_x = 0
    theta_d = 0
    v_theta = 0
    err_theta = 0
    d = Distance_compute(goal_loc,trans)  # Distance

    theta_d = Heading_angle(goal_loc,trans)
    heading_theta = theta_d
    err_theta = theta_d-theta
    if(d>d_tolerence):
        # Linear Velocity
        v_x = kl*d
        if(v_x>v_max):
            v_x = v_max
        # Angular Velocity
        # Turn and Go
        if(abs(err_theta)>0.01):
            v_theta = ka*err_theta
            v_x = 0

    # print('v_x - ',v_x,' v_theta - ',v_theta,'Distance -',distance,' error -',err_theta)
    # print('Current Heading-',math.degrees(theta),'Goal Heeading -',math.degrees(theta_d))
    # print('Current - ',trans,'Goal -',goal_loc )
    # print('\n')
    return v_x,v_theta # linear velocity Angular Velocity

def vel_compute_withAngle(robot_rotation,heading):
    # Propotional Controller
    # Parameters
    ka = 4 # Angular V tune
    rot = robot_rotation
    theta = rot[2]
    v_x = 1
    theta_d = 0
    v_theta = 0
    err_theta = 0
    theta_d = heading
    err_theta = theta_d-theta
    # Angular Velocity
    v_theta = ka*err_theta

    
    # print('v_x - ',v_x,' v_theta - ',v_theta,'Distance -',distance,' error -',err_theta)
    # print('Current Heading-',math.degrees(theta),'Goal Heeading -',math.degrees(theta_d))
    # print('Current - ',trans,'Goal -',goal_loc )
    # print('\n')
    return v_x,v_theta # linear velocity Angular Velocity

def move(linear,angular):
    vel_1 = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=10) # Publish Command to robot_1
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular 
    vel_1.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('perception')
    rate = rospy.Rate(10.0)
    goal_l, goal_b = line_param(robot_location,final_goal_location)

    sub = rospy.Subscriber('/base_scan', LaserScan, callback_laser) # Receive Laser Msg from /stage
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom) # Receive Laser Msg from /stage


    while not rospy.is_shutdown():

        line_list = []
        sorted_lines = []
        d_thresh = 2.3
        Distance_goal = Distance_compute(final_goal_location,robot_location)
        # print('Goal Distance -',Distance_compute(final_goal_location,robot_location))
        # print('Goal Angle -',Heading_angle(final_goal_location,robot_location))
        left,slight_left, front, right,slight_right = laser_range_direction(laser_ranges)

        n_lines = 10
        if(np.mean(laser_ranges)<3):
            sorted_lines,line_list = get_sorted_lines(laser_ranges, n_lines)
        # print('Lines - ',len(line_list)/2)
        lines_publisher(line_list)
        goal_line_publisher([final_goal_location,robot_start_location])
        goal_location_marker(final_goal_location)
        ################ Bug2 Algorithm Start ########################
        if(Distance_goal > 0.5):
            if(np.mean(laser_ranges)<d_thresh):
                if(np.mean(laser_ranges)<d_thresh):
                    print('Wall Follow')
                    wall_follow2(laser_ranges)
                else:
                    print('Go to Goal')
                    go_to_goal(final_goal_location)
            else:
                if(isRobot_goal_line(robot_location)==False):
                    print('Go to Course Line')
                    go_to_goal(orthoProjection(robot_start_location,final_goal_location,robot_location))
                else:
                    print('Go to Goal')
                    go_to_goal(final_goal_location)
        else:
            print('Goal Reached')




        
        





        
        
