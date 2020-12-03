#!/usr/bin/python
import roslib
roslib.load_manifest('lab4')
import rospy
import tf
import math
from math import cos,sin
import random
import geometry_msgs.msg
from geometry_msgs.msg import Point,Pose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, GridCells
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import time
from pprint import pprint
################### Classes #####################################
class Node:
    def __init__(self,position):
        self.location = position
        self.prev = None
        self.g_cost = 0
        self.h_cost = 0
        self.f_cost = 0
        self.Occupied = False
    def update_cost(self,g,h):
        self.g_cost = g
        self.h_cost = h
        self.f_cost = g+h
    def __eq__(self, node):
        return self.location == node.location

################### Global Variables ####################################
global robot_location, robot_rotation, final_goal_location, robot_start_location, global_map, final_path, map_odom_trans

global_map = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,0, 0],
                       [0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,0, 0],
                       [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,1, 0],
                       [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,1, 1],
                       [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,1, 1],
                       [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,1, 1],
                       [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,1, 0],
                       [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1,1, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1,1, 0],
                       [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1,1, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1,1, 1]])

print('Loaded Map of Shpae - ',global_map.shape)
final_path = None
map_odom_trans = [-9,-10,0]
laser_ranges = np.zeros((361,))
laser_intensities = np.zeros((361,))
robot_start_location = [-8,-2,0]
robot_location = [-8,-2,0]

robot_rotation = [0,0,1.57]
robot_orientation = quaternion_from_euler(robot_rotation[0],robot_rotation[1],robot_rotation[2])

Distance_goal = 10**5
heading_theta = 0

rospy.set_param('goalx', 4.5)
rospy.set_param('goaly', 7)
final_goal_location = [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]
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

def OccupancyGrid_publish(global_map):
    data = np.array(100*global_map,dtype=np.int8)
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = '/odom'
    map_msg.info = MapMetaData()
    map_msg.info.resolution = 1
    pose_msg = Pose()
    pose_msg.position.x = -data.shape[1]/2
    pose_msg.position.y = data.shape[0]/2
    pose_msg.orientation.x = 3.14
    map_msg.info.origin = pose_msg
    map_msg.info.height = data.shape[0]
    map_msg.info.width = data.shape[1]
    map_msg.data = data.ravel()
    grid_publisher = rospy.Publisher('map', OccupancyGrid,queue_size=1) # Publish Occupancy Grid to RVIZ
    grid_publisher.publish(map_msg)

def points_publisher(points_list):
    global robot_location, robot_rotation, robot_orientation
    marker_pub = rospy.Publisher('path_points', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.POINTS
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/odom'

    marker_data.scale.x = 0.5 # width
    marker_data.scale.y = 0.5 # Height

    marker_data.color.a = 1
    marker_data.color.r = 0
    marker_data.color.g = 1
    marker_data.color.b = 0

    for p in points_list:
        marker_data.points.append(Point(p[0],p[1],0))
    marker_pub.publish(marker_data)

def points_publisher_start_goal(points_list):
    global robot_location, robot_rotation, robot_orientation
    marker_pub = rospy.Publisher('startgoal_points', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.POINTS
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/odom'

    marker_data.scale.x = 1 # width
    marker_data.scale.y = 1 # Height

    marker_data.color.a = 1
    marker_data.color.r = 1
    marker_data.color.g = 0
    marker_data.color.b = 0

    for p in points_list:
        marker_data.points.append(Point(p[0],p[1],0))
    marker_pub.publish(marker_data)
#################### Planning Part #########################

def A_STAR(global_map, start, end, Type = '8c'):
    """
    Refference - https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
    """
    print('Generating New Path')
    start_node = Node(start)
    end_node = Node(end)
    open_list = [start_node]
    closed_nodes = []

    while open_list:
        current_node = open_list[0]
        index = 0
        for i, x in enumerate(open_list):
            if x.f_cost < current_node.f_cost:
                current_node = x
                index = i
        open_list.pop(index)
        closed_nodes.append(current_node)

        if current_node == end_node:
            path = []
            node = current_node
            while node is not None:
                path.append(node.location)
                node = node.prev
            print('--->',path[::-1])
            return path[::-1]


        neibhours = []
        i_list = [0,0,-1,1,-1,-1,1,1]
        j_list = [-1,1,0,0,-1,1,-1,1]
        if(Type=='4c'):
            i_list = [0,0,1,-1]
            j_list = [1,-1,0,0]


        for k in range(len(i_list)):
            node_pos = [current_node.location[0]+i_list[k],current_node.location[1]+j_list[k]]
            if (node_pos[0] > (len(global_map) - 1) or node_pos[0] < 0 or
               (node_pos[1] > len(global_map[node_pos[0]]) - 1) or node_pos[1] < 0):
               continue

            if global_map[node_pos[0]][node_pos[1]] != 0:
                continue

            neibhour_node = Node((node_pos[0], node_pos[1]))
            neibhour_node.prev = current_node
            neibhours.append(neibhour_node)

        for neibhour in neibhours:
            if neibhour in closed_nodes: continue

            g = current_node.g_cost + 1
            h = Distance_compute(neibhour.location,end_node.location,'eu')
            neibhour.update_cost(g,h)
            for onode in open_list:
                if neibhour == onode and neibhour.g_cost > onode.g_cost:
                    continue
            open_list.append(neibhour)
    

def go_to_goal(final_goal_location):
    global robot_location, robot_rotation
    linear,angular = vel_compute2(robot_location,robot_rotation,final_goal_location)
    move(linear,angular)

def isRobot_in_goal(robot_location):
    global final_goal_location
    if(Distance_compute(robot_location,final_goal_location)<0.1):
        return True
    else:
        return False

def Follow_path(path):
    global global_map,final_goal_location, map_odom_trans
    cpath = []
    for x in path:
        cpath.append(convert_odom_frame(x,map_odom_trans))
    print(cpath)
    for loc in cpath:
        while(Distance_compute(robot_location,loc)>0.1):
            OccupancyGrid_publish(global_map)
            goal_location_marker(final_goal_location)
            points_publisher(cpath)
            go_to_goal(loc)

def convert_map_frame(pos,map_odom_trans = [0,0]):
    x = pos[0]- map_odom_trans[0]
    y = pos[1]- map_odom_trans[1]
    return (int(x),int(y))

def convert_odom_frame(pos,map_odom_trans = [0,0]):
    x = pos[0]+ map_odom_trans[0] 
    y = pos[1]+ map_odom_trans[1] 
    return (x,y)

def convert_path(path,trans,t):
    npath = []
    for x in path:
        mat = [x[0],x[1]]
        mat = rot2d(mat,t)
        npath.append((mat[0]+trans[0],mat[1]+trans[1]))
    return npath

################## Motion Controll Part ####################

def Heading_angle(goal_loc,Current_loc):
    return math.atan2(goal_loc[1]-Current_loc[1],goal_loc[0]-Current_loc[0])

def Distance_compute(pos1,pos2,Type = 'd'):
    x1 = pos1[0]
    y1 = pos1[1]
    x2 = pos2[0]
    y2 = pos2[1]
    d = ((x1-x2)**2) + ((y1-y2)**2)
    if Type == 'd':
        return math.sqrt(d)
    if Type == 'eu':
        return d
    if Type == 'manhattan':
        return abs(x1-x2)+abs(y1-y2)

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

def move(linear,angular):
    vel_1 = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=10) # Publish Command to robot_1
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular 
    vel_1.publish(cmd)

def rot2d(v,t):
    x,y = v[0],v[1]
    xr = x*cos(t)-y*sin(t)
    yr = x*sin(t)+y*cos(t)
    return [xr,yr]

if __name__ == '__main__':
    rospy.init_node('A_STAR')
    rate = rospy.Rate(10.0)
    sub = rospy.Subscriber('/base_scan', LaserScan, callback_laser) # Receive Laser Msg from /stage
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom) # Receive Odom readings
    start = convert_map_frame(robot_start_location,map_odom_trans)
    end = convert_map_frame(final_goal_location,map_odom_trans)
    print(start,end)
    final_path = A_STAR(global_map,start,end,'4c')
    
    while not rospy.is_shutdown():

        if(final_goal_location != [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]):
            final_goal_location = [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]
            start = convert_map_frame(robot_location,map_odom_trans)
            end = convert_map_frame(final_goal_location,map_odom_trans)
            final_path = A_STAR(global_map,start,end,'4c')

        OccupancyGrid_publish(global_map)
        points_publisher_start_goal([robot_start_location,final_goal_location])
        points_publisher(convert_path(final_path,[0,0],-3.14))
        goal_location_marker(final_goal_location)
        ################ Follow the Generated Path ########################
        # if(Distance_compute(robot_location,final_goal_location)>0.1):
        #     Follow_path(final_path)



        
        





        
        
