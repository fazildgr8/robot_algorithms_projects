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
laser_ranges = np.zeros((361,))
laser_intensities = np.zeros((361,))
robot_start_location = [-8,-2,0]
robot_location = [-8,-2,0]

robot_rotation = [0,0,1.57]
robot_orientation = quaternion_from_euler(robot_rotation[0],robot_rotation[1],robot_rotation[2])

Distance_goal = 10**5
heading_theta = 0

rospy.set_param('goalx', 4.5)
rospy.set_param('goaly', 9)
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
    pose_msg.position.x = -9-0.5
    pose_msg.position.y = -10-0.5
    pose_msg.orientation.x = 0
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

#################### Planning Part #########################

def A_STAR(global_map, start, end, Type = '8c',e = 1, heuristic = 'eu' ):
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
            if (node_pos[1] >   global_map.shape[0]-1 or node_pos[1] < 0 or node_pos[0] > global_map.shape[1]-1 or node_pos[0] < 0):
               continue
            # if (node_pos[0] > (len(global_map) - 1) or node_pos[0] < 0 or
            #    (node_pos[1] > len(global_map[node_pos[0]]) - 1) or node_pos[1] < 0):
            #    continue

            if global_map[node_pos[1]][node_pos[0]] != 0:
                continue

            neibhour_node = Node((node_pos[0], node_pos[1]))
            neibhour_node.prev = current_node
            neibhours.append(neibhour_node)

        for neibhour in neibhours:
            if neibhour in closed_nodes: continue

            # g = current_node.g_cost + 1
            g = Distance_compute(neibhour.location,start_node.location,heuristic)
            h = Distance_compute(neibhour.location,end_node.location,heuristic)
            neibhour.update_cost(g,h*e)
            for onode in open_list:
                if neibhour == onode and neibhour.g_cost > onode.g_cost:
                    continue
            open_list.append(neibhour)
    
def convert_path(path,trans,t):
    npath = []
    for x in path:
        mat = [x[0],x[1]]
        mat = rot2d(mat,t)
        npath.append((mat[0]+trans[0],mat[1]+trans[1]))
    return npath

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
    start = (int(robot_location[0]+9),int(robot_location[1]+10))
    final_goal_location = [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]
    end = (int(final_goal_location[0]+9), int(final_goal_location[1]+10))
    print('Start-',start,' End-',end)
    neibhour_type = '4c'
    heuristic = 'eu'
    heuristic_factor = 10
    final_path = A_STAR(global_map[::-1],start,end,neibhour_type,heuristic_factor,heuristic)
    
    while not rospy.is_shutdown():

        if(final_goal_location != [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]):
            final_goal_location = [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]
            start = (int(robot_location[0]+9),int(robot_location[1]+10))
            end = (int(final_goal_location[0]+9), int(final_goal_location[1]+10))
            print('Start-',start,' End-',end)
            final_path = A_STAR(global_map[::-1],start,end,neibhour_type,heuristic_factor,heuristic)

        OccupancyGrid_publish(global_map[::-1])
        points_publisher(convert_path(final_path,[-9,-10],0))
        goal_location_marker(final_goal_location)




        
        





        
        
