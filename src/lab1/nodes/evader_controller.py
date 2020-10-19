#!/usr/bin/python
import roslib
roslib.load_manifest('lab1')
import rospy
import tf
import math
import random
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import time

left = 0
right =  0
front =  0

# Broadcast to /robot_1/odom frame
def tf_broadcaster(msg, robot_name):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0,msg.pose.pose.orientation.z),
                     rospy.Time.now(),
                     robot_name,
                     "/robot_1/odom")  


def callback(msg):
    global front, right, left
    range_arr = np.array(msg.ranges)
    left  = np.mean(range_arr[241:]) # Mean range of left 120 values
    front = np.mean(range_arr[121:240]) # Mean range of front 120 values
    right = np.mean(range_arr[0:120]) # Mean range of right 120 values
    # print('robot_0 Laser-',(left,front,right))

if __name__ == '__main__':
    rospy.init_node('evader_controller')
    vel_0 = rospy.Publisher('/robot_0/cmd_vel', geometry_msgs.msg.Twist,queue_size=1) # Publish Command to robot_0
    sub = rospy.Subscriber('/robot_0/base_scan', LaserScan, callback) # Receive Laser Msg from /stage
    robot_name = '/robot_0/odom' # TF Broadcast robot
    rospy.Subscriber(robot_name,Odometry,tf_broadcaster,robot_name)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():

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
