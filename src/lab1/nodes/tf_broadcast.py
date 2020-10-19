#!/usr/bin/python  
import roslib
roslib.load_manifest('lab1')
import rospy
from nav_msgs.msg import Odometry
import tf
import turtlesim.msg
import math

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0), # POSE X Y Z
                     tf.transformations.quaternion_from_euler(0, 0,msg.pose.pose.orientation.z), # ROT X Y Z
                     rospy.Time.now(),
                     turtlename,
                     "world")

if __name__ == '__main__':
    rospy.init_node('tf_broadcast')
    turtlename = '/robot_0/odom'
    rospy.Subscriber(turtlename,
                     Odometry,
                     handle_turtle_pose,
                     turtlename)
                     
    turtlename = '/robot_1/odom'
    rospy.Subscriber(turtlename,
                     Odometry,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()