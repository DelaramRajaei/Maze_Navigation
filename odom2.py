#!/usr/bin/env python2
# -*- coding: utf-8 -*-


import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

path = Path()

global coke_count
coke_count=0



def callback(msg):
    global path
    global coke_count
    print (msg.pose.pose)
    
    if(msg.pose.pose.position.x==0  and msg.pose.pose.position.y==0 and msg.pose.pose.position.z==0 ):
       coke_count+=1
    print("coke count is {}".format( coke_count))
      
    
#    path.header = msg.header
#    pose = PoseStamped()
#    pose.header = msg.header
#    pose.pose = msg.pose.pose
#    path.poses.append(pose)
#    path_pub.publish(path)
#    zaviye_enheraf=math.atan(1.5-msg.pose.pose.position.y/1.5-msg.pose.pose.position.x)
#    print("zaviyeh enheraf :")
#    print( zaviye_enheraf)
#    
#  
#  




if __name__ == '__main__':
    rospy.init_node('check_odometry')
    odom_sub = rospy.Subscriber('/odom' ,Odometry,callback)
    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    
    rospy.spin()
   
   
