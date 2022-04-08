#!/usr/bin/env python
import rospy
from std_msgs.msg import String#Callback function to print the subscribed data on the terminal
from geometry_msgs.msg import Twist
global BUMPERCOUNT
BUMPERCOUNT=0
import random

#2 arrays for estimating average velocity over time
global x_vel
global z_vel

x_vel=[]
z_vel=[]

def callback_str(subscribedData):
     global x_vel
     global z_vel
     
     rospy.loginfo('Subscribed: ' + subscribedData.data)#Subscriber node function which will subscribe the messages from the Topic
     string=str(subscribedData.data)
     ranges=string.split("#")
     range_center=float(ranges[0])
     range_left=float(ranges[1])             
     range_right=float(ranges[2])
     obstacle_description =""
     if range_center > 1 and range_left > 1 and range_right > 1:
        obstacle_description = 'case 1 - nothing'
        linear_x = 0.6
        angular_z = 0
        x_vel.append(linear_x )
        z_vel.append(angular_z )
     elif range_center < 1 and range_left > 1 and range_right > 1:
        obstacle_description = 'case 2 - front'
        linear_x = 0
        angular_z = -0.3
     elif range_center > 1 and range_left > 1 and range_right < 1:
        obstacle_description = 'case 3 - right'
        linear_x = 0
        angular_z = -0.3
        x_vel.append(linear_x )
        z_vel.append(angular_z )
     elif range_center > 1 and range_left < 1 and range_right > 1:
        obstacle_description = 'case 4 - left'
        linear_x = 0
        angular_z = 0.3
        x_vel.append(linear_x )
        z_vel.append(angular_z )
     elif range_center < 1 and range_left > 1 and range_right < 1:
        obstacle_description = 'case 5 - front and right'
        linear_x = 0
        angular_z = -0.3
        x_vel.append(linear_x )
        z_vel.append(angular_z )
     elif range_center < 1 and range_left < 1 and range_right > 1:
        obstacle_description = 'case 6 - front and left'
        linear_x = 0
        angular_z = 0.3
        x_vel.append(linear_x )
        z_vel.append(angular_z )
     elif range_center < 1 and range_left < 1 and range_right < 1:
        obstacle_description = 'case 7 - front and left and right'
        linear_x = 0
        angular_z = -0.3
        x_vel.append(linear_x )
        z_vel.append(angular_z )
     elif range_center > 1 and range_left < 1 and range_right < 1:
        obstacle_description = 'case 8 - left and right'
        linear_x = 0
        angular_z = -0.3
        x_vel.append(linear_x )
        z_vel.append(angular_z )
     
     rospy.loginfo( obstacle_description)
     vel_msg.linear.x = linear_x
     vel_msg.angular.z = angular_z
     
     velocity_publisher.publish(vel_msg)
     
     average_linear=random.choices(x_vel)
     average_angular=random.choices(z_vel)
     
     print("average Linear Velocity {} ".format(average_linear))
     print("average angular Velocity {} ".format(average_angular))
 
def callback_bumper(subscribedData):
     global BUMPERCOUNT
     rospy.loginfo('Subscribed: ' + subscribedData.data)#Subscriber node function which will subscribe the messages from the Topic
     string=str(subscribedData.data)
     if string=="True":
         BUMPERCOUNT+=1
         print(BUMPERCOUNT)

def messageSubscriber():
    #initialize the subscriber node called 'messageSubNode'
    rospy.init_node('velocity_controller', anonymous=False)    #This is to subscribe to the messages from the topic named 'messageTopic'
    rospy.Subscriber('messageTopic', String, callback_str) 
    rospy.Subscriber('bumper_topic', String, callback_bumper)#rospy.spin() stops the node from exitind until the node has been shut down
    
    rospy.spin()


if __name__ == '__main__':
    try:
        velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        vel_msg =Twist()
        
        messageSubscriber()
    except rospy.ROSInterruptException:
        pass