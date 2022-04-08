#!/usr/bin/env python2
# -*- coding: utf-8 -*-


from kobuki_msgs.msg import BumperEvent
import rospy

from std_msgs.msg import String

def process_Bumper(data):
    global bump
    bumper=""
    if (data.state == BumperEvent.PRESSED):
        bump = True
        bumper="True"
    else:
        bump = False
        bumper="False"
    message_publisher.publish(bumper)
    rospy.loginfo("Bumper Event")
    rospy.loginfo(data.bumper)
    
    

#END MEASUREMENT

rospy.init_node('bumper',anonymous=True)
rospy.Subscriber('/bumper', BumperEvent, process_Bumper)
message_publisher = rospy.Publisher("bumper_topic", String, queue_size=10) 
rospy.spin() 