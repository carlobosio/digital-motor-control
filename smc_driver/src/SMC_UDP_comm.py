#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import socket
import numpy as np
import SMC_driver
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########

driver1 = SMC_driver.SMC_driver()
print(driver1)
buttons_to_send = [0,0,0,0,0]

######### SUBSCRIBER CALLBACKS #########

def callback(data):
    driver1.set_axis(data.axes[3])
    buttons_to_send[0] = data.buttons[2]
    buttons_to_send[1] = data.buttons[0]
    buttons_to_send[2] = data.buttons[3]
    buttons_to_send[3] = data.buttons[1]
    buttons_to_send[4] = data.buttons[5]
    driver1.set_buttons(buttons_to_send)
    
    
def callback1(pos):
    driver1.set_desired_position(pos.data)

######### ROS NODE, SUBSCRIBERS AND PUBLISHERS INITIALIZATION #########

# ros init
rospy.init_node('Data_exchange', anonymous=True)
# ros subscribe to topics
rospy.Subscriber("joy", Joy, callback)
rospy.Subscriber("des_position", Float32, callback1)
# ros set rate
r = rospy.Rate(100)

print_counter = 0

while not rospy.is_shutdown():	
    
    driver1.send_to_driver()
    
    driver1.receive_from_driver()
    
    print_counter += 1
    if print_counter > 100:
        driver1.driver_echo()
        print_counter = 0
        
    # ROS SLEEP
    r.sleep()
