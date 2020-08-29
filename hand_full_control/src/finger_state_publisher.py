#!/usr/bin/python 

'''
----------------------------
Author: Avishai Sintov
        Tel-Aviv University
Date: August 2020
----------------------------
'''


import rospy
import numpy as np 
from std_msgs.msg import Float64, Float32MultiArray
from gazebo_msgs.msg import LinkStates
from rosgraph_msgs.msg import Clock

class finger_state_publisher():

    fingers = np.zeros((5,3))
    current_time = 0.0
    prev_time = 0.0
    tip_links = ['soft_hand_thumb_distal_link','soft_hand_index_middle_link','soft_hand_middle_distal_link','soft_hand_ring_distal_link','soft_hand_little_distal_link']
    base_label = 'box'
    base = np.array([0,0,1.0])

    def __init__(self):
        rospy.init_node('finger_state_publisher', anonymous=True)
        rospy.wait_for_message('/gazebo/link_states', LinkStates)

        rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkStatesCallback)
        rospy.Subscriber('/clock', Clock, self.ClockCallback)
        state_pub = rospy.Publisher('/finger_state', Float32MultiArray, queue_size=10)
        
        

        msg = Float32MultiArray()

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():

            msg.data = self.fingers.flatten()
            state_pub.publish(msg)

            rate.sleep()

    def linkStatesCallback(self, msg):

        names = msg.name

        for i, tip in enumerate(self.tip_links):
            try:
                j = names.index('soft_hand::' + tip)
                self.fingers[i, :] = np.array([msg.pose[j].position.x, msg.pose[j].position.y, msg.pose[j].position.z]) - self.base
            except:
                continue


    def ClockCallback(self, msg):
        self.current_time = msg.clock.secs + msg.clock.nsecs * 1e-9


if __name__ == '__main__':

    try:
        finger_state_publisher()
    except rospy.ROSInterruptException:
        pass