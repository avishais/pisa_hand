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
from std_msgs.msg import Float64, String
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import JointState
from hand_full_control.srv import TargetAngles

class RodNode():

    angles = None 
    velocities = None
    names = None
    publishers = dict()
    fingers = np.zeros((5,3))
    joint_max_bound = np.deg2rad([30., 60., 60., 45.])
    tip_links = ['soft_hand_thumb_distal_link','soft_hand_index_distal_link','soft_hand_middle_distal_link','soft_hand_ring_distal_link','soft_hand_little_distal_link']
    
    def __init__(self):
        rospy.init_node('HandNode', anonymous=True)

        rospy.Service('/MoveGripper', TargetAngles, self.MoveGripper)
        rospy.Service('/OpenGripper', Empty, self.OpenGripper)
        rospy.Service('/CloseGripper', Empty, self.CloseGripper)

        rospy.Subscriber('/soft_hand/joint_states', JointState, self.JointStatesCallback)
        rospy.loginfo('Waiting for list of controller names...')
        j = 0
        while self.names is None:
            rospy.sleep(0.01)
            j += 1
            if j > 100:
                rospy.logerr('Topic /soft_hand/joint_states not available!')

        rospy.loginfo('Found list of controller names.')
        self.Names = self.names

        for name in self.Names.flatten():
            self.publishers[name] = rospy.Publisher('/soft_hand/' + name[5:] + '_position_controller/command', Float64, queue_size=10)

        self.num_joints = len(self.names.flatten())
        self.num_fingers = self.names.shape[0]
        self.desired_angles = np.zeros((self.num_joints,)).reshape(-1,4)

        msg = Float64MultiArray()

        rate = rospy.Rate(100)
        rospy.loginfo('Running...')
        while not rospy.is_shutdown():

            for finger in range(self.num_fingers):
                for i, name in enumerate(self.Names[finger]):
                    self.publishers[name].publish(self.desired_angles[finger, i])


            rate.sleep()

    def get_finger_index(self, finger_name):
        s = finger_name[10]
        return self.finger_order.index(s)
        
    def JointStatesCallback(self, msg):
        self.names = np.array(msg.name)#[:4]
        self.angles = np.array(msg.position)#[:4]c
        self.velocities = np.array(msg.velocity)#[:4]
        self.effort = np.array(msg.effort)#[:4]

        self.names = np.append(self.names, 'dummy').reshape(-1,4)
        self.angles = np.append(self.angles, 0)
        self.effort = np.append(self.effort, 0).reshape(-1,4)

        self.angles = np.array([(v if np.abs(v) > 1e-2 else 0.) for v in self.angles ]).reshape(-1,4)

        f = [4, 0, 2, 3, 1]
        self.names = self.names[np.array(f),:]
        self.angles = self.angles[np.array(f),:]
        self.effort = self.effort[np.array(f),:]
        
    def MoveGripper(self, msg):
        
        self.desired_angles = msg.angles

        return {'success': True}

    def OpenGripper(self, msg):
        
        self.desired_angles = np.zeros((self.num_joints,)).reshape(-1,4)

        return EmptyResponse()

    def CloseGripper(self, msg):
        
        l = [False] * self.num_fingers
        while not np.all(l):
            for finger in range(self.num_fingers):
                if l[finger]:
                    continue

                if np.any(np.abs(self.effort[finger]) > 10):
                    l[finger] = True
                    continue

                for i, name in enumerate(self.Names[finger]):
                    if name.find('abd') >= 0:
                        continue
                    
                    self.desired_angles[finger, i] += 0.01 if self.desired_angles[finger, i] < self.joint_max_bound[i] else 0
                
                if np.all(self.desired_angles[finger,1:] > self.joint_max_bound[1:]):
                    l[finger] = True

            for finger in range(self.num_fingers):
                for i, name in enumerate(self.Names[finger]):
                    self.publishers[name].publish(self.desired_angles[finger, i])
                
            rospy.sleep(0.01)

        return EmptyResponse()
        

if __name__ == '__main__':

    try:
        RodNode()
    except rospy.ROSInterruptException:
        pass