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
    
    def __init__(self):
        rospy.init_node('HandNode', anonymous=True)

        self.Kp = 10#np.array([k1, k1 ,k2]) # Springs coefficients
        self.Kd = 1#np.array([100, 100, .1]) # Damping

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
        print(self.Names)

        for name in self.Names:
            self.publishers[name] = rospy.Publisher('/soft_hand/' + name[5:] + '_position_controller/command', Float64, queue_size=10)

        self.num_joints = len(self.names)
        self.desired_angles = np.zeros((self.num_joints,))

        msg = Float32MultiArray()

        rate = rospy.Rate(100)
        rospy.loginfo('Starting...')
        while not rospy.is_shutdown():

            for i, name in enumerate(self.Names):
                self.publishers[name].publish(self.desired_angles[i])

            # print("Names: ", self.Names)
            # print("Angles: ", np.rad2deg(self.angles))
            # print("Effort: ", self.effort)

            rate.sleep()

    def JointStatesCallback(self, msg):
        self.names = msg.name#[:4]
        self.angles = np.array(msg.position)#[:4]
        self.velocities = np.array(msg.velocity)#[:4]
        self.effort = np.array(msg.effort)#[:4]

        self.angles = np.array([(v if np.abs(v) > 1e-2 else 0.) for v in self.angles ])
        # self.velocities = np.array([(v if np.abs(v) > 1e-2 else 0.) for v in self.velocities ])

    def MoveGripper(self, msg):
        
        self.desired_angles = msg.angles

        return {'success': True}

    def OpenGripper(self, msg):
        
        self.desired_angles = np.zeros((self.num_joints,))

        return EmptyResponse()

    def CloseGripper(self, msg):
        
        while np.any(self.effort < 10):
            for i, name in enumerate(self.Names):
                if name.find('abd') < 0:
                    self.desired_angles[i] += 0.05

            for i, name in enumerate(self.Names):
                self.publishers[name].publish(self.desired_angles[i])
            
            rospy.sleep(0.1)

        return EmptyResponse()
        

if __name__ == '__main__':

    try:
        RodNode()
    except rospy.ROSInterruptException:
        pass