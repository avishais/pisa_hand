#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


rospy.init_node('insert_object',log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 0

f = open('/home/avishai/catkin_ws/src/pisa_hand/object_spawn/urdf/object.urdf','r')
urdff = f.read()

rospy.wait_for_service('gazebo/spawn_urdf_model')
spawn_model_proxy = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
spawn_model_proxy("ycb_object", urdff, "", initial_pose, "world")