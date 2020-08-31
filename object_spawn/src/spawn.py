#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import tty, termios, sys

import numpy as np 
from std_msgs.msg import Float64, String
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from std_srvs.srv import Empty, EmptyResponse

class SpawnNode():
    model_name = 'ycb_object'
    position = [0.05, 0, 1.17]
    use_keyboard = True

    def __init__(self):
        rospy.init_node('spawn_object', log_level=rospy.INFO)

        self.initial_pose = Pose()
        self.initial_pose.position.x = self.position[0]
        self.initial_pose.position.y = self.position[1]
        self.initial_pose.position.z = self.position[2]
        self.rpy = np.array([0.1,0.1,0.1])

        f = open('/home/avishai/catkin_ws/src/pisa_hand/object_spawn/urdf/object.urdf','r')
        self.urdf_file = f.read()

        # Find object number
        f_copy = open('/home/avishai/catkin_ws/src/pisa_hand/object_spawn/urdf/object.urdf','r')
        datafile = f_copy.readlines()
        for line in datafile:
            l = line.find('/meshes/')
            if l > -1:
                self.obj_num = line[l+8:l+10]
                break
        f_copy.close()
        rospy.loginfo('Spawning object number %s.'%self.obj_num)

        rospy.wait_for_service('gazebo/spawn_urdf_model')
        self.spawn_model_proxy = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        self.delete_model_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

        rospy.Subscriber('finger_state', Float32MultiArray, self.FingerStatesCallback)

        self.open_srv = rospy.ServiceProxy('/OpenGripper', Empty)
        self.close_srv = rospy.ServiceProxy('/CloseGripper', Empty)

        rospy.Service('/respawn', Empty, self.respawn)
        rospy.Service('/spawn_random', Empty, self.random_spawn)

        msg = Empty()

        rate = rospy.Rate(100)
        rospy.loginfo('Reasy to spawn...')
        while not rospy.is_shutdown():

            if self.use_keyboard:
                ch = self.getchar()
                print(ch)

                if ord(ch) == 27:
                    break

                k = self.Move(ch)
                if k == 1:
                    self.respawn(msg)

            rate.sleep()

    def FingerStatesCallback(self, msg):
        self.fingers = np.array(msg.data).reshape((-1,3))

    def respawn(self, msg):
        try:
            res = self.delete_model_proxy(self.model_name)
        except:
            pass

        print("spawn", self.rpy)
        quaternion = R.from_euler('zyx', list(self.rpy), degrees=False).as_quat()
        self.initial_pose.orientation.x = quaternion[0]
        self.initial_pose.orientation.y = quaternion[1]
        self.initial_pose.orientation.z = quaternion[2]
        self.initial_pose.orientation.w = quaternion[3]

        self.spawn_model_proxy(self.model_name, self.urdf_file, "", self.initial_pose, 'world')

        return EmptyResponse()

    def random_quat(self):
        E = np.random.random((3,)) * 2 * np.pi - np.pi
        r = R.from_euler('zyx', list(E), degrees=False)
        self.rpy = np.array(E)
        return r.as_quat()

    def random_spawn(self, msg):
        try:
            res = self.delete_model_proxy(self.model_name)
        except:
            pass

        quaternion = self.random_quat()

        self.initial_pose.orientation.x = quaternion[0]
        self.initial_pose.orientation.y = quaternion[1]
        self.initial_pose.orientation.z = quaternion[2]
        self.initial_pose.orientation.w = quaternion[3]

        self.spawn_model_proxy(self.model_name, self.urdf_file, "", self.initial_pose, 'world')

        return EmptyResponse()

    def getchar(self):
        #Returns a single character from standard input
        
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def Move(self, ch):
        step = 0.01
        if ch == 's': # Record fingers
            return 1 
        if ch == 'x': # Down
            self.initial_pose.position.z -= step   
            return 1         
        if ch == 'w': # Up
            self.initial_pose.position.z += step     
            return 1
        if ch == 'a': # Left
            self.initial_pose.position.y -= step     
            return 1
        if ch == 'd': # Right
            self.initial_pose.position.y += step     
            return 1
        if ch == 'q': # Backward
            self.initial_pose.position.x -= step     
            return 1
        if ch == 'w': # Fardward
            self.initial_pose.position.x += step     
            return 1

        step = 0.1
        if ch == 'i': # roll -
            self.rpy[0] -= step
            print(self.rpy)     
            return 1
        if ch == 'o': # roll +
            self.rpy[0] += step     
            return 1
        if ch == 'k': # pitch -
            self.rpy[1] -= step     
            return 1
        if ch == 'l': # pitch +
            self.rpy[1] += step     
            return 1
        if ch == ',': # yaw -
            self.rpy[2] -= step     
            return 1
        if ch == '.': # yaw +
            self.rpy[2] += step     
            return 1

        if ch == '[': # close hand
            self.close_srv()
            return 2
        if ch == ']': # open hand
            self.open_srv()
            return 2
        if ch == 'r': # Random spawn
            msg = Empty()
            self.random_spawn(msg)
            return 2

        if ch == 't': # Log coordinates
            f = open(r'/home/avishai/catkin_ws/src/pisa_hand/object_spawn/data/finger_logs' + str(self.obj_num) + '_' + str(self.fingers.shape[0]) + '.txt',"a")
            for s in self.fingers.flatten():
                f.write(' ' + str(s))
            f.write('\n')
            f.close()
            return 2
        

if __name__ == '__main__':

    try:
        SpawnNode()
    except rospy.ROSInterruptException:
        pass