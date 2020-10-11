from threading import Thread
import numpy as np
from copy import deepcopy

import rospy
from franka_interface import GripperInterface
from sensor_msgs.msg import JointState
from franka_core_msgs.msg import EndPointState, JointCommand, RobotState

PANDA_SIMULATION_INTERFACE = "/panda_simulator"
PANDA_REAL_INTERFACE = "franka_ros_interface"
# rospy.init_node('panda_joint_controller', anonymous=True)

class PandaStateSubscriber:
    def __init__(self, interface_name):
        self.state = None
        self.interface_name = interface_name
        
    def callback(self, data):
        self.state = data
        
    def start_subsciption(self):
        rospy.Subscriber('{}/custom_franka_state_controller/joint_states'.format(self.interface_name), JointState, self.callback)
        t = Thread(target=lambda: rospy.spin())
        t.start()
        
class PandaControlPublisher:
    def __init__(self, interface_name, update_rate=100,
                 arm_joint_names=['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7'],
                 finger_joint_names=['panda_finger_joint1', 'panda_finger_joint2']
                ):
        self.arm_pub = rospy.Publisher('{}/motion_controller/arm/joint_commands'.format(interface_name), JointCommand, tcp_nodelay=True, queue_size=1)
        self.finger_pub = rospy.Publisher('{}/franka_gripper/move'.format(interface_name), JointCommand, tcp_nodelay=True, queue_size=1)
        self.rate = rospy.Rate(update_rate) # 100hz
        self.arm_joint_names = arm_joint_names
        self.finger_joint_names = finger_joint_names
        self.init_commands()
        self.gi = GripperInterface()
        
    def init_commands(self):
        self.arm_cmd = JointCommand()
        self.arm_cmd.names = self.arm_joint_names
        self.arm_cmd.mode = JointCommand.POSITION_MODE
        
    def joint_move_arm(self, target_position):
        self.arm_cmd.position = target_position
        self.arm_pub.publish(self.arm_cmd)
        
    def close_finger(self):
        t = Thread(target=lambda: self.gi.close())
        t.start()
        
    def open_finger(self):
        t = Thread(target=lambda: self.gi.open())
        t.start()
            