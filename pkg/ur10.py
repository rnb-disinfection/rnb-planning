import numpy as np
import os

ROBOT_PATH = "/home/junsu/etasl/ws/etasl-py/src/etasl_py_examples/robots/"
URDF_PATH = os.path.join(ROBOT_PATH, "ur10_robot.urdf")
JOINT_NAMES = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
LINK_NAMES = ['world', 'base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0']
ZERO_JOINT_POSE = np.array([0, -np.pi*0.6 , np.pi*0.6,0,0,0])