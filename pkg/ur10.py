import numpy as np
import os
from .rotation_utils import *
from .geometry import *

ROBOT_PATH = "/home/junsu/etasl/ws/etasl-py/src/etasl_py_examples/robots/"
URDF_PATH = os.path.join(ROBOT_PATH, "ur10_robot.urdf")
JOINT_NAMES = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
LINK_NAMES = ['world', 'base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0']
ZERO_JOINT_POSE = np.array([0, -np.pi*0.6 , np.pi*0.6,-np.pi/2,-np.pi/2,0])

def get_link_items_offsets(color=(0,1,0,0.5), display=True):
    return [
        GeometryItem(name='base_capsule', gtype=GeoType.CYLINDER, dims=[0.16,0.16,0.1273], color=color, display=display, collision=True),
        GeometryItem(name='shoulder_capsule', gtype=GeoType.CYLINDER, dims=[0.16,0.16,0.22094], color=color, display=display, collision=True),
        GeometryItem(name='upper_arm_capsule', gtype=GeoType.CYLINDER, dims=[0.12,0.12,0.612], color=color, display=display, collision=True),
        GeometryItem(name='forearm_capsule', gtype=GeoType.CYLINDER, dims=[0.1,0.1,0.5723], color=color, display=display, collision=True),
        GeometryItem(name='wrist_1_capsule', gtype=GeoType.CYLINDER, dims=[0.094,0.094,0.1149], color=color, display=display, collision=True),
        GeometryItem(name='wrist_2_capsule', gtype=GeoType.CYLINDER, dims=[0.092,0.092,0.1157], color=color, display=display, collision=True),
        GeometryItem(name='wrist_3_capsule', gtype=GeoType.CYLINDER, dims=[0.092,0.092,0.0922-0.0522], color=color, display=display, collision=True),
        GeometryItem(name='tool_mesh', gtype=GeoType.MESH, dims=[1e-3,1e-3,1e-3], uri="package://my_mesh/meshes/stl/AirPick_cup_ctd.stl", color=(0.1,0.1,0.1,1), display=True, collision=False)
    ], {
        "base_capsule":GeometryFrame(SE3(Rot_zyx(0,0,0),(0,0,0.06365)), "base_link"),
        "shoulder_capsule":GeometryFrame(SE3(Rot_zyx(0,0,np.pi/2),(0,0.11047,0)), "shoulder_link"),
        "upper_arm_capsule":GeometryFrame(SE3(Rot_zyx(0,0,0),(0,-0.045,0.306)), "upper_arm_link"),
        "forearm_capsule":GeometryFrame(SE3(Rot_zyx(0,0,0),(0,0,0.28665)), "forearm_link"),
        "wrist_1_capsule":GeometryFrame(SE3(Rot_zyx(0,0,np.pi/2),(0,0.05745,0)), "wrist_1_link"),
        "wrist_2_capsule":GeometryFrame(SE3(Rot_zyx(0,0,0),(0,0,0.05785)), "wrist_2_link"),
        "wrist_3_capsule":GeometryFrame(SE3(Rot_zyx(0,0,np.pi/2),(0,0.02,0)), "wrist_3_link"),
        "tool_mesh":GeometryFrame(SE3(Rot_zyx(0,0,0),(0,0,0)), "tool0")
    }