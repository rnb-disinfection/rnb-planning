import numpy as np
import os

from .global_config import *
from .geometry import GeoSegment

URDF_PATH = os.path.join(PROJ_DIR, "robots", "ur10_robot.urdf")
JOINT_NAMES = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
LINK_NAMES = ['world', 'base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0']
ZERO_JOINT_POSE = np.array([0, -np.pi*0.6 , np.pi*0.6,0,0,0])

def get_collision_items_dict(urdf_content, color=(0,1,0,0.5), display=True):
    collision_items_dict = {'world':[],
     "base_link":[GeoSegment((0,0,0), 'Z', 0.1273, 0.08, 
                                 name="base_capsule", link_name="base_link", urdf_content=urdf_content, color=color, display=display)],
     "shoulder_link":[GeoSegment((0,0,0), 'Y', 0.22094, 0.08, 
                                 name="shoulder_capsule", link_name="shoulder_link", urdf_content=urdf_content, color=color, display=display)],
     "upper_arm_link":[GeoSegment((0,-0.045,0.0), 'Z', 0.612, 0.06, 
                                  name="upper_arm_capsule", link_name="upper_arm_link", urdf_content=urdf_content, color=color, display=display)],
     "forearm_link":[GeoSegment((0,-0,0), 'Z', 0.5723, 0.05, 
                                name="forearm_capsule", link_name="forearm_link", urdf_content=urdf_content, color=color, display=display)],
     "wrist_1_link":[GeoSegment((0,0,0), 'Y', 0.1149, 0.047, 
                                name="wrist_1_capsule", link_name="wrist_1_link", urdf_content=urdf_content, color=color, display=display)],
     "wrist_2_link":[GeoSegment((0,0,0), 'Z', 0.1157, 0.046, 
                                name="wrist_2_capsule", link_name="wrist_2_link", urdf_content=urdf_content, color=color, display=display)],
     "wrist_3_link":[GeoSegment((0,0,0), 'Y', 0.0922-0.0522, 0.046, 
                                name="wrist_3_capsule", link_name="wrist_3_link",urdf_content=urdf_content, color=color, display=display)],
     "tool0":[]
    }
    return collision_items_dict
