import numpy as np
import os

from .global_config import *
from .geometry import GeoSegment

URDF_PATH = os.path.join(PROJ_DIR, "robots", "ur10_robot_dual.urdf")
JOINT_NAMES = ["shoulder_pan_joint_1","shoulder_lift_joint_1","elbow_joint_1","wrist_1_joint_1","wrist_2_joint_1","wrist_3_joint_1",
               "shoulder_pan_joint_2","shoulder_lift_joint_2","elbow_joint_2","wrist_1_joint_2","wrist_2_joint_2","wrist_3_joint_2"]
LINK_NAMES = ['world',
              'base_link_1', 'shoulder_link_1', 'upper_arm_link_1', 'forearm_link_1', 'wrist_1_link_1', 'wrist_2_link_1', 'wrist_3_link_1', 'tool0_1',
              'base_link_2', 'shoulder_link_2', 'upper_arm_link_2', 'forearm_link_2', 'wrist_1_link_2', 'wrist_2_link_2', 'wrist_3_link_2', 'tool0_2']
ZERO_JOINT_POSE = np.array([0, -np.pi*0.6 , np.pi*0.6,0,0,0]+[0, -np.pi*0.6 , np.pi*0.6,0,0,0])

def get_geometry_items_dict(urdf_content, color=(0,1,0,0.5), display=True):
    geometry_items_dict = {
        "world":[],
        "base_link_1":[GeoSegment((0,0,0), 'Z', 0.1273, 0.08, 
                                  name="base_capsule_1", link_name="base_link_1", urdf_content=urdf_content, color=color, display=display)],
        "shoulder_link_1":[GeoSegment((0,0,0), 'Y', 0.22094, 0.08, 
                                      name="shoulder_capsule_1", link_name="shoulder_link_1", urdf_content=urdf_content, color=color, display=display)],
        "upper_arm_link_1":[GeoSegment((0,-0.045,0.0), 'Z', 0.612, 0.06, 
                                       name="upper_arm_capsule_1", link_name="upper_arm_link_1", urdf_content=urdf_content, color=color, display=display)],
        "forearm_link_1":[GeoSegment((0,-0,0), 'Z', 0.5723, 0.05, 
                                     name="forearm_capsule_1", link_name="forearm_link_1", urdf_content=urdf_content, color=color, display=display)],
        "wrist_1_link_1":[GeoSegment((0,0,0), 'Y', 0.1149, 0.047, 
                                     name="wrist_1_capsule_1", link_name="wrist_1_link_1", urdf_content=urdf_content, color=color, display=display)],
        "wrist_2_link_1":[GeoSegment((0,0,0), 'Z', 0.1157, 0.046, 
                                     name="wrist_2_capsule_1", link_name="wrist_2_link_1", urdf_content=urdf_content, color=color, display=display)],
        "wrist_3_link_1":[GeoSegment((0,0,0), 'Y', 0.0922-0.0522, 0.046, 
                                     name="wrist_3_capsule_1", link_name="wrist_3_link_1",urdf_content=urdf_content, color=color, display=display)],
        "tool0_1":[],
        "base_link_2":[GeoSegment((0,0,0), 'Z', 0.1273, 0.08, 
                                  name="base_capsule_2", link_name="base_link_2", urdf_content=urdf_content, color=color, display=display)],
        "shoulder_link_2":[GeoSegment((0,0,0), 'Y', 0.22094, 0.08, 
                                      name="shoulder_capsule_2", link_name="shoulder_link_2", urdf_content=urdf_content, color=color, display=display)],
        "upper_arm_link_2":[GeoSegment((0,-0.045,0.0), 'Z', 0.612, 0.06, 
                                       name="upper_arm_capsule_2", link_name="upper_arm_link_2", urdf_content=urdf_content, color=color, display=display)],
        "forearm_link_2":[GeoSegment((0,-0,0), 'Z', 0.5723, 0.05, 
                                     name="forearm_capsule_2", link_name="forearm_link_2", urdf_content=urdf_content, color=color, display=display)],
        "wrist_1_link_2":[GeoSegment((0,0,0), 'Y', 0.1149, 0.047, 
                                     name="wrist_1_capsule_2", link_name="wrist_1_link_2", urdf_content=urdf_content, color=color, display=display)],
        "wrist_2_link_2":[GeoSegment((0,0,0), 'Z', 0.1157, 0.046, 
                                     name="wrist_2_capsule_2", link_name="wrist_2_link_2", urdf_content=urdf_content, color=color, display=display)],
        "wrist_3_link_2":[GeoSegment((0,0,0), 'Y', 0.0922-0.0522, 0.046, 
                                     name="wrist_3_capsule_2", link_name="wrist_3_link_2",urdf_content=urdf_content, color=color, display=display)],
        "tool0_2":[]
    }
    return geometry_items_dict
