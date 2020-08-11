import numpy as np
import os
import subprocess
from enum import Enum
from urdf_parser_py.urdf import URDF

from .global_config import *
from .geometry import GeoSegment

XACRO_PATH_DEFAULT = '{}robots/custom_robots.urdf.xacro'.format(TF_GMT_ETASL_DIR)
URDF_PATH_DEFAULT = '{}robots/custom_robots.urdf'.format(TF_GMT_ETASL_DIR)

URDF_PATH = os.path.join(PROJ_DIR, "robots", "custom_robots.urdf")
JOINT_NAMES = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
LINK_NAMES = ['world', 'base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0']
ZERO_JOINT_POSE = np.array([0, -np.pi*0.6 , np.pi*0.6,0,0,0])

class RobotType(Enum):
    indy7_robot=0
    panda_robot=1
    
class XacroCustomizer:
    def __init__(self, xacro_path = XACRO_PATH_DEFAULT):
        self.xacro_path = xacro_path
        self.subp = None
        self.clear()

    def add_robot(self, rtype, xyz=[0,0,0], rpy=[0,0,0]):
        rexpression = \
            '<xacro:{rtype} robot_id="{rid}" xyz="{xyz}" rpy="{rpy}" connected_to="world"/>'.format(
                rtype=rtype.name, rid=self.rid_count, xyz='{} {} {}'.format(*xyz), rpy='{} {} {}'.format(*rpy)
            )
        self.rexpression_list += [rexpression]
        self.rid_count += 1
        
    def write_xacro(self):
        xacro_file = open(self.xacro_path, "r")

        new_xacro_content = ""
        toggle_body = False
        for line in xacro_file:
            stripped_line = line.strip()
            if "BODY BEGIN" in stripped_line:
                toggle_body = True
                new_xacro_content += stripped_line +"\n"
            elif "BODY END" in stripped_line:
                toggle_body = False
                for rexpression in self.rexpression_list:
                    new_xacro_content += rexpression +"\n"
                new_xacro_content += stripped_line +"\n"
            elif not toggle_body:
                new_xacro_content += stripped_line +"\n"
        xacro_file.close()

        xacro_file = open(self.xacro_path, "w")
        xacro_file.write(new_xacro_content)
        xacro_file.close()

    def convert_xacro_to_urdf(self, urdf_path=URDF_PATH_DEFAULT, joint_fix_dict={}):
        urdf_content = subprocess.check_output(['xacro', self.xacro_path])
        self.urdf_content = URDF.from_xml_string(urdf_content)
        for joint in self.urdf_content.joints:
            if any([jkey in joint.name for jkey in joint_fix_dict.keys()]):
                lim_dir = [v for k,v in joint_fix_dict.items() if k in joint.name][0]
                joint.type='fixed'
                joint.origin.xyz = list(np.add(joint.origin.xyz, 
                                               np.multiply(joint.axis, getattr(joint.limit, lim_dir))
                                              )
                                       )
                
        f = open(urdf_path, "w")
        f.write(URDF.to_xml_string(self.urdf_content))
        f.close()
        
        self.joint_names = [joint.name for joint in self.urdf_content.joints if joint.type != 'fixed']
        self.link_names = [link.name for link in self.urdf_content.links]
        self.zero_joint_pose = np.zeros((len(self.joint_names),))
        return self.joint_names, self.link_names, self.zero_joint_pose, self.urdf_content
    
    def start_rviz(self):
        self.kill_existing_subprocess()
        self.subp = subprocess.Popen(['roslaunch', '{}launch/gui_custom_robots.launch'.format(TF_GMT_ETASL_DIR)])
        
    def kill_existing_subprocess(self):
        if self.subp is not None:
            self.subp.terminate()
        self.subp = None

    def clear(self):
        self.rexpression_list = []
        self.rid_count = 0
        self.kill_existing_subprocess()

def get_collision_items_dict(urdf_content, color=(0,1,0,0.5), display=True, collision=True):
    collision_items_dict = {'world':[],
     "base_link":[GeoSegment((0,0,0), 'Z', 0.1273, 0.08, 
                                 name="base_capsule", link_name="base_link", urdf_content=urdf_content, color=color, display=display, collision=collision)],
     "shoulder_link":[GeoSegment((0,0,0), 'Y', 0.22094, 0.08, 
                                 name="shoulder_capsule", link_name="shoulder_link", urdf_content=urdf_content, color=color, display=display, collision=collision)],
     "upper_arm_link":[GeoSegment((0,-0.045,0.0), 'Z', 0.612, 0.06, 
                                  name="upper_arm_capsule", link_name="upper_arm_link", urdf_content=urdf_content, color=color, display=display, collision=collision)],
     "forearm_link":[GeoSegment((0,-0,0), 'Z', 0.5723, 0.05, 
                                name="forearm_capsule", link_name="forearm_link", urdf_content=urdf_content, color=color, display=display, collision=collision)],
     "wrist_1_link":[GeoSegment((0,0,0), 'Y', 0.1149, 0.047, 
                                name="wrist_1_capsule", link_name="wrist_1_link", urdf_content=urdf_content, color=color, display=display, collision=collision)],
     "wrist_2_link":[GeoSegment((0,0,0), 'Z', 0.1157, 0.046, 
                                name="wrist_2_capsule", link_name="wrist_2_link", urdf_content=urdf_content, color=color, display=display, collision=collision)],
     "wrist_3_link":[GeoSegment((0,0,0), 'Y', 0.0922-0.0522, 0.046, 
                                name="wrist_3_capsule", link_name="wrist_3_link",urdf_content=urdf_content, color=color, display=display, collision=collision)],
     "tool0":[]
    }
    return collision_items_dict
