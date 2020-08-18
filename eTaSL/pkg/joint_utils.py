from __future__ import print_function
from scipy.spatial.transform import Rotation
import numpy as np
from rotation_utils import *

parent_joint_map = {}

def set_parent_joint_map(urdf_content):
    global parent_joint_map
    for link_name in urdf_content.link_map.keys():
        for joint in urdf_content.joints:
            if joint.child == link_name:
                parent_joint_map[link_name] = joint.name

def get_parent_joint(link_name):
    return parent_joint_map[link_name]

def get_tf(to_link, joint_dict, urdf_content, from_link='world'):
    T = np.identity(4)
    link_cur = to_link
    while link_cur != from_link:
        parent_joint = urdf_content.joint_map[get_parent_joint(link_cur)]
        if parent_joint.type == 'revolute':
            T_J = SE3(Rot_rotvec(np.array(parent_joint.axis)*joint_dict[parent_joint.name]), [0,0,0])
        elif parent_joint.type == 'fixed':
            T_J = np.identity(4)
        else:
            raise NotImplementedError
        Toff = SE3(Rot_rpy(parent_joint.origin.rpy), parent_joint.origin.xyz)
        # T = np.matmul(Toff, np.matmul(T_J,T))
        T = matmul_series(Toff,T_J,T)
        link_cur = parent_joint.parent
    return T

def get_transformation(link_name):
    return "T_{link_name}".format(link_name=link_name)

def joint_list2dict(joint_list, joint_names):
    return {joint_names[i]: joint_list[i] for i in range(len(joint_names))}

def get_joint_names_csv(joint_names, urdf_content):
    jnames_format = ""
    for jname in joint_names:
        if jname not in urdf_content.joint_map \
                or urdf_content.joint_map[jname].axis is  None:
            raise("invalid joint name is defined")
        jnames_format = jnames_format+'"%s",'%jname
    jnames_format = jnames_format[:-1]
    return jnames_format