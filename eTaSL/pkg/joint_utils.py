from __future__ import print_function
from scipy.spatial.transform import Rotation
import numpy as np
from .rotation_utils import *

parent_joint_map = {}
link_adjacency_map = {}

def set_parent_joint_map(urdf_content):
    global parent_joint_map
    for link_name in urdf_content.link_map.keys():
        for joint in urdf_content.joints:
            if joint.child == link_name:
                parent_joint_map[link_name] = joint.name

def get_adjacent_links(link_name):
    return link_adjacency_map[link_name]

def get_parent_joint(link_name):
    return parent_joint_map[link_name]

def set_link_adjacency_map(urdf_content):
    global link_adjacency_map
    for lname in urdf_content.link_map.keys():
        link_adjacency_map[lname] = __get_adjacent_links(lname, urdf_content)

def __get_adjacent_links(link_name, urdf_content, adjacent_links=None, propagate=True):
    if adjacent_links is None:
        adjacent_links = []
    if link_name in adjacent_links:
        return adjacent_links
    adjacent_links += [link_name]
    for k, v in urdf_content.joint_map.items():
        if v.parent == link_name:
            if not (propagate or v.type == 'fixed'):
                return adjacent_links
            adjacent_links = __get_adjacent_links(v.child, urdf_content, adjacent_links, propagate and v.type == 'fixed')
        elif v.child == link_name:
            if not (propagate or v.type == 'fixed'):
                return adjacent_links
            adjacent_links = __get_adjacent_links(v.parent, urdf_content, adjacent_links, propagate and v.type == 'fixed')
    return list(set(adjacent_links))

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