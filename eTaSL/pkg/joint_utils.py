from __future__ import print_function
import numpy as np
from .rotation_utils import *
from collections import defaultdict

parent_joint_map = {}
link_adjacency_map = {}
link_adjacency_map_ext = {}
mindist_dict = defaultdict(lambda: dict())

def get_root_chain(lname, urdf_content):
    chain_list = []
    lname_cur = lname
    chain_list.append(lname_cur)
    while lname_cur in parent_joint_map:
        jname_parent = parent_joint_map[lname_cur]
        lname_cur = urdf_content.joint_map[jname_parent].parent
        chain_list.append(lname_cur)
    return chain_list

def joint_dist_list(rchain, urdf_content):
    jdist_list = []
    for lname in rchain:
        jname = get_parent_joint(lname)
        joint = urdf_content.joint_map[jname]
        if joint.type == 'fixed':
            if len(jdist_list)>0:
                jdist_list[-1] = np.add(joint.origin.xyz, np.matmul(Rot_rpy(joint.origin.rpy), jdist_list[-1]))
            else:
                jdist_list.append(np.array(joint.origin.xyz))
        else:
            jdist_list.append(np.array(joint.origin.xyz))
    return jdist_list

def get_link_min_distance(lname1, lname2, urdf_content):
    if lname1 == lname2:
        return 0
    rchain1 = get_root_chain(lname1, urdf_content)
    rchain2 = get_root_chain(lname2, urdf_content)
    for root_common in rchain1:
        if root_common in rchain2:
            break

    rchain1 = rchain1[:rchain1.index(root_common)]
    rchain2 = rchain2[:rchain2.index(root_common)]
    tf_chain1 = joint_dist_list(rchain1, urdf_content)
    tf_chain2 = joint_dist_list(rchain2, urdf_content)
    tf_chain2 = list(reversed(tf_chain2))
    tf_chain = tf_chain1 + tf_chain2
    if len(tf_chain1)>0 and len(tf_chain2)>0:
        len1 = len(tf_chain1)
        tf_chain = tf_chain[:len1-1] + [np.subtract(tf_chain1[-1], tf_chain2[0])]
        if len(tf_chain2)>0:
            tf_chain += tf_chain2[1:]
    len_chain = np.linalg.norm(tf_chain, axis=-1)

    max_len = max(len_chain)
    rest_len = sum(len_chain) - max_len

    min_distance = max_len - rest_len
    return min_distance

def set_min_distance_map(link_names, urdf_content):
    global mindist_dict
    mindist_dict = defaultdict(lambda: dict())
    for idx1 in range(len(link_names)):
        lname1 = link_names[idx1]
        for lname2 in link_names[idx1:]:
            mindist = get_link_min_distance(lname1,lname2, urdf_content)
            mindist_dict[lname1][lname2] = mindist
            mindist_dict[lname2][lname1] = mindist
    return mindist_dict

def get_min_distance_map():
    return mindist_dict

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
    global link_adjacency_map, link_adjacency_map_ext
    link_adjacency_map = {}
    for lname in urdf_content.link_map.keys():
        link_adjacency_map[lname] = __get_adjacent_links(lname, urdf_content)
    link_adjacency_map_ext = {}
    for k, v in link_adjacency_map.items():
        adj_list = []
        for k2 in v:
            adj_list += link_adjacency_map[k2]
        link_adjacency_map_ext[k] = list(set(adj_list))


def get_link_adjacency_map_ext():
    return link_adjacency_map_ext

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
            T_J = SE3(Rot_axis(np.where(parent_joint.axis)[0] + 1, joint_dict[parent_joint.name]), [0,0,0])
        elif parent_joint.type == 'fixed':
            T_J = np.identity(4)
        else:
            raise NotImplementedError
        Toff = SE3(Rot_rpy(parent_joint.origin.rpy), parent_joint.origin.xyz)
        # T = np.matmul(Toff, np.matmul(T_J,T))
        T = matmul_series(Toff,T_J,T)
        link_cur = parent_joint.parent
    return T


def get_tf_full(link_end, joint_dict, urdf_content, from_link='world'):
    T = np.identity(4)
    T_dict = {}
    link_cur = link_end
    while link_cur != from_link:
        parent_joint = urdf_content.joint_map[get_parent_joint(link_cur)]
        if parent_joint.type == 'revolute':
            T_J = SE3(Rot_axis(np.where(parent_joint.axis)[0] + 1, joint_dict[parent_joint.name]), [0,0,0])
        elif parent_joint.type == 'fixed':
            T_J = np.identity(4)
        else:
            raise NotImplementedError
        Toff = SE3(Rot_rpy(parent_joint.origin.rpy), parent_joint.origin.xyz)
        # T = np.matmul(Toff, np.matmul(T_J,T))
#         T = matmul_series(Toff,T_J,T)
        Toff_cur = np.matmul(Toff,T_J)
        for k in T_dict.keys():
            T_dict[k] = np.matmul(Toff_cur, T_dict[k])
        if link_cur not in T_dict:
            T_dict[link_cur] = Toff_cur
        link_cur = parent_joint.parent
    return T_dict

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