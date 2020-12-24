from __future__ import print_function
import numpy as np
from .rotation_utils import *
from collections import defaultdict

link_adjacency_map = {}
link_adjacency_map_ext = {}
mindist_dict = defaultdict(dict)

def get_root_chain(lname, urdf_content):
    chain_list = []
    lname_cur = lname
    chain_list.append(lname_cur)
    while lname_cur in urdf_content.parent_map:
        jname_parent, lname_cur = urdf_content.parent_map[lname_cur]
        chain_list.append(lname_cur)
    return chain_list

def joint_dist_list(rchain, urdf_content):
    jdist_list = []
    for lname in rchain:
        jname = get_parent_joint(lname, urdf_content)
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

def get_min_distance_map(urdf_content):
    link_names = urdf_content.link_map.keys()
    mindist_dict = defaultdict(dict)
    for idx1 in range(len(link_names)):
        lname1 = link_names[idx1]
        for lname2 in link_names[idx1:]:
            mindist = get_link_min_distance(lname1,lname2, urdf_content)
            mindist_dict[lname1][lname2] = mindist
            mindist_dict[lname2][lname1] = mindist
    return mindist_dict

def get_parent_joint(link_name, urdf_content):
    return urdf_content.parent_map[link_name][0]

def get_link_adjacency_map(urdf_content):
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
    return link_adjacency_map, link_adjacency_map_ext

def get_link_adjacency_map_ext():
    return link_adjacency_map_ext

def __get_adjacent_links(link_name, urdf_content, adjacent_links=None, propagate=True):
    if adjacent_links is None:
        adjacent_links = []
    if link_name in adjacent_links:
        return adjacent_links
    adjacent_links += [link_name]
    if link_name in urdf_content.parent_map:
        pjname, plname  = urdf_content.parent_map[link_name]
        pjoint = urdf_content.joint_map[pjname]
        if not (propagate or pjoint.type == 'fixed'):
            return adjacent_links
        adjacent_links = __get_adjacent_links(plname, urdf_content, adjacent_links, propagate and pjoint.type == 'fixed')
    if link_name in urdf_content.child_map:
        child_list = urdf_content.child_map[link_name]
        for cjname, clname in child_list:
            cjoint = urdf_content.joint_map[cjname]
            if not (propagate or cjoint.type == 'fixed'):
                return adjacent_links
            adjacent_links = __get_adjacent_links(clname, urdf_content, adjacent_links, propagate and cjoint.type == 'fixed')
    return list(set(adjacent_links))

def get_joint_tf(joint, joint_dict):
    if joint.type == 'revolute':
        T_J = SE3(Rot_axis(np.where(joint.axis)[0] + 1, joint_dict[joint.name]), [0,0,0])
    elif joint.type == 'fixed':
        T_J = np.identity(4)
    else:
        raise NotImplementedError
    Toff = SE3(Rot_rpy(joint.origin.rpy), joint.origin.xyz)
    # T = np.matmul(Toff, np.matmul(T_J,T))
    return np.matmul(Toff,T_J)

def get_tf(to_link, joint_dict, urdf_content, from_link='base_link'):
    T = np.identity(4)
    link_cur = to_link
    while link_cur != from_link:
        pjname = get_parent_joint(link_cur, urdf_content)
        if pjname is None:
            break
        parent_joint = urdf_content.joint_map[pjname]
        Tj = get_joint_tf(parent_joint, joint_dict)
        T = np.matmul(Tj,T)
        link_cur = parent_joint.parent
    return T


def get_tf_full(link_end, joint_dict, urdf_content, from_link='base_link'):
    T = np.identity(4)
    T_dict = {}
    link_cur = link_end
    while link_cur != from_link:
        parent_joint = urdf_content.joint_map[get_parent_joint(link_cur, urdf_content)]
        Toff_cur = get_joint_tf(parent_joint, joint_dict)
        for k in T_dict.keys():
            T_dict[k] = np.matmul(Toff_cur, T_dict[k])
        if link_cur not in T_dict:
            T_dict[link_cur] = Toff_cur
        link_cur = parent_joint.parent
    return T_dict

def build_T_chain(link_names, joint_dict, urdf_content, Tlink_dict={}):
    for lname in link_names:
        if lname not in Tlink_dict:
            if lname not in urdf_content.parent_map:
                Tlink_dict[lname] = np.identity(4)
                continue
            pjname, plname = urdf_content.parent_map[lname]
            if plname not in Tlink_dict:
                build_T_chain(list(set(link_names)-set([lname])), joint_dict, urdf_content, Tlink_dict)
            Tp = Tlink_dict[plname]
            Tlink_dict[lname] = np.matmul(Tp, get_joint_tf(urdf_content.joint_map[pjname], joint_dict))
    return Tlink_dict            

def get_joint_names_csv(joint_names):
    jnames_format = ""
    for jname in joint_names:
        jnames_format = jnames_format+'"%s",'%jname
    jnames_format = jnames_format[:-1]
    return jnames_format

def make_colliding_list(geometry_items1, geometry_items2=None, min_distance_map=None, link_adjacency_map_ext=None):
    idx1 = 0
    collision_list = []
    collision_ext_list = []
    for ctem1 in geometry_items1:
        idx1 += 1
        if geometry_items2 is None:
            geometry_items_tmp = geometry_items1[idx1:]
        else:
            geometry_items_tmp = geometry_items2

        for ctem2 in geometry_items_tmp:
            if ctem2.link_name in ctem1.adjacent_links or ctem1.link_name in ctem2.adjacent_links:
                continue
            else:
                if min_distance_map is not None:
                    min_link_dist = min_distance_map[ctem1.link_name][ctem2.link_name]
                    min_col_dist = min_link_dist - (np.linalg.norm(ctem1.get_off_max()) + np.linalg.norm(ctem2.get_off_max()))
                    if min_col_dist > 0:
                        continue
                collision_list.append((ctem1, ctem2))
                if ctem2.link_name not in link_adjacency_map_ext[ctem1.link_name] and \
                        ctem1.link_name not in link_adjacency_map_ext[ctem2.link_name]:
                    collision_ext_list.append((ctem1, ctem2))
    if link_adjacency_map_ext is not None:
        return collision_list, collision_ext_list
    return collision_list