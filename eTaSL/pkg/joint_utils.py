from __future__ import print_function
from scipy.spatial.transform import Rotation
import numpy as np

def get_parent_joint(link_name, urdf_content):
    for joint in urdf_content.joints:
        if joint.child == link_name:
            return joint.name
        
def get_tf(to_link, joint_dict, urdf_content, from_link='world'):
    T = np.identity(4)
    link_cur = to_link
    while link_cur != from_link:
        parent_joint = urdf_content.joint_map[get_parent_joint(link_cur, urdf_content)]
        T_J = np.identity(4)
        if parent_joint.type == 'revolute':
            T_J[:3,:3] = Rotation.from_rotvec(np.array(parent_joint.axis)*joint_dict[parent_joint.name]).as_dcm()
        elif parent_joint.type == 'fixed':
            pass
        else:
            raise NotImplementedError  
        Toff = np.identity(4)
        Toff[:3,3] = parent_joint.origin.xyz
        Toff[:3,:3] = Rotation.from_euler("xyz", 
                                          parent_joint.origin.rpy, degrees=False).as_dcm()
        T = np.matmul(Toff, np.matmul(T_J,T))
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