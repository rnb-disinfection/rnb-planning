import numpy as np
import os
import subprocess
from enum import Enum
from urdf_parser_py.urdf import URDF

from .global_config import *
from .geometry import GeoSegment
from .rotation_utils import  *

XACRO_PATH_DEFAULT = '{}robots/custom_robots.urdf.xacro'.format(TF_GMT_ETASL_DIR)
URDF_PATH_DEFAULT = '{}robots/custom_robots.urdf'.format(TF_GMT_ETASL_DIR)

URDF_PATH = os.path.join(PROJ_DIR, "robots", "custom_robots.urdf")
JOINT_NAMES = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
LINK_NAMES = ['world', 'base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0']

class XacroCustomizer:
    def __init__(self, rtuples, xyz_rpy_dict, xacro_path = XACRO_PATH_DEFAULT):
        self.xacro_path = xacro_path
        self.subp = None
        self.clear()
        for rtuple in rtuples:
            self.add_robot(rtuple[1], *xyz_rpy_dict[rtuple[0]])
        self.write_xacro()

    @classmethod
    def update_limit_dict(cls, limit_dict, addkey, jnames, values):
        for jname, val in zip(jnames, values):
            if jname not in limit_dict:
                limit_dict[jname] = {}
            limit_dict[jname].update({addkey:val})

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

    def convert_xacro_to_urdf(self, urdf_path=URDF_PATH_DEFAULT, joint_offset_dict={}, joint_limit_dict={},
                              joint_fix_dict={}, vel_limit_dict={}, effort_limit_dict={}):
        urdf_content = subprocess.check_output(['xacro', self.xacro_path])
        self.urdf_content = URDF.from_xml_string(urdf_content)
        for joint in self.urdf_content.joints:
#             if joint.name in vel_limit_dict:
#                 joint.limit.velocity = str(vel_limit_dict[joint.name])
#             if joint.name in effort_limit_dict:
#                 joint.limit.effort = str(effort_limit_dict[joint.name])
            if any([jkey in joint.name for jkey in joint_fix_dict.keys()]):
                lim_dir = [v for k,v in joint_fix_dict.items() if k in joint.name][0]
                joint.type='fixed'
                joint.origin.xyz = list(np.add(joint.origin.xyz, 
                                               np.multiply(joint.axis, getattr(joint.limit, lim_dir))
                                              )
                                       )
                joint.axis = None
                joint.limit  = None
            if joint.name in joint_offset_dict.keys():
                T_reeo_load = joint_offset_dict[joint.name]
                T_e = SE3(Rot_rpy(joint.origin.rpy), joint.origin.xyz)
                T_e = np.matmul(T_e, T_reeo_load)
                joint.origin.xyz, joint.origin.rpy = T2xyzrpy(T_e)
            if joint.name in joint_limit_dict.keys():
                jlim = joint_limit_dict[joint.name]
                if "upper" in jlim:
                    joint.limit.upper = jlim['upper']
                if "lower" in jlim:
                    joint.limit.lower = jlim['lower']
                if "vel" in jlim:
                    joint.limit.velocity = jlim['vel']
                if "acc" in jlim:
                    joint.limit.effort = jlim['acc']
                
        f = open(urdf_path, "w")
        f.write(URDF.to_xml_string(self.urdf_content))
        f.close()
        
        self.joint_names = [joint.name for joint in self.urdf_content.joints if joint.type != 'fixed']
        self.link_names = [link.name for link in self.urdf_content.links]
        return self.joint_names, self.link_names, self.urdf_content
    
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

        
from scipy.spatial.transform import Rotation
from collections import defaultdict
from stl import mesh
from sklearn.decomposition import PCA
from scipy import optimize
import rospkg
rospack = rospkg.RosPack()

def dist_pt_seg(pt, seg):
    P1 = pt
    P21, P22 = seg

    V2 = (P22-P21)
    V2abs = np.linalg.norm(V2) # 4.5 us
    V2nm = V2/V2abs
    V1 = (P1-P21)
    V1prj = np.dot(V1, V2nm) #1.8 us
    if V1prj<0:
        PP = P1-P21
        return np.linalg.norm(PP) # 3.5 us
    elif V1prj<V2abs:
        Vcros = (V2nm[1]*V1[2]-V2nm[2]*V1[1], V2nm[2]*V1[0]-V2nm[0]*V1[2], V2nm[0]*V1[1]-V2nm[1]*V1[0]) # 60 us
        return np.linalg.norm(Vcros) # 6 us
    else:
        PP = P1-P22
        return np.linalg.norm(PP) # 3.5 us
def dist_vertice_seg(vertice, seg):
    return np.max([dist_pt_seg(vtx, seg) for vtx in vertice])
def get_capsule_volume(seg, radii):
    return (np.pi*radii**2)*(radii*4/3 + np.linalg.norm(seg[0]-seg[1]))

def get_min_capsule_volume(vertice, seg):
    radii = dist_vertice_seg(vertice, seg)
    return get_capsule_volume(seg, radii)
def get_min_seg_radii(vertice):
    pca = PCA(3)
    pca_res = pca.fit(vertice)
    pca_dir = pca_res.components_[0]
    vec_init = np.array([(pca_res.mean_ - pca_dir/10), (pca_res.mean_ + pca_dir/10)])
    res = optimize.minimize(
        fun=lambda seg_flat: get_min_capsule_volume(vertice, np.reshape(seg_flat, (2,3))), 
        x0=np.array(vec_init).flatten(), method="Nelder-Mead", options={'disp':False})
    seg = np.reshape(res.x, (2,3))
    radii = dist_vertice_seg(vertice, seg)
    return seg, radii

def get_geometry_items_dict(urdf_content, color=(0,1,0,0.5), display=True, collision=True, exclude_link=[]):
    geometry_items_dict = defaultdict(lambda: list())
    geometry_dir = "./geometry_tmp"
    try: os.mkdir(geometry_dir)
    except: pass
    for link in urdf_content.links:
        skip = False
        for ex_link in exclude_link:
            if ex_link in link.name:
                skip = True
        if skip:
            continue
        for col_item in link.collisions:
            geometry = col_item.geometry
            geotype = geometry.__class__.__name__
#             print("{}-{}".format(link.name, geotype))
            if col_item.origin is None:
                xyz = [0,0,0]
                rpy = [0,0,0]
            else:
                xyz = col_item.origin.xyz
                rpy = col_item.origin.rpy

            if geotype == 'Cylinder':
                geometry_items_dict[link.name] += [GeoSegment(xyz, rpy, geometry.length, geometry.radius,
                                                               name="{}_{}_{}".format(link.name, geotype, len(geometry_items_dict[link.name])),
                                                               link_name=link.name, urdf_content=urdf_content, 
                                                               color=color, display=display, collision=collision
                                                              )]
            elif geotype == 'Mesh':
                name = "{}_{}_{}".format(link.name, geotype, len(geometry_items_dict[link.name]))
                geo_file_name = os.path.join(geometry_dir, name+".npy")
                geo_file_name_bak = os.path.join(geometry_dir, name+"_bak.npy")
                if os.path.isfile(geo_file_name):
                    seg_radius = np.load(geo_file_name)
                    seg, radius = np.reshape(seg_radius[:-1], (2,3)), seg_radius[-1]
                else:
                    filename_split = col_item.geometry.filename.split('/')
                    package_name = filename_split[2]
                    in_pack_path = "/".join(filename_split[3:])
                    file_path = os.path.join(rospack.get_path(package_name), in_pack_path)
                    col_mesh = mesh.Mesh.from_file(file_path)
                    vertice = np.reshape(col_mesh.vectors, (-1,3))
                    seg, radius = get_min_seg_radii(vertice)
                    np.save(geo_file_name, np.concatenate([seg.flatten(), [radius]], axis=0))
                    np.save(geo_file_name_bak, np.concatenate([seg.flatten(), [radius]], axis=0))
                xyz = seg[0]
#                 print('xyz: {}'.format(xyz))
                seg_vec = seg[1]-seg[0]
#                 print('seg_vec: \n{}'.format(seg_vec))
                length = np.linalg.norm(seg_vec)
#                 print('length: {}'.format(length))
                seg_vec_nm = seg_vec/length
                seg_cross = np.cross([0,0,1], seg_vec_nm)
                seg_cross_abs = np.linalg.norm(seg_cross)
                seg_cross_nm = seg_cross/seg_cross_abs
                theta = np.arcsin(seg_cross_abs)
                sign_theta = np.sign(np.dot([0,0,1], seg_vec_nm))
                theta = (sign_theta*theta + np.pi*(1-sign_theta)/2)
                rotvec = seg_cross_nm*theta
#                 print('sign_theta: {}'.format(sign_theta))
#                 print('theta: {}'.format(theta))
#                 print('rotvec: {}'.format(rotvec))
                rpy_mat = Rot_rpy(rpy)
                xyz_rpy = np.matmul(rpy_mat, xyz)
                dcm = np.matmul(rpy_mat, Rotation.from_rotvec(rotvec).as_dcm())
                xyz_rpy = np.add(xyz_rpy, dcm[:,2]*length/2).tolist()
#                 print('xyz_rpy: {}'.format(xyz_rpy))
                quat = Rotation.from_dcm(dcm).as_quat()
                geometry_items_dict[link.name] += [GeoSegment(xyz_rpy, quat, length, radius,
                                                               name=name,
                                                               link_name=link.name, urdf_content=urdf_content, 
                                                               color=color, display=display, collision=collision
                                                              )]
            else:
                raise(NotImplementedError("collision geometry {} is not implemented".format(geotype)))
    return geometry_items_dict

# exclude_parents=['world']
# joint_names=JOINT_NAMES
# def transfer_fixed_links(col_items_dict, urdf_content, joint_names, exclude_parents=['world']):
#     fixed = False
#     zero_dict = joint_list2dict([0]*len(joint_names), joint_names)
#     for joint in urdf_content.joints:
#         parent_name = joint.parent
#         if joint.type == 'fixed' and joint.parent not in exclude_parents:
#             for k,v in col_items_dict.items():
#                 if k == joint.child:
#                     for ctem in v:
#                         Toff = get_tf(ctem.link_name, zero_dict, urdf_content, from_link=parent_name)
#                         ctem.set_link(parent_name)
#                         ctem.center = (np.matmul(Toff[:3,:3], ctem.center) + Toff[:3,3]).tolist()
#                         ctem.orientation = Rotation.from_dcm(
#                             np.matmul(Toff[:3,:3], Rotation.from_quat(ctem.orientation).as_dcm())).as_quat()
#                         col_items_dict[parent_name] += [ctem]
#                     del col_items_dict[k]
#                     fixed = True
#     if fixed:
#         transfer_fixed_links(col_items_dict, urdf_content, joint_names, exclude_parents)
# transfer_fixed_links(col_items_dict, urdf_content, joint_names=JOINT_NAMES)

def make_mesh_backup(meshname):
    val = np.load('./geometry_tmp/'+meshname+'.npy')
    print("["+", ".join(["%.3f"%v for v in val])+"]")
    np.save('./geometry_tmp/'+meshname+'_bak.npy', val)

def refine_meshes():    
    val = [0,  0.005, 0.053, 
           0, 0.02, 0.02, 
           0.018]
    np.save('./geometry_tmp/panda1_rightfinger_Mesh_0.npy', val)
    
    val = [0,  0.005, 0.055, 
           0, 0.02, 0.02, 
           0.018]
    np.save('./geometry_tmp/panda1_leftfinger_Mesh_0.npy', val)

    meshname = 'panda1_link0_Mesh_0'
    val = [0.02,  0.0,   0.04,  
           -0.08, 0.0,  0.02,
           0.10]
    np.save('./geometry_tmp/'+meshname+'.npy', val)

    meshname = 'panda1_link1_Mesh_0'
    val = [0,  -0.055,   -0.015,  
           0, -0.000,  -0.135,
           0.11]
    np.save('./geometry_tmp/'+meshname+'.npy', val)

    meshname = 'panda1_link2_Mesh_0'

    val = [0,  0.0,  0.07,  
           0, -0.16,  0.00,
           0.09]
    np.save('./geometry_tmp/'+meshname+'.npy', val)

    meshname = 'panda1_link3_Mesh_0'
    val = [0.08, 0.054, 0.00,
           0.00, 0, -0.07,
           0.09]
    np.save('./geometry_tmp/'+meshname+'.npy', val)

    meshname = 'panda1_link4_Mesh_0'
    val = [0, 0, 0.054, 
           -0.111, 0.116, 0, 
           0.08]
    np.save('./geometry_tmp/'+meshname+'.npy', val)


    meshname = 'panda1_link5_Mesh_0'
    val = [0.00, 0.063, -0.00, 
           -0.00, 0.00, -0.230, 
           0.08]

    np.save('./geometry_tmp/'+meshname+'.npy', val)

    meshname = 'panda1_link6_Mesh_0'
    val = [0.080, 0.000, 0.000, 
           -0.010, 0.000, 0.000, 
           0.09]

    np.save('./geometry_tmp/'+meshname+'.npy', val)

    meshname = 'panda1_hand_Mesh_0'
    val = [0.000, 0.070, 0.01, 
           -0.000, -0.070, 0.01, 
           0.06]


    np.save('./geometry_tmp/'+meshname+'.npy', val)

