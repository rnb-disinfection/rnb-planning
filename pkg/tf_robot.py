import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras import backend as K
import numpy as np

from pkg.tf_transform import *
from pkg.joint_utils import *
from pkg.info import *

class LinkLayer(layers.Layer):
    def __init__(self, link_info, dim, Q=None, idx=None, *args, **kwargs):
        if "name" not in kwargs:
            kwargs["name"] = link_info.lname
        super(LinkLayer, self).__init__(*args, **kwargs)
        self.link_info = link_info
        self.Toff = tf.constant(link_info.Toff)
        self.axis = link_info.axis
        self.parent = link_info.parent
        self.ltype = link_info.ltype
        self.lname = link_info.lname
        self.dim = dim
        self.Tjnt = JointMatLayer(axis=self.axis, dim=self.dim, ltype=self.ltype, name="{}_joint".format(kwargs["name"]))
        
        self.Q = Q
        self.idx = idx
        if idx is None or self.ltype == 'fixed':
            self.q = tf.constant([[0.0]]*self.dim, name="{}_q".format(kwargs["name"]))
            self.get_q = self.__fix_q
        else:
            self.get_q = self.__var_q
        
    # 변수를 만듭니다.
    def build(self, input_shape):
        pass
    
    def __fix_q(self):
        return self.q
    
    def __var_q(self):
        return tf.gather(self.Q, [self.idx], axis=-1)

    # call 메서드가 그래프 모드에서 사용되면
    # training 변수는 텐서가 됩니다.
    @tf.function
    def call(self, inputs):
        Tpre = inputs
        Tjnt = self.Tjnt(tf.gather(self.get_q(), 0,axis=-1))
        Tpo = tf.matmul(Tpre,self.Toff)
        Tcur = tf.matmul(Tpo,Tjnt)
        return Tcur
    

class RobotLayer(layers.Layer):
    def __init__(self, robot_info, dim, *args, **kwargs):
        link_info_list = robot_info.link_info_list
        rname = robot_info.rname
        if "name" not in kwargs:
            kwargs["name"] = rname
        super(RobotLayer, self).__init__(*args, **kwargs)
        self.link_info_list = link_info_list
        self.link_name_list = []
        self.link_list = []
        self.rname = rname
        self.dim = dim
        self.DOF = len([True for link_info in self.link_info_list if link_info.ltype != 'fixed'])
        self.Q = tf.Variable([[0.0]*self.DOF]*self.dim, name="{}_q".format(kwargs["name"]), trainable=False)
        self.idx_var = []
        for i_q, link_info in zip(range(len(self.link_info_list)), self.link_info_list):
            if link_info.ltype != 'fixed':
                idx_Q = len(self.idx_var)
                self.idx_var += [i_q]
            else:
                idx_Q = None
            self.link_name_list += [link_info.lname]
            self.link_list += [LinkLayer(link_info, dim=self.dim, name="{}_{}".format(rname,link_info.lname), Q=self.Q, idx=idx_Q)]
        self.num_link = len(self.link_list)
        self.link_idx_dict = {name: idx for name, idx in zip(self.link_name_list, range(len(self.link_name_list)))}
        self.adjacency_list = [
            list(map(
                lambda x: self.link_idx_dict[x],
                [lname for lname in get_adjacent_links(link_name, robot_info) if lname in self.link_name_list]
            )) for link_name in self.link_name_list
        ]
        link_num = len(self.adjacency_list)
        self.adjacency_mat = np.zeros((link_num,link_num), dtype=np.bool)
        for i_adj, adj in zip(range(link_num), self.adjacency_list):
            self.adjacency_mat[i_adj, adj] = True
        self.dependency_list = [[self.link_idx_dict[lname] for lname in get_parent_links(link.lname, robot_info)] for link in link_info_list]
        self.dependency_mat = np.zeros((link_num,link_num), dtype=np.bool)
        for i_dep, dep in zip(range(link_num), self.dependency_list):
            self.dependency_mat[i_dep, dep] = True
            
        self.mask_depend = tf.reshape(
            tf.gather(tf.cast(self.dependency_mat, dtype = tf.float32), self.idx_var, axis=-1)
            , (1, self.num_link, self.DOF, 1))
        self.mask_prism = tf.cast(tf.reshape(
            np.array([self.link_info_list[self.idx_var[i_q]].ltype == 'prismatic' for i_q in range(self.DOF)]), 
            (1,1,self.DOF,1)), dtype=tf.float32)
        self.mask_revol = tf.cast(tf.reshape(
            np.array([self.link_info_list[self.idx_var[i_q]].ltype == 'revolute' for i_q in range(self.DOF)]), 
            (1,1,self.DOF,1)), dtype=tf.float32)
        self._axis_prism = tf.reshape(tf.one_hot([self.link_list[idx_q].axis-1 for idx_q in self.idx_var], 3), (1, self.DOF, 1, 3))
            
        
    def assign_Q(self, Q):
        self.Q.assign(Q)
    
    @tf.function
    def get_Q(self):
        return self.Q
        
    # 변수를 만듭니다.
    def build(self, input_shape):
        pass

    # call 메서드가 그래프 모드에서 사용되면
    # training 변수는 텐서가 됩니다.
    @tf.function
    def call(self, inputs):
        T_dict = {None: inputs}
        T_list = []
        for link in self.link_list:
            Tcur = link(T_dict[link.parent])
            T_dict[link.lname] = Tcur
            T_list += [Tcur]
        T_list = K.stack(T_list, axis=1)
        return T_list
    
    @tf.function
    def prepare_joint_axis(self, T_all):
        RP_all = tf.gather(T_all, [0,1,2], axis=-2)
        R_all = tf.gather(RP_all, [0,1,2], axis=-1) # N_sim, N_link, 3, 3
        P_all = tf.gather(RP_all, [3], axis=-1) # N_sim, N_link, 3, 1
        R_jnt = tf.gather(R_all, self.idx_var, axis=-3) # N_sim, DOF, 3, 3
        P_jnt = tf.gather(P_all, self.idx_var, axis=-3) # N_sim, DOF, 3, 1
        axis_jnt = K.sum(R_jnt*self._axis_prism, axis=-1) # N_sim, DOF, 3
        return P_jnt, axis_jnt
    
    @tf.function
    def jacobian_link(self, T_all, P_jnt, axis_jnt):
        return self.jacobian_object(T_all, self.num_link, self.mask_depend, P_jnt, axis_jnt)
    
    def jacobian_object(self, Tbo_all, N_obj, mask_depend, P_jnt, axis_jnt): # mask_depend: 1, N_obj, DOF, 1
        RP_all = tf.gather(Tbo_all, [0,1,2], axis=-2)
        R_all = tf.gather(RP_all, [0,1,2], axis=-1) # N_sim, N_obj, 3, 3
        P_all = tf.gather(RP_all, [3], axis=-1) # N_sim, N_obj, 3, 1

        jac_prism_ = tf.pad(axis_jnt, [[0,0], [0,0], [0,3]]) # N_sim, DOF, 3+3
        jac_prism = tf.expand_dims(jac_prism_, axis=-3) * mask_depend # N_sim, N_obj, DOF, 6

        axis_jnt_rep = tf.tile(tf.expand_dims(axis_jnt, axis=-3), [1,N_obj,1,1]) # N_sim, N_obj, DOF, 3
        pij = tf.reshape(tf.expand_dims(P_all, axis=-3)-tf.expand_dims(P_jnt, axis=-4), (self.dim, N_obj, self.DOF, 3)) # N_sim, N_obj, DOF, 3
        jac_p_rev = tf.linalg.cross(pij, axis_jnt_rep) # N_sim, N_obj, DOF, 3
        jac_revol = tf.concat([jac_p_rev, axis_jnt_rep], axis=-1) * mask_depend # N_sim, N_obj, DOF, 6

        jacobian = self.mask_prism*jac_prism + self.mask_revol*jac_revol # N_sim, N_obj, DOF, 6
        return jacobian
    
    def jacobian_rot(self, pij, N_obj, mask_depend, axis_jnt):
        # pij: N_sim, N_obj, N_axis, Dim / mask_depend: 1, N_obj, DOF, 1 / axis_jnt: N_sim, DOF, 3
        pij = tf.tile(tf.expand_dims(pij, axis=-2), [1,1,1,self.DOF,1]) # N_sim, N_obj, N_axis, DOF, Dim
        axis_jnt_rep = tf.reshape(tf.tile(tf.expand_dims(axis_jnt, axis=-3), [1,N_obj*3,1,1]), 
                                  (self.dim, N_obj, 3, self.DOF, 3)) # N_sim, N_obj, N_axis, DOF, 3
        jac_p_rev = tf.linalg.cross(pij, axis_jnt_rep) * tf.expand_dims(mask_depend, axis=-3) # N_sim, N_obj, N_axis, DOF, 3

        return tf.expand_dims(self.mask_revol, axis=-3)*jac_p_rev # N_sim, N_obj, N_axis, DOF, 3


def get_link_info_list(link_names, urdf_content):
    joint_names = list(urdf_content.joint_map.keys())
    len_joints = len(joint_names)
    zero_joint_dict = joint_list2dict([0]*len_joints, joint_names)
    link_info_list = []
    for lname in link_names:
        jname = get_parent_joint(lname, urdf_content)
        if jname is None:
            link_info_list += [
                LinkInfo(
                    Toff=np.identity(4, dtype=np.float32),
                    axis=3,
                    ltype="fixed",
                    lname=lname,
                    parent=None
                )
            ]
            continue
        parent_name = get_parent_link(lname, urdf_content)
        joint= urdf_content.joint_map[jname]
        link_info_list += [
            LinkInfo(
                Toff=get_tf(
                    lname, zero_joint_dict, urdf_content=urdf_content, 
                    from_link=parent_name).astype(np.float32),
                axis=np.where(joint.axis)[0][0]+1 if joint.axis is not None else 3,
                ltype=joint.type,
                lname=lname,
                parent=parent_name
            )
        ]
    return link_info_list


def get_adjacent_links_urdf(link_name, urdf_content):
    adjacent_links = [link_name]
    for k, v in urdf_content.joint_map.items():
        if v.parent == link_name:
            adjacent_links += [v.child]
        if v.child == link_name:
            adjacent_links += [v.parent]
    return list(set(adjacent_links))

def get_adjacent_links(link_name, robot_info):
    adjacent_links = [link_name]
    for v in robot_info.link_info_list:
        if v.parent == link_name:
            adjacent_links += [v.lname]
        if v.lname == link_name:
            adjacent_links += [v.parent]
    return list(set(adjacent_links))

def get_parent_links(link_name, robot_info):
    parent_links = [link_name] if link_name else []
    for v in robot_info.link_info_list:
        if v.lname == link_name:
            parent_links = get_parent_links(v.parent, robot_info) + parent_links
    return parent_links