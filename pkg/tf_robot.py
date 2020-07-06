import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras import backend as K
import numpy as np

from pkg.tf_transform import *
from pkg.joint_utils import *
from pkg.info import *

class LinkLayer(layers.Layer):
    def __init__(self, link_info, dim, *args, **kwargs):
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
        
        if self.ltype == 'fixed':
            self.q = tf.constant([0.0]*self.dim, name="{}_q".format(kwargs["name"]))
        else:
            self.q = tf.Variable([0.0]*self.dim, name="{}_q".format(kwargs["name"]))
        
    # 변수를 만듭니다.
    def build(self, input_shape):
        pass

    # call 메서드가 그래프 모드에서 사용되면
    # training 변수는 텐서가 됩니다.
    @tf.function
    def call(self, inputs):
        Tpre = inputs
        Tjnt = self.Tjnt(self.q)
        Tpo = tf.matmul(Tpre,self.Toff)
        Tcur = tf.matmul(Tpo,Tjnt)
        return Tcur
    

class RobotLayer(layers.Layer):
    def __init__(self, link_info_list, rname, dim, *args, **kwargs):
        if "name" not in kwargs:
            kwargs["name"] = rname
        super(RobotLayer, self).__init__(*args, **kwargs)
        self.link_info_list = link_info_list
        self.link_name_list = []
        self.link_list = []
        self.rname = rname
        self.dim = dim
        for link_info in self.link_info_list:
            self.link_name_list += [link_info.lname]
            self.link_list += [LinkLayer(link_info, dim=self.dim, name="{}_{}".format(rname,link_info.lname))]
        self.len_Q = len(self.link_list)
        
    def assign_Q(self, Q):
        for i_q in range(self.len_Q):
            link = self.link_list[i_q]
            if link.ltype != 'fixed':
                link.q.assign(Q[:,i_q])
    
    def get_Q(self):
        Q = []
        for link in self.link_list:
            Q += [link.q]
        Q = K.stack(Q, axis=1)#, name="{}_Q".format(self.rname))
        return Q
        
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