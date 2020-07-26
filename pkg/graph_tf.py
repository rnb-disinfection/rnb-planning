import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras import backend as K
import numpy as np
from itertools import combinations

from collections import defaultdict

from pkg.tf_transform import *
from pkg.tf_robot import *
from pkg.constraint import *
from pkg.info import *
from pkg.tf_utils import *
from pkg.rotation_utils import *
from pkg.utils import *
from pkg.ur10 import *
from pkg.geometry import *
from pkg.collision import *
from pkg.distance import *
from pkg.distance_calculator import *
from pkg.binding_calculator import *

class GraphModel(tf.keras.Model):
    def __init__(self, robot_info, gitem_list, binfo_list, urdf_content, N_sim, 
                 alpha_jc=5, alpha_fc=200, alpha_jl=1, alpha_cl=1,alpha_cs=0, dQ_max = tf.constant([[1.0]*6])*2e-1,
                 LIM=np.pi, LIM_BOUND=1e-1, COL_BOUND=1e-2, learning_rate=5e-3, col_iteration = 20):
        super(GraphModel, self).__init__()
        self.alpha_jc = alpha_jc
        self.alpha_fc = alpha_fc
        self.alpha_jl = alpha_jl
        self.alpha_cl = alpha_cl
        self.alpha_cs = alpha_cs
        self.N_sim = N_sim
        self.LIM = LIM
        self.LIM_BOUND = LIM_BOUND
        self.COL_BOUND = COL_BOUND
        self.dQ_max = tf.constant(dQ_max)
        self.learning_rate = learning_rate
        self.col_iteration = col_iteration
        self.robot_info = robot_info
        self.robot = RobotLayer(
            robot_info, dim=N_sim)
        self.joint_constraint = JointConstraintLoss(self.robot)
        self.frame_constraint = FrameConstraintLoss()
        self.robot_base = [robot_info.base_frame]*N_sim
        self.set_custom_loss(lambda *args: tf.constant(0.0), 0)
        self.link_name_list = self.robot.link_name_list
        self.link_idx_dict = self.robot.link_idx_dict
        self.adjacency_list = self.robot.adjacency_list
        self.dependency_list = self.robot.dependency_list
        self.dependency_mat = self.robot.dependency_mat
            
        self.object_dict = {}
        self.object_name_list = []
        for gitem in gitem_list:
            self.object_name_list += [gitem.name]
            self.object_dict[gitem.name] = ObjectLayer(gitem, N_sim, self.robot.num_link)
        self.num_objects = len(self.object_name_list)
        
        self.binding_dict = {}
        self.binding_name_list = []
        self.binding_index_list = []
        for binfo in binfo_list:
            self.binding_name_list += [binfo.name]
            self.binding_dict[binfo.name] = BindingLayer(**binfo.get_kwargs(self.object_dict))
            self.binding_index_list += [self.object_name_list.index(binfo.obj_name)]
        self.binding_index_list = tf.constant(self.binding_index_list)
        self.num_binding = len(self.binding_name_list)
        
        self.col_cal = CollisionCalculator(self.object_name_list, self.object_dict, self.robot, self.N_sim)
        self.bind_cal = BindingCalculator(self.binding_name_list, self.binding_dict, self.robot, self.N_sim)
        self.optimizer = tf.optimizers.SGD(learning_rate=learning_rate)
            
    def assign_frame_dict(self, gframeset_list):
        frame_dict = {k: [] for k in self.object_dict.keys()}
        link_dict = {k: [] for k in self.object_dict.keys()}
        for gframeset in gframeset_list:
            for k, gframe in gframeset.items():
                frame_dict[k] += [gframe.Toff]
                link_dict[k] += [self.robot.link_name_list.index(gframe.link_name)]
        for k in frame_dict.keys():
            self.object_dict[k].set_frame(np.array(frame_dict[k]), np.array(link_dict[k]))
            
        self.col_cal.set_link_dependence(link_dict)
        self.col_cal.set_pair_mask()
        self.bind_cal.set_link_dependence(link_dict)
        
    def set_slack_batch(self, slack_batch):
        self.bind_cal.set_pair_mask(slack_batch)
        
    def update_slack_frames(self, gframe_dict_list, T_all, Tbo_all):
        for i_s in range(self.N_sim):
            slack_pairs = self.bind_cal.slack_pair_list[i_s]
            for pair in slack_pairs:
                oname1 = self.bind_cal.object_name_list[pair[0]]
                oname2 = self.bind_cal.object_name_list[pair[1]]
                obj1 = self.bind_cal.object_dict[oname1]
                obj2 = self.bind_cal.object_dict[oname2]
                if obj1.btype.is_binder():
                    binder = obj1.obj_layer
                    bindee = obj2.obj_layer
                else:
                    binder = obj2.obj_layer
                    bindee = obj1.obj_layer
                i_link_binder = binder.link_idx_list[i_s]
                ibindee = self.object_name_list.index(bindee.gitem.name)
                Tbo_bindee = Tbo_all[i_s,ibindee]
                Tbl_binder = T_all[i_s, i_link_binder]
                Tlo = np.matmul(np.linalg.inv(Tbo_bindee), Tbl_binder)
                gframe_dict_list[i_s][bindee.gitem.name].link_name = self.link_name_list[i_link_binder]
                gframe_dict_list[i_s][bindee.gitem.name].Toff = Tlo
        return gframe_dict_list
            
    @tf.function
    def assign_Q(self, Q):
        self.robot.assign_Q(Q)
            
    @tf.function
    def get_Q(self):
        return self.robot.get_Q()

    @tf.function
    def call(self, inputs=None):
        T_all = self.robot(self.robot_base)
        Tbo_all = []
        for obj_name in self.object_name_list:
            Tbo_all += [self.object_dict[obj_name](T_all)] #(Nobj,N_sim,4,4)
        Tbo_all = K.stack(Tbo_all, axis=1) #(N_sim,Nobj,4,4)
        Tbo_bnd = tf.gather(Tbo_all, self.binding_index_list, axis=-3)
        Tbb_all = []
        for i_bnd, bnd_name in zip(range(self.num_binding), self.binding_name_list):
            Tbb_all += [self.binding_dict[bnd_name](tf.gather(Tbo_bnd, i_bnd, axis=-3))] #(Nobj,N_sim,4,4)
        Tbb_all = K.stack(Tbb_all, axis=1) #(N_sim,Nobj,4,4)
        return T_all, Tbo_all, Tbb_all
    
    @tf.function
    def calc_joint_limit(self):
        Q = self.get_Q()
        return K.sum(1/((K.min(((self.LIM-Q), (self.LIM+Q)), axis=-1)/self.LIM_BOUND)**3))
    
    @tf.function
    def calc_loss(self, T_all, Tbo_all, Qtar, binQ, Ttar, binT):
        jl_loss = self.calc_joint_limit()
        jc_loss = self.joint_constraint((Qtar, binQ))
        fc_loss = self.frame_constraint((T_all[:,-1,:,:],Ttar, binT))
        #cl_loss = self.calc_collision_loss(T_all, Tbo_all)
        return self.alpha_jc*jc_loss + self.alpha_fc*fc_loss + \
                self.alpha_jl*jl_loss + self.alpha_cs*self.calc_custom_loss(T_all, Tbo_all) \
                #+ self.alpha_cl*cl_loss 
                
    
    @tf.function
    def forward(self, Qtar, binQ, Ttar, binT):
        T_all, Tbo_all= self(None)
        loss = self.calc_loss(T_all, Tbo_all, Qtar, binQ, Ttar, binT)
        return loss
    
    @tf.function
    def update_once(self, Qtar, binQ, Ttar, binT, max_gradient):
        with tf.GradientTape() as g:
            # Forward pass.
            loss = self.forward(Qtar, binQ, Ttar, binT)

        # Variables to update, i.e. trainable variables.
        trainable_variables = self.trainable_variables

        # Compute gradients.
        gradients = g.gradient(loss, trainable_variables)
        gradients = tf.unstack(clip_gradient(gradients, max_gradient))
    
        # Update W and b following gradients.
        self.optimizer.apply_gradients(zip(gradients, self.trainable_variables))
        return loss
    
    def set_custom_loss(self, custom_loss, alpha_cs, *args, **kwargs):
        self.custom_loss = custom_loss
        self.alpha_cs = alpha_cs
        self.cl_args = args
        self.cl_kwargs = kwargs
        
    def calc_custom_loss(self, T_all, Tbo_all):
        return self.custom_loss(self, T_all, Tbo_all, *self.cl_args, **self.cl_kwargs)

    @tf.function
    def test_collision(self, T_all, Tbo_all):
        Pbo_all = tf.gather(tf.gather(Tbo_all, [0,1,2], axis=-2), 3, axis=-1)
        col_combs_masked_batch = self.get_collision_combination()
        batch_flat = tf.repeat(np.reshape(np.arange(self.N_sim), (self.N_sim, 1)), (self.N_col,)*self.N_sim,axis=0)
        col_combs_masked_flat = tf.concat([batch_flat, col_combs_masked_batch], axis=-1)
        vtx_tf = tf.gather(
            K.sum(tf.expand_dims(Tbo_all, axis=-3)*tf.expand_dims(self.object_vertice_mat, axis=-2), axis=-1), 
            [0,1,2], axis=-1)
        vtx_tf_1 = tf.reshape(tf.gather_nd(vtx_tf, tf.gather(col_combs_masked_flat, [0,1], axis=1)), (self.N_sim, self.N_col, -1,3))
        vtx_tf_2 = tf.reshape(tf.gather_nd(vtx_tf, tf.gather(col_combs_masked_flat, [0,2], axis=1)), (self.N_sim, self.N_col, -1,3))
        Pbo_all_1 = tf.reshape(tf.gather_nd(Pbo_all, tf.gather(col_combs_masked_flat, [0,1], axis=1)), (self.N_sim, self.N_col,3))
        Pbo_all_2 = tf.reshape(tf.gather_nd(Pbo_all, tf.gather(col_combs_masked_flat, [0,2], axis=1)), (self.N_sim, self.N_col,3))
        v_batch = tf.expand_dims(Pbo_all_1-Pbo_all_2, axis=-2)
        v_batch = tf.stop_gradient(tf.linalg.cross(v_batch, self.x_batch)+tf.linalg.cross(v_batch, self.y_batch))
        dist, flag = test_collision_batch(vtx_tf_1, vtx_tf_2, 
                             v_batch, self.flag_default, self.dist_default, IterationAllowed=self.col_iteration)
        return dist, flag, col_combs_masked_flat 
    
    @tf.function
    def calc_collision_loss(self, T_all, Tbo_all):
        dist, flag, _  = self.test_collision(T_all, Tbo_all)
        return K.sum(1/((dist/self.COL_BOUND)**3))
    
    def jacobian(self, T_all, Tbo_all, Tbb_all):
        P_jnt, axis_jnt = self.robot.prepare_joint_axis(T_all)
        return (
            self.robot.jacobian_link(T_all, P_jnt, axis_jnt), 
            self.col_cal.jacobian_object(Tbo_all, P_jnt, axis_jnt), 
            self.bind_cal.jacobian_object(Tbb_all, P_jnt, axis_jnt),
            self.bind_cal.jacobian_rot(Tbb_all, axis_jnt)
        )
        