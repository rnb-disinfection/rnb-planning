from .geometry import *
from .collision import *
from .distance import *
from .distance_calculator import *
            

class BindingType(Enum):
    VACC = 1
    SUCC = 2
    PLANE = 3
    
    def full_axis(self):
        N_axis = len(BIND_AXIS_DICT[self])
        return np.pad(BIND_AXIS_DICT[self], [[0, 3-N_axis], [0,0]])
    
    def num_axis(self):
        return len(BIND_AXIS_DICT[self])
    
    def is_binder(self):
        return BINDER_DICT[self]

BIND_AXIS_DICT = {
    BindingType.VACC: [[0,0,-0.1]],
    BindingType.SUCC: [[0,0,0.1]],
    BindingType.PLANE: [[0,0,0.1]]
}

BINDING_DICT = {
    BindingType.VACC: [BindingType.SUCC],
    BindingType.SUCC: [BindingType.VACC, BindingType.PLANE],
    BindingType.PLANE: [BindingType.SUCC]
}

BINDER_DICT = {
    BindingType.VACC: True,
    BindingType.SUCC: False,
    BindingType.PLANE: True
}


class BindingInfo:
    def __init__(self, name, btype, obj_name, gtype=None, Toff=np.identity(4), dims=(0,0,0), counterparts=None):
        self.name, self.btype, self.obj_name, self.gtype = name, btype, obj_name, gtype
        self.Toff, self.dims = Toff, dims
        if counterparts is None:
            counterparts = BINDING_DICT[btype]
        self.counterparts = counterparts
        
    def get_gitem(self):
        if self.gtype is None:
            self.gitem = self.parent.gitem
        else:
            self.gitem = GeometryItem(self.name, self.gtype, self.dims, color=(0,0,0,1), 
                                      display=False, collision=False, uri=None)
        return self.gitem
    
    def get_kwargs(self, object_dict):
        self.parent = object_dict[self.obj_name]
        return dict(btype=self.btype, obj_layer=self.parent, gitem=self.get_gitem(), 
                    Toff=self.Toff, counterparts=self.counterparts)
    
    
    
class BindingLayer(layers.Layer):
    def __init__(self, btype, obj_layer, gitem=None, Toff=np.identity(4), counterparts=None, *args, **kwargs):
        self.obj_layer = obj_layer
        self.gitem = gitem
        if self.gitem is None:
            self.gitem = obj_layer.gitem
        self.btype, self.counterparts = btype, counterparts
        self.Toff = tf.constant(Toff,dtype='float32')
        self.Raxis = tf.constant(btype.full_axis(),dtype='float32')
        self.Naxis = btype.num_axis()
        if self.counterparts is None:
            self.counterparts = [self.btype]
        super(BindingLayer, self).__init__(*args, **kwargs)
        
    @tf.function
    def call(self, input):
        T_bo = input # (N_sim, N_link, 4,4)
        self.T_bb = tf.matmul(T_bo, self.Toff)
        return self.T_bb
    
    @tf.function
    def get_axis(self):
        return self.Raxis
    
    
class BindingCalculator(DistanceCalculator):
    def __init__(self, *args, **kwargs):
        super(BindingCalculator, self).__init__(*args, **kwargs)
        self.set_rot_axis()
        
    @tf.function
    def angle_dist(self, Tbo_all_res): # (N_sim, 1, graph.num_objects, 1, 4,4))
        Rbo_all = tf.gather(
            tf.gather(tf.gather(Tbo_all_res, [0,1,2], axis=-2), [0,1,2], axis=-1),
            0, axis=1) # N_sim, N_object, (N_axis), 3, 3
         # 1, N_obj, N_axis, 1, 3
        axis_all = K.sum(Rbo_all*self.obj_axis_exp, axis=-1) # N_sim, N_object, N_axis=3, 3
        axis1_all = tf.gather(axis_all, self.pair_obj1, axis=-3) # N_sim, N_rot, N_axis=3, 3
        axis2_all = tf.gather(axis_all, self.pair_obj2, axis=-3) # N_sim, N_rot, N_axis=3, 3
        axis1_all = tf.expand_dims(axis1_all, axis=-2)
        axis2_all = tf.expand_dims(axis2_all, axis=-2)
        dist, vec, flag = distance_pt_pt(axis1_all, axis2_all, 0, 0) # N_sim, N_rot*N_axis=3, 1, 3
        dist = tf.reshape(dist, (self.N_sim, self.N_rot, 3, 1)) # N_sim, N_rot, N_axis=3,1
        vec = tf.reshape(vec, (self.N_sim, self.N_rot, 3, 3)) # N_sim, N_rot, N_axis=3, 3
        return dist, vec, self.mask_rot
    
    def set_rot_axis(self):
        self.N_rot = len(self.pair_all)
        self.obj_axis = np.zeros((self.num_objects, 3, 3), dtype=np.float32) # N_obj, N_axis, 3
        self.axis_list = []
        for i_o, obj_name in zip(range(self.num_objects), self.object_name_list):
            obj = self.object_dict[obj_name]
            self.obj_axis[i_o] = obj.get_axis()
            self.axis_list += [obj.Naxis]
        self.obj_axis_exp = tf.expand_dims(tf.expand_dims(self.obj_axis, 0), -2) # 1,1, N_obj, N_axis, 1, 3
        self.pair_axis_list = []
        for i_c, comb in zip(range(self.N_rot), self.pair_all):
            self.pair_axis_list += np.pad(
                np.expand_dims(np.arange(self.axis_list[comb[0]]), axis=1), 
                [[0,0],[1,0]], constant_values=i_c).tolist()
    
    def set_link_dependence(self, link_dict):
        link_dict_binding = {}
        for i_obj in range(self.num_objects):
            binding_name = self.object_name_list[i_obj]
            object_name = self.object_dict[binding_name].obj_layer.gitem.name
            link_dict_binding[binding_name] = link_dict[object_name]
        super(BindingCalculator, self).set_link_dependence(link_dict_binding)
    
    @tf.function
    def calc_all(self, Tbo_all_res):
        dist_all, flag_all, vec_all, mask_all = super(BindingCalculator, self).calc_all(Tbo_all_res)
        angle_all, vec_angle, mask_rot = self.angle_dist(Tbo_all_res)
        return dist_all, flag_all, vec_all, mask_all, angle_all, vec_angle, mask_rot
        
    def pair_condition(self, obj1, obj2):
        return (obj1.btype in obj2.counterparts and obj2.btype in obj1.counterparts)
    
    def set_pair_mask(self, slack_pair_list):
        self.slack_pair_list = slack_pair_list
        mask_dict = {}
        N_stack_dict = {}
        N_stack = 0
        for pair_case in DistanceCalculator.pair_cases:
            N_col = self.N_pair_dict[pair_case]
            mask_dict[pair_case] = np.zeros((self.N_sim, N_col, 1), dtype=np.float32)
            N_stack_dict[pair_case] = N_stack
            N_stack += N_col

        self.mask_rot = np.zeros((self.N_sim, self.N_rot), dtype=np.float32) # N_sim, N_rot
        self.slack_idx_list = []
        for i_s, pair_vec in zip(range(self.N_sim), slack_pair_list):
            pair_idx_list = []
            for pair in pair_vec:
                i_p = self.pair_all.index(pair) if pair in self.pair_all else self.pair_all.index(tuple(reversed(pair)))
                self.slack_idx_list += [(i_s, i_p)]
                self.mask_rot[i_s, i_p] = 1
                for pair_case in DistanceCalculator.pair_cases:
                    N_stack = N_stack_dict[pair_case]
                    N_col = self.N_pair_dict[pair_case]
                    if N_stack <= i_p < N_stack+N_col:
                        i_c = i_p-N_stack
                        mask_dict[pair_case][i_s, i_c, 0] = 1
        self.mask_rot_rev = 1 - self.mask_rot
        self.mask_rot_diag = np.expand_dims(self.mask_rot, axis=-2)*np.expand_dims(self.mask_rot, axis=-1)
        self.mask_rot_diag_rev = np.identity(self.N_rot, dtype=np.float32) - self.mask_rot_diag

        for k,v in mask_dict.items():
            setattr(self, "mask_"+k, 1-v)
            
    def jacobian_rot(self,Tbo_all, axis_jnt):
        Rbo_all = tf.expand_dims(tf.gather(tf.gather(Tbo_all, [0,1,2], axis=-2), [0,1,2], axis=-1), axis=-3)
                    # N_sim, N_object, (N_axis), 3, 3
        axis_all = K.sum(Rbo_all*self.obj_axis_exp, axis=-1) # N_sim, N_object, N_axis=3, 3
        return self.robot.jacobian_rot(axis_all, self.num_objects, self.object_depend_mask, axis_jnt)
    
    def jacobian_binding(self, jac_o, vec_all, jac_rot, vec_ang):
        jac_d = self.jacobian_distance(jac_o, vec_all)
        jac_ang = self.jacobian_angle(jac_rot, vec_ang)
        return jac_d, jac_ang
    
    def jacobian_angle(self, jac_rot, vec_ang):# (N_sim, N_obj, N_axis, DOF, Dim), (N_sim, N_pair, N_axis, Dim)
        jac_rot1 = tf.gather(jac_rot, self.pair_obj1, axis=-4) # (N_sim, N_pair, N_axis, DOF, Dim)
        jac_rot2 = tf.gather(jac_rot, self.pair_obj2, axis=-4) # (N_sim, N_pair, N_axis, DOF, Dim)
        vec_ang = tf.expand_dims(vec_ang, axis=-2) # (N_sim, N_pair, N_axis, 1, Dim)
        jac_ang = K.sum(vec_ang*(jac_rot2 - jac_rot1), axis=-1) # (N_sim, N_pair, N_axis, DOF)
        return jac_ang