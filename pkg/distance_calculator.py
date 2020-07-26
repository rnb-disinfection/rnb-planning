from .geometry import *
from .collision import *
from .distance import *
from collections import defaultdict


class ObjectLayer(layers.Layer):
    def __init__(self, gitem, N_sim, N_link, *args, **kwargs):
        self.gitem, self.N_sim, self.N_link = gitem, N_sim, N_link
        self.Toff_list = tf.Variable(np.zeros((N_sim, 4,4), dtype=np.float32), trainable=False) # (N_sim, 4,4)
        self.link_one_hot = tf.Variable(np.zeros((self.N_sim,self.N_link,1,1), dtype=np.float32), trainable=False) # (N_sim, N_link)
        super(ObjectLayer, self).__init__(*args, **kwargs)
        
    def set_frame(self, Toff_list, link_idx_list):
        self.link_idx_list = link_idx_list
        self.Toff_list.assign(tf.constant(Toff_list)) # (N_sim, 4,4)
        self.link_one_hot.assign(tf.reshape(tf.one_hot(link_idx_list, self.N_link), (self.N_sim,self.N_link,1,1))) # (N_sim, N_link)
        
    # 변수를 만듭니다.
    def build(self, input_shape):
        pass
    
    @tf.function
    def get_vertice(self):
        return self.vertice

    # call 메서드가 그래프 모드에서 사용되면
    # training 변수는 텐서가 됩니다.
    @tf.function
    def call(self, input=None):
        T_all = input # (N_sim, N_link, 4,4)
        T_act = K.sum(T_all*self.link_one_hot, axis=1) # (N_sim, 4,4)
        self.T_bo = tf.matmul(T_act, self.Toff_list) # (N_sim, 4,4)
        return self.T_bo


class DistanceCalculator:
    N_vtx_dict = {"pt":1, "ln":2, "pl":4, "bx":8}
    pair_cases = ['pt_pt', 'pt_ln', 'pt_pl', 'pt_bx',
                  'ln_ln', 'ln_pl', 'ln_bx',
                  'pl_pl', 'pl_bx','bx_bx']
    
    def __init__(self, object_name_list, object_dict, robot, N_sim):
        self.object_name_list, self.object_dict = object_name_list, object_dict
        self.object_link_idx_list = []
        for _ in object_name_list:
            self.object_link_idx_list += [0]
        self.robot = robot
        self.N_sim = N_sim
        self.build_combinations()
        self.set_pair_vtx()
        self.prepare_collision_dat()
        
    def pair_condition(self, obj1, obj2):
        return True
    
    def gtype_name(self, obj):
        if obj.gitem.gtype == GeoType.SPHERE:
            return 'pt'
        elif obj.gitem.gtype == GeoType.CYLINDER or obj.gitem.gtype == GeoType.LINE:
            return 'ln'
        elif obj.gitem.gtype == GeoType.PLANE:
            return 'pl'
        elif obj.gitem.gtype == GeoType.BOX:
            return 'bx'

    def build_combinations(self):
        self.num_objects = len(self.object_name_list)
        obj_idx_list = np.arange(self.num_objects)
        pair_combinations = list(combinations(obj_idx_list,2))
        
        self.pair_dict = defaultdict(lambda:[])
        for comb in pair_combinations:
            obj1 = self.object_dict[self.object_name_list[comb[0]]]
            obj2 = self.object_dict[self.object_name_list[comb[1]]]
            if not self.pair_condition(obj1, obj2): # pass if not colliding object 
                continue
            type1 = self.gtype_name(obj1)
            type2 = self.gtype_name(obj2)
            if DistanceCalculator.N_vtx_dict[type1] > DistanceCalculator.N_vtx_dict[type2]:
                pair_name = "{}_{}".format(type2, type1)
                comb = tuple(reversed(comb))
            else:
                pair_name = "{}_{}".format(type1, type2)
            self.pair_dict[pair_name] += [comb]
    
    def set_pair_vtx(self):
        self.vtx1_dict = {}
        self.vtx2_dict = {}
        self.dist1_dict = {}
        self.dist2_dict = {}
        self.N_pair_dict = {}
        for pair_case in DistanceCalculator.pair_cases:
            self.vtx1_dict[pair_case], self.vtx2_dict[pair_case], \
                self.dist1_dict[pair_case], self.dist2_dict[pair_case], self.N_pair_dict[pair_case] = \
                    self.get_vtx_pair( self.pair_dict[pair_case], 
                                      DistanceCalculator.N_vtx_dict[pair_case[:2]], 
                                      DistanceCalculator.N_vtx_dict[pair_case[3:]]
                                     )
            
    def prepare_collision_dat(self):
        pair_all = []
        self.dist_fun_list = []
        for pair_case in DistanceCalculator.pair_cases:
            N_col = self.N_pair_dict[pair_case]
            if N_col>0:
                self.dist_fun_list += [getattr(self, pair_case+"_dist")]
                setattr(self, "vtx1_"+pair_case, self.vtx1_dict[pair_case])
                setattr(self, "vtx2_"+pair_case, self.vtx2_dict[pair_case])
                setattr(self, "dist1_"+pair_case, self.dist1_dict[pair_case])
                setattr(self, "dist2_"+pair_case, self.dist2_dict[pair_case])
                setattr(self, "zeros_"+pair_case, tf.constant(get_zero_points(self.N_sim, N_col), dtype=tf.float32))
                setattr(self, "N_"+pair_case, N_col)
                setattr(self, "flag_default_"+pair_case, 
                        tf.constant(get_flag_default(self.N_sim, N_col), dtype=tf.bool))
                setattr(self, "dist_default_"+pair_case, 
                        tf.constant(get_dist_default(self.N_sim, N_col), dtype=tf.float32))
                x_batch, y_batch = get_xy_batch(self.N_sim, N_col)
                setattr(self, "x_batch_"+pair_case, 
                        tf.constant(x_batch, dtype=tf.float32))
                setattr(self, "y_batch_"+pair_case, 
                        tf.constant(y_batch, dtype=tf.float32))

                pair_list = self.pair_dict[pair_case]
                pair_all += pair_list
        self.pair_all = pair_all
        pair_all = np.array(pair_all)
        self.pair_obj1 = pair_all[:,0]
        self.pair_obj2 = pair_all[:,1]
            
    def get_vtx_pair(self, pair_list, N_vtx1, N_vtx2):
        N_col = len(pair_list)
        pair_vtx1 = np.zeros((1, len(pair_list), self.num_objects, N_vtx1, 1, 4)) # (N_sim, N_col, N_obj, N_vtx, 1, 4)
        pair_vtx2 = np.zeros((1, len(pair_list), self.num_objects, N_vtx2, 1, 4))
        pair_dist1 = np.zeros((1, len(pair_list), 1))
        pair_dist2 = np.zeros((1, len(pair_list), 1))
        for i_c, comb in zip(range(len(pair_list)), pair_list):
            obj1 = self.object_dict[self.object_name_list[comb[0]]]
            obj2 = self.object_dict[self.object_name_list[comb[1]]]
            pair_vtx1[0,i_c,comb[0],:,0] = np.pad(obj1.gitem.get_vertice(),((0,0),(0,1)),'constant', constant_values=(1))
            pair_vtx2[0,i_c,comb[1],:,0] = np.pad(obj2.gitem.get_vertice(),((0,0),(0,1)),'constant', constant_values=(1))
            pair_dist1[0,i_c, 0] = obj1.gitem.get_radius()
            pair_dist2[0,i_c, 0] = obj2.gitem.get_radius()
        pair_vtx1 = tf.constant(pair_vtx1, dtype=tf.float32)
        pair_vtx2 = tf.constant(pair_vtx2, dtype=tf.float32)
        pair_dist1 = tf.constant(pair_dist1, dtype=tf.float32)
        pair_dist2 = tf.constant(pair_dist2, dtype=tf.float32)
        return pair_vtx1, pair_vtx2, pair_dist1, pair_dist2, N_col
    
    def get_object_index(self, name):
        return self.object_name_list.index(name)
    
    def set_link_dependence(self, link_dict):
        for i_obj in range(self.num_objects):
            object_name = self.object_name_list[i_obj]
            self.object_link_idx_list[i_obj] = np.array(link_dict[object_name])
        self.object_link_idx_mat = np.transpose(self.object_link_idx_list)
        self.object_depend_mask = tf.reshape(
            tf.gather(self.robot.mask_depend, self.object_link_idx_mat, axis=-3), 
            (self.N_sim, self.num_objects, self.robot.DOF, 1))
    
    @tf.function
    def pt_pt_dist(self, Tbo_all_res):
        Tbo_all_res = tf.tile(Tbo_all_res, [1, self.N_pt_pt, 1,1,1,1])
        vtx1_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx1_pt_pt, axis=-1), axis=-3), [0,1,2], axis=-1)
        vtx2_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx2_pt_pt, axis=-1), axis=-3), [0,1,2], axis=-1)
        dist, vec, flag = distance_pt_pt(vtx1_all, vtx2_all, self.dist1_pt_pt, self.dist2_pt_pt)
        return dist, flag, vec, self.mask_pt_pt

    @tf.function
    def pt_ln_dist(self, Tbo_all_res):
        Tbo_all_res = tf.tile(Tbo_all_res, [1, self.N_pt_ln, 1,1,1,1])
        vtx1_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx1_pt_ln, axis=-1), axis=-3), [0,1,2], axis=-1)
        vtx2_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx2_pt_ln, axis=-1), axis=-3), [0,1,2], axis=-1)
        dist, vec, flag = distance_pt_ln(vtx1_all, vtx2_all, self.dist1_pt_ln, self.dist2_pt_ln)
        return dist, flag, vec, self.mask_pt_ln

    @tf.function
    def pt_pl_dist(self, Tbo_all_res):
        Tbo_all_res = tf.tile(Tbo_all_res, [1, self.N_pt_pl, 1,1,1,1])
        vtx1_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx1_pt_pl, axis=-1), axis=-3), [0,1,2], axis=-1)
        vtx2_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx2_pt_pl, axis=-1), axis=-3), [0,1,2], axis=-1)
        dist, vec, flag = distance_pt_pl(vtx1_all, vtx2_all, self.dist1_pt_pl, self.dist2_pt_pl, self.N_sim, self.N_pt_pl)
        return dist, flag, vec, self.mask_pt_pl

    @tf.function
    def pt_bx_dist(self, Tbo_all_res):
        Tbo_all_res = tf.tile(Tbo_all_res, [1, self.N_pt_bx, 1,1,1,1])
        vtx1_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx1_pt_bx, axis=-1), axis=-3), [0,1,2], axis=-1)
        vtx2_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx2_pt_bx, axis=-1), axis=-3), [0,1,2], axis=-1)
        dist, vec, flag = distance_pt_bx(vtx1_all, vtx2_all, self.dist1_pt_bx, self.dist2_pt_bx, self.N_sim, self.N_pt_bx)
        return dist, flag, vec, self.mask_pt_bx

    @tf.function
    def ln_ln_dist(self, Tbo_all_res):
        Tbo_all_res = tf.tile(Tbo_all_res, [1, self.N_ln_ln, 1,1,1,1])
        vtx1_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx1_ln_ln, axis=-1), axis=-3), [0,1,2], axis=-1)
        vtx2_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx2_ln_ln, axis=-1), axis=-3), [0,1,2], axis=-1)
        dist, vec, flag = distance_ln_ln(vtx1_all, vtx2_all, self.dist1_ln_ln, self.dist2_ln_ln, 
                                    self.N_sim, self.N_ln_ln, self.zeros_ln_ln)
        return dist, flag, vec, self.mask_ln_ln

    @tf.function
    def ln_pl_dist(self, Tbo_all_res):
        Tbo_all_res = tf.tile(Tbo_all_res, [1, self.N_ln_pl, 1,1,1,1])
        vtx1_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx1_ln_pl, axis=-1), axis=-3), [0,1,2], axis=-1)
        vtx2_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx2_ln_pl, axis=-1), axis=-3), [0,1,2], axis=-1)
        dist, vec, flag = distance_ln_pl(vtx1_all, vtx2_all, self.dist1_ln_pl, self.dist2_ln_pl, 
                                    self.N_sim, self.N_ln_pl, self.zeros_ln_pl)
        return dist, flag, vec, self.mask_ln_pl

    @tf.function
    def ln_bx_dist(self, Tbo_all_res):
        Tbo_all_res = tf.tile(Tbo_all_res, [1, self.N_ln_bx, 1,1,1,1])
        vtx1_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx1_ln_bx, axis=-1), axis=-3), [0,1,2], axis=-1)
        vtx2_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx2_ln_bx, axis=-1), axis=-3), [0,1,2], axis=-1)
        dist, vec, flag = distance_ln_bx(vtx1_all, vtx2_all, self.dist1_ln_bx, self.dist2_ln_bx, 
                                    self.N_sim, self.N_ln_bx, self.zeros_ln_bx
                                   )
#        dist, vec, flag = distance_ln_bx(vtx1_all, vtx2_all, self.dist1_ln_bx, self.dist2_ln_bx, 
#                                    self.flag_default_ln_bx, self.dist_default_ln_bx,
#                                    self.x_batch_ln_bx, self.y_batch_ln_bx,
#                                    4
#                                   )
        return dist, flag, vec, self.mask_ln_bx

    @tf.function
    def pl_pl_dist(self, Tbo_all_res):
        Tbo_all_res = tf.tile(Tbo_all_res, [1, self.N_pl_pl, 1,1,1,1])
        vtx1_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx1_pl_pl, axis=-1), axis=-3), [0,1,2], axis=-1)
        vtx2_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx2_pl_pl, axis=-1), axis=-3), [0,1,2], axis=-1)
        dist, vec, flag = distance_pl_pl(vtx1_all, vtx2_all, self.dist1_pl_pl, self.dist2_pl_pl, 
                                    self.flag_default_pl_pl, self.dist_default_pl_pl,
                                    self.x_batch_pl_pl, self.y_batch_pl_pl,
                                    4
                                   )
        return dist, flag, vec, self.mask_pl_pl

    @tf.function
    def pl_bx_dist(self, Tbo_all_res):
        Tbo_all_res = tf.tile(Tbo_all_res, [1, self.N_pl_bx, 1,1,1,1])
        vtx1_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx1_pl_bx, axis=-1), axis=-3), [0,1,2], axis=-1)
        vtx2_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx2_pl_bx, axis=-1), axis=-3), [0,1,2], axis=-1)
        dist, vec, flag = distance_pl_bx(vtx1_all, vtx2_all, self.dist1_pl_bx, self.dist2_pl_bx, 
                                    self.flag_default_pl_bx, self.dist_default_pl_bx,
                                    self.x_batch_pl_bx, self.y_batch_pl_bx,
                                    4
                                   )
        return dist, flag, vec, self.mask_pl_bx

    @tf.function
    def bx_bx_dist(self, Tbo_all_res):
        Tbo_all_res = tf.tile(Tbo_all_res, [1, self.N_bx_bx, 1,1,1,1])
        vtx1_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx1_bx_bx, axis=-1), axis=-3), [0,1,2], axis=-1)
        vtx2_all = tf.gather(K.sum(K.sum(Tbo_all_res*self.vtx2_bx_bx, axis=-1), axis=-3), [0,1,2], axis=-1)
        dist, vec, flag = distance_bx_bx(vtx1_all, vtx2_all, self.dist1_bx_bx, self.dist2_bx_bx, 
                                    self.flag_default_bx_bx, self.dist_default_bx_bx,
                                    self.x_batch_bx_bx, self.y_batch_bx_bx,
                                    4
                                   )
        return dist, flag, vec, self.mask_bx_bx
    
    @tf.function
    def calc_all(self, Tbo_all_res):
        dist_all = []
        flag_all = []
        vec_all = []
        mask_all = []
        for dist_fun in self.dist_fun_list:
            dist_, flag_, vec_, mask_ = dist_fun(Tbo_all_res)
            dist_all += [dist_]
            flag_all += [flag_]
            vec_all += [vec_]
            mask_all += [mask_]

        dist_all = tf.concat(dist_all, axis=-2)
        flag_all = tf.concat(flag_all, axis=-2)
        vec_all = tf.concat(vec_all, axis=-2)
        mask_all = tf.concat(mask_all, axis=-2)
        
        return dist_all, flag_all, vec_all, mask_all
        
    def jacobian_object(self, Tbo_all, P_jnt, axis_jnt):
        return self.robot.jacobian_object(Tbo_all, self.num_objects, self.object_depend_mask, P_jnt, axis_jnt)
    
    def jacobian_distance(self, jac_o, vec_all):
        jac_op = tf.gather(jac_o, [0,1,2], axis=-1)
        jac_dp1 = tf.gather(jac_op, self.pair_obj1, axis=-3)
        jac_dp2 = tf.gather(jac_op, self.pair_obj2, axis=-3)
        vec_all = tf.expand_dims(vec_all, axis=-2)
        jac_d = K.sum(vec_all*(jac_dp2 - jac_dp1), axis=-1)
        return jac_d
    
    
class CollisionCalculator(DistanceCalculator):    
    def pair_condition(self, obj1, obj2):
        return (obj1.gitem.collision and obj2.gitem.collision)
    
    def set_pair_mask(self):
        for pair_case in DistanceCalculator.pair_cases:
            N_col = self.N_pair_dict[pair_case]
            mask = np.zeros((self.N_sim, N_col, 1), dtype=np.float32)
            for i_col, comb in zip(range(N_col), self.pair_dict[pair_case]):
                mask[:, i_col, 0] = tf.gather_nd(self.robot.adjacency_mat, self.object_link_idx_mat[:, comb])
            setattr(self, "mask_"+pair_case, 1-mask)
            
            