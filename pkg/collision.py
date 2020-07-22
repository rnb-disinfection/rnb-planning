import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras import backend as K
import numpy as np

from .rotation_utils import *
from .utils import *


def get_facepoints(verts, faces, N_fcs=20): # (fc, vtx, ax)
    global_save['verts'] = verts
    global_save['faces'] = faces
    return np.concatenate([np.array([verts[s1f-1] for s1f in faces]), np.zeros((N_fcs-len(faces),3,3))],axis=0)

def pad_vertex(verts, N_vtx):
    l_v = len(verts)
    assert l_v<N_vtx, "vertex more than N_vtx"
    return np.concatenate([verts, np.zeros((N_vtx-l_v,3))],axis=0).astype('float32')


BATCH_DIM_DEFAULT = 2

Imat_line = tf.constant([[0, 1], [0, 2]], dtype=tf.int64)
@tf.function
def pickClosestLine(a,b,c,batch_dims=2):
    ab = b-a
    ao = -a
    ac = c-a

    #Normal to face of triangle
    abc = tf.linalg.cross(ab,ac)
    #Perpendicular to AB going away from triangle
    abp = tf.linalg.cross(ab,abc)
    #Perpendicular to AC going away from triangle
    acp = tf.linalg.cross(abc,ac)
    abp, abp_nm = tf.linalg.normalize(abp, axis=-1)
    acp, abp_nm = tf.linalg.normalize(acp, axis=-1)
    v_candi = tf.stack([abp,acp],axis=-2)
    ao_exp = tf.expand_dims(ao, axis=-2)
    vo_candi = K.sum(v_candi*ao_exp, axis=-1)
    I = K.argmax(vo_candi, axis=-1)
    I_exp = tf.expand_dims(I, axis=-1)
    dist = tf.gather_nd(vo_candi, I_exp, batch_dims=batch_dims)
    abc = tf.stack([a,b,c],axis=-2)
    Ivec = tf.gather(Imat_line, I, axis=0)
    b = tf.gather_nd(abc, Ivec[:,:,0:1],batch_dims=batch_dims)
    c = tf.gather_nd(abc, Ivec[:,:,1:2],batch_dims=batch_dims)
    v = tf.gather_nd(v_candi, I_exp,batch_dims=batch_dims)
    return b,c,v,dist

@tf.function
def getFarthestInDir_batch(FX_batch, v_batch, batch_dims=BATCH_DIM_DEFAULT):
    dotted = K.sum(FX_batch*v_batch,axis=-1)
    IdxSet = tf.expand_dims(tf.argmax(dotted, axis=-1),-1)
    point=tf.gather_nd(params = FX_batch, indices=IdxSet, batch_dims=batch_dims)
    return point

@tf.function
def support_batch(FX1_batch, FX2_batch, v_batch, batch_dims=BATCH_DIM_DEFAULT):
    point1 = getFarthestInDir_batch(FX1_batch, v_batch, batch_dims)
    point2 = getFarthestInDir_batch(FX2_batch, -v_batch, batch_dims)
    return point1 - point2

@tf.function
def pickLineTF_batch(v_batch, FX1_batch, FX2_batch, batch_dims=BATCH_DIM_DEFAULT):
    b= support_batch(FX2_batch, FX1_batch, v_batch, batch_dims)
    a= support_batch(FX2_batch, FX1_batch, -v_batch, batch_dims)
    return a, b

Imat_line = tf.constant([[0, 1], [0, 2]], dtype=tf.int64)
@tf.function
def pickClosestLine_batch(a,b,c,batch_dims=2):
    ab = b-a
    ao = -a
    ac = c-a

    #Normal to face of triangle
    abc = tf.linalg.cross(ab,ac)
    #Perpendicular to AB going away from triangle
    abp = tf.linalg.cross(ab,abc)
    #Perpendicular to AC going away from triangle
    acp = tf.linalg.cross(abc,ac)
    abp, abp_nm = tf.linalg.normalize(abp, axis=-1)
    acp, abp_nm = tf.linalg.normalize(acp, axis=-1)
    v_candi = tf.stack([abp,acp],axis=-2)
    ao_exp = tf.expand_dims(ao, axis=-2)
    vo_candi = K.sum(v_candi*ao_exp, axis=-1)
    I = K.argmax(vo_candi, axis=-1)
    I_exp = tf.expand_dims(I, axis=-1)
    abc = tf.stack([a,b,c],axis=-2)
    Ivec = tf.gather(Imat_line, I, axis=0)
    b = tf.gather_nd(abc, Ivec[:,:,0:1],batch_dims=batch_dims)
    c = tf.gather_nd(abc, Ivec[:,:,1:2],batch_dims=batch_dims)
    v = tf.gather_nd(v_candi, I_exp,batch_dims=batch_dims)
    dist = tf.expand_dims(tf.gather_nd(vo_candi, I_exp, batch_dims=batch_dims), axis=-1)
    v = tf.expand_dims(v, axis=-2)
    return b,c,v,dist

Imat_face = tf.constant([[0, 1, 2], [0, 2, 3], [0, 3, 1]], dtype=tf.int64)
@tf.function
def pickClosestFace_batch(a,b,c,d,batch_dims=2):
    
    # Check the tetrahedron:
    ab = b-a
    ao = -a
    ac = c-a
    ad = d-a

    #We KNOW that the origin is not under the base of the tetrahedron based on
    #the way we picked a. So we need to check faces ABC, ABD, and ACD.

    #Normal to face of triangle
    abc = tf.linalg.cross(ab,ac)
    acd = tf.linalg.cross(ac,ad) # Normal to face of triangle
    adb = tf.linalg.cross(ad,ab) # Normal to face of triangle
    abc, _ = tf.linalg.normalize(abc, axis=-1)
    acd, _ = tf.linalg.normalize(acd, axis=-1)
    adb, _ = tf.linalg.normalize(adb, axis=-1)
    v_candi = tf.stack([abc,acd,adb],axis=-2)
    ao_exp = tf.expand_dims(ao, axis=-2)
    vo_candi = K.sum(v_candi*ao_exp, axis=-1)
    I = K.argmax(vo_candi,axis=-1)
    I_exp = tf.expand_dims(I, axis=-1)
    abcd = tf.stack([a,b,c,d],axis=-2)
    Ivec = tf.gather(Imat_face, I, axis=0)
    a = tf.gather_nd(abcd, Ivec[:,:,0:1],batch_dims=batch_dims)
    b = tf.gather_nd(abcd, Ivec[:,:,1:2],batch_dims=batch_dims)
    c = tf.gather_nd(abcd, Ivec[:,:,2:3],batch_dims=batch_dims)
    v = tf.gather_nd(v_candi, I_exp,batch_dims=batch_dims)
    dist = tf.expand_dims(tf.gather_nd(vo_candi, I_exp, batch_dims=batch_dims), axis=-1)
    v = tf.expand_dims(v, axis=-2)
    return a,b,c,v,dist


Imat_resort = tf.constant([[0,1,2],[1,2,0],[2,0,1]], dtype=tf.int64)
@tf.function
def resort_points_batch(a,b,c,batch_dims=2):
    XX = tf.stack([a,b,c], axis=-2)
    norms = tf.linalg.norm(XX, axis=-1)
    I=K.argmin(norms, axis=-1)
    Imin = tf.expand_dims(tf.gather(Imat_resort, I),axis=-1)
    a = tf.gather_nd(XX,tf.gather(Imin,0,axis=-2), batch_dims=batch_dims)
    b = tf.gather_nd(XX,tf.gather(Imin,1,axis=-2), batch_dims=batch_dims)
    c = tf.gather_nd(XX,tf.gather(Imin,2,axis=-2), batch_dims=batch_dims)
    return a,b,c

@tf.function
def direct(a,b,c,v,dist):
    dist = tf.expand_dims(dist, axis=-1)
    a, b, c = resort_points_batch(a, b, c)
    a_,b_,v_,dist_ = pickClosestLine_batch(a,b,c)
    dist_ = tf.expand_dims(dist_,axis=-1)
    v_dist = tf.multiply(v,dist)
    v_dist_ori = v_dist
    dist_p = tf.cast(tf.greater(dist_,0), tf.float32)
    ab = b_-a_
    ab_nm, _ = tf.linalg.normalize(ab,axis=-1)
    oab = K.sum(a_*ab_nm, axis=-1, keepdims=True)
    oab_p = tf.cast(tf.greater(oab,0), tf.float32)
    v_dist_ = v_*dist_-tf.expand_dims(oab_p*ab_nm*oab,axis=-2)
    v_, dist_ = tf.linalg.normalize(v_dist_,axis=-1)
    v_dist = v_dist + dist_p*v_dist_
    v, dist = tf.linalg.normalize(v_dist,axis=-1)
    dist = tf.reduce_sum(dist, axis=-1)
    return v, dist

@tf.function
def nearest_simplex4_batch(a,b,c,v,dist, FX1_batch,FX2_batch):
    tri_in = tf.greater(K.sum(tf.multiply(v,-tf.expand_dims(a,axis=-2)),axis=-1),0)
    tri_out = tf.logical_not(tri_in)
    tri_in = tf.cast(tri_in, tf.float32)
    tri_out = tf.cast(tri_out, tf.float32)
    d = tri_in*c + tri_out*b
    c = tri_in*b + tri_out*c
    b = tri_in*a + tri_out*a
    v = tf.expand_dims(tri_in,axis=-2)*v + tf.expand_dims(tri_out,axis=-2)*(-v)
    v, dist = direct(b,c,d,v,dist)
    a = support_batch(FX2_batch,FX1_batch,v) # Tetrahedron new point
    return a,b,c,d

@tf.function
def _loop_PickTriangle_batch(a, b, c, flag, FX1_batch, FX2_batch):
    [b_,c_,v,dist] = pickClosestLine_batch(a,b,c);
    a__,b__,c__,flag = (support_batch(FX2_batch, FX1_batch,v, BATCH_DIM_DEFAULT),b_,c_,tf.less_equal(dist,0))
    def_case = tf.cast(flag, tf.float32)
    def_case_not = tf.cast(tf.logical_not(flag), tf.float32)
    c = def_case_not*c__ + def_case*c
    b = def_case_not*b__ + def_case*b
    a = def_case_not*a__ + def_case*a
    
    return a,b,c,flag, FX1_batch, FX2_batch

@tf.function
def _cond_PickTriangle_batch(a, b, c, flag, FX1_batch, FX2_batch):
    return tf.reduce_all(tf.logical_not(flag))
    

@tf.function
def PickTriangleTF_batch(a, b, FX1_batch, FX2_batch, flag_default, IterationAllowed=6):
    # First try:
    ab = b-a
    ao = -a
    v_batch = tf.expand_dims(tf.linalg.cross(tf.linalg.cross(ab,ao),ab), axis=-2) # v is perpendicular to ab pointing in the general direction of the origin
    c = b
    b = a
    a = support_batch(FX2_batch,FX1_batch,v_batch,BATCH_DIM_DEFAULT)
    a,b,c,flag, _, _ = tf.while_loop(
        _cond_PickTriangle_batch, _loop_PickTriangle_batch, (a,b,c,flag_default, FX1_batch, FX2_batch), 
        parallel_iterations=10, maximum_iterations=IterationAllowed
    )

    return a, b, c, flag

@tf.function
def _loop_pickTetrahedron_batch(a, b, c, d, v, dist, flag, FX1_batch, FX2_batch, iter):
    a,b,c,v_,dist_ = pickClosestFace_batch(a,b,c,d)
    a_,b_,c_,d_ = nearest_simplex4_batch(a,b,c,v_, dist_, FX1_batch,FX2_batch)
    flag = tf.less_equal(dist_,0)
    def_case = tf.cast(flag, tf.float32)
    def_case_not = tf.cast(tf.logical_not(flag), tf.float32)
    def_case_exp = tf.expand_dims(def_case,axis=-1)
    def_case_not_exp = tf.expand_dims(def_case_not,axis=-1)
    d = def_case_not*d_ + def_case*d
    c = def_case_not*c_ + def_case*c
    b = def_case_not*b_ + def_case*b
    a = def_case_not*a_ + def_case*a
#     v = def_case_not_exp*v_ + def_case_exp*v
#     dist = def_case_not*dist_ + def_case*dist
#     d = d_
#     c = c_
#     b = b_
#     a = a_
    v = v_
    dist = dist_
    iter += 1
    return a, b, c, d, v, dist, flag, FX1_batch, FX2_batch, iter

@tf.function
def _cond_pickTetrahedron_batch(a, b, c, d, v, dist, flag, FX1_batch, FX2_batch, iter):
    return tf.reduce_any(tf.logical_not(flag))

@tf.function
def pickTetrahedronTF_batch(a,b,c,FX1_batch,FX2_batch,flag_default,dist_default,IterationAllowed=6):
    ab = b-a
    ac = c-a

    # Normal to face of triangle
    abc = tf.linalg.cross(ab,ac)
    v, abc_nm = tf.linalg.normalize(abc,axis=-1)
    dist = K.sum(v*a,axis=-1, keepdims=True)
    v = tf.expand_dims(v, axis=-2)

    a,b,c,d = nearest_simplex4_batch(a,b,c,v,dist, FX1_batch,FX2_batch)


    a, b, c, d, v, dist, flag, _, _, iter = tf.while_loop(
        _cond_pickTetrahedron_batch, _loop_pickTetrahedron_batch, 
        (a,b,c,d,v,dist_default,flag_default, FX1_batch, FX2_batch, 0), 
        parallel_iterations=10, maximum_iterations=IterationAllowed
    )
    v_, dist_ = direct(b,c,d,v,dist)
    dist = dist_
#     signdist = tf.sign(dist) # erroneous
#     dist = signdist*dist_ # erroneous
#     v_ = tf.expand_dims(signdist,axis=-1)*v_ # erroneous

#     a = support_batch(FX2_batch,FX1_batch,v_) # Tetrahedron new point
#     v_rs = tf.reduce_sum(v, axis=-2)
#     dist3_ = K.sum(-a*v_rs, axis=-1)
#     dotted = tf.reduce_sum(K.sum(v_*v, axis=-1), axis=-1)
#     dist = dist3_/dotted
    return a,b,c,d, v, v_, dist,flag, iter


@tf.function
def test_collision_batch(FX1_batch, FX2_batch, v_batch,flag_default,dist_default, IterationAllowed=6):
    a, b = pickLineTF_batch(v_batch, FX2_batch, FX1_batch)
    a, b, c, flag = PickTriangleTF_batch(a,b,FX2_batch,FX1_batch,flag_default,IterationAllowed)
    a,b,c,d,v, v_, dist,flag, iter = pickTetrahedronTF_batch(a,b,c,FX2_batch,FX1_batch,flag_default,dist_default,IterationAllowed)
#     v = v_batch
#     v_ = v_batch
    v = tf.stop_gradient(v)
    v_ = tf.stop_gradient(v_)
    a = support_batch(FX1_batch,FX2_batch,v_) # Tetrahedron new point
    a = tf.expand_dims(a, axis=2)
    dist3_ = K.sum(-a*v, axis=-1)
    dotted = K.sum(v_*v, axis=-1)
    dist = dist3_/dotted
    return dist, flag

def get_flag_default(N_sim, N_col, val=False):
    return np.reshape(np.array([[[val]]*N_col]*N_sim), (N_sim, N_col, 1))

def get_dist_default(N_sim,N_col,  val=0.0):
    return np.reshape(np.array([[[val]]*N_col]*N_sim, dtype=np.float32), (N_sim, N_col, 1))

def get_xy_batch(N_sim, N_col):
    return (
        np.reshape(np.array([[[[1,0,0]]]*N_col]*N_sim, dtype=np.float32), (N_sim, N_col, 1, 3)), 
        np.reshape(np.array([[[[0,1,0]]]*N_col]*N_sim, dtype=np.float32), (N_sim, N_col, 1, 3))
    )














# @tf.function
# def getFarthestInDir(FX, v):
#     dotted = K.sum(tf.multiply(FX,v),axis=-1)
#     IdxSet = tf.expand_dims(tf.argmax(dotted, axis=-1),axis=-1)
#     point=tf.gather(params = FX, indices=IdxSet)
#     return point

# @tf.function
# def support(FX1, FX2, v):
#     point1 = getFarthestInDir(FX1, v)
#     point2 = getFarthestInDir(FX2, -v)
#     return point1 - point2

# @tf.function
# def pickLineTF(v, FX1, FX2):
#     b= support(FX2, FX1, v)
#     a= support(FX2, FX1, -v)
#     return a, b

# Imat_line = tf.constant([[0, 1], [0, 2]], dtype=tf.int64)
# @tf.function
# def pickClosestLine(a,b,c):
#     ab = b-a
#     ao = -a
#     ac = c-a

#     #Normal to face of triangle
#     abc = tf.linalg.cross(ab,ac)
#     #Perpendicular to AB going away from triangle
#     abp = tf.linalg.cross(ab,abc)
#     #Perpendicular to AC going away from triangle
#     acp = tf.linalg.cross(abc,ac)
#     abp, abp_nm = tf.linalg.normalize(abp, axis=-1)
#     acp, abp_nm = tf.linalg.normalize(acp, axis=-1)
#     v_candi = tf.stack([abp,acp],axis=-2)
#     ao_exp = tf.expand_dims(ao, axis=-2)
#     vo_candi = K.sum(v_candi*ao_exp, axis=-1)
#     I = K.argmax(vo_candi, axis=-1)
#     I_exp = tf.expand_dims(I, axis=-1)
#     dist = tf.gather_nd(vo_candi, I_exp)
#     abc = tf.stack([a,b,c],axis=-2)
#     Ivec = tf.gather(Imat_line, I, axis=0)
#     b = tf.gather_nd(abc, Ivec[0:1])
#     c = tf.gather_nd(abc, Ivec[1:2])
#     v = tf.gather_nd(v_candi, I_exp)
#     v = tf.expand_dims(v, axis=-2)
#     return b,c,v,dist

# @tf.function
# def _loop_PickTriangle(a, b, c, flag, FX1, FX2):
#     [b_,c_,v,dist] = pickClosestLine(a,b,c,batch_dims=0);
#     a,b,c,flag = (support(FX2, FX1,v),b_,c_,tf.less_equal(dist,0))  
# #     ab = b-a;
# #     ao = -a;
# #     ac = c-a;
# #     abc = tf.linalg.cross(ab,ac)
# #     abp = tf.linalg.cross(ab,abc)
# #     acp = tf.linalg.cross(abc,ac)
# #     abpo = tf.greater(K.sum(abp*ao), 0)
# #     acpo = tf.greater(K.sum(acp*ao), 0)
# #     a,b,c,flag = tf.case(
# #         [
# #             (abpo, lambda: (support(FX2, FX1,abp),a,b,False)),
# #             (acpo, lambda: (support(FX2, FX1,acp),a,c,False))
# #         ], default=lambda: (a,b,c,True))
#     return a,b,c,flag, FX1, FX2

# @tf.function
# def _cond_PickTriangle(a, b, c, flag, FX1, FX2):
#     return tf.logical_not(flag)
    

# @tf.function
# def PickTriangleTF(a, b, FX1, FX2, IterationAllowed=6):
#     flag = False

#     # First try:
#     ab = b-a
#     ao = -a
#     v = tf.linalg.cross(tf.linalg.cross(ab,ao),ab) # v is perpendicular to ab pointing in the general direction of the origin
#     c = b
#     b = a
#     a = support(FX2,FX1,v)
#     a,b,c,flag, _, _ = tf.while_loop(
#         _cond_PickTriangle, _loop_PickTriangle, (a,b,c,flag, FX1, FX2), parallel_iterations=10, maximum_iterations=IterationAllowed
#     )

#     return a, b, c, flag

# @tf.function
# def _loop_pickTetrahedron(a, b, c, d, dist, flag, FX1, FX2):
#     #Check the tetrahedron:
#     ab = b-a
#     ao = -a
#     ac = c-a
#     ad = d-a

#     #We KNOW that the origin is not under the base of the tetrahedron based on
#     #the way we picked a. So we need to check faces ABC, ABD, and ACD.

#     #Normal to face of triangle
#     abc = tf.linalg.cross(ab,ac)
#     acd = tf.linalg.cross(ac,ad)
#     adb = tf.linalg.cross(ad,ab)
#     abco = K.sum(abc*ao)
#     acdo = K.sum(acd*ao)
#     adbo = K.sum(adb*ao)

#     b,c,abc,flag = tf.cond(
#         tf.greater(abco, 0), 
#         lambda: (b,c,abc,flag), 
#         lambda: tf.cond(
#             tf.greater(acdo, 0),
#             lambda: (c,d,acd,flag),
#             lambda: tf.cond(
#                 tf.greater(adbo, 0),
#                 lambda: (d,b,adb,flag),
#                 lambda: (b,c,abc,True)
#             )
#         )
#     )

#     a,b,c,d = tf.cond(tf.greater(K.sum(abc*ao), 0), 
#                         lambda: (support(FX2,FX1,abc),a,b,c), 
#                         lambda: (support(FX2,FX1,-abc),a,c,b)
#                        )
#     return a, b, c, d, K.max([acdo, adbo]), flag, FX1, FX2

# @tf.function
# def _cond_pickTetrahedron(a, b, c, d, dist, flag, FX1, FX2):
#     return tf.logical_not(flag)

# @tf.function
# def pickTetrahedronTF(a,b,c,FX1,FX2,IterationAllowed):
#     flag = False
#     ab = b-a
#     ac = c-a

#     # Normal to face of triangle
#     abc = tf.linalg.cross(ab,ac)
#     ao = -a

#     a,b,c,d = tf.cond(tf.greater(K.sum(abc* ao), 0), 
#                         lambda: (support(FX2,FX1,abc),a,b,c), 
#                         lambda: (support(FX2,FX1,-abc),a,c,b)
#                        )

#     a, b, c, d, dist, flag, _, _ = tf.while_loop(
#         _cond_pickTetrahedron, _loop_pickTetrahedron, (a,b,c,d,0.0,flag, FX1, FX2), 
#         parallel_iterations=10, maximum_iterations=IterationAllowed
#     )   
#     return a,b,c,d,dist,flag

# @tf.function
# def test_collision(FX1, FX2, v, iterations):
#     a, b = pickLineTF(v, FX2, FX1)
#     a, b, c, flag = PickTriangleTF(a,b,FX2,FX1,iterations)
#     a,b,c,d,dist,flag = tf.cond(flag, # Only bother if we could find a viable triangle.
#                            lambda: pickTetrahedronTF(a,b,c,FX2,FX1,iterations),
#                            lambda: (a,b,c,c,0.0,flag))
#     return dist, flag

