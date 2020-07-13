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


@tf.function
def getFarthestInDir(FX, v):
    dotted = K.sum(tf.multiply(FX,v),axis=-1)
    IdxSet = tf.expand_dims(tf.argmax(dotted, axis=-1),axis=-1)
    point=tf.gather(params = FX, indices=IdxSet)
    return point

@tf.function
def support(FX1, FX2, v):
    point1 = getFarthestInDir(FX1, v)
    point2 = getFarthestInDir(FX2, -v)
    return point1 - point2

@tf.function
def pickLineTF(v, FX1, FX2):
    b= support(FX2, FX1, v)
    a= support(FX2, FX1, -v)
    return a, b

@tf.function
def _loop_PickTriangle(a, b, c, flag, FX1, FX2):
    ab = b-a;
    ao = -a;
    ac = c-a;
    abc = tf.linalg.cross(ab,ac)
    abp = tf.linalg.cross(ab,abc)
    acp = tf.linalg.cross(abc,ac)
    abpo = tf.greater(K.sum(abp*ao), 0)
    acpo = tf.greater(K.sum(acp*ao), 0)
    a,b,c,flag = tf.case(
        [
            (abpo, lambda: (support(FX2, FX1,abp),a,b,False)),
            (acpo, lambda: (support(FX2, FX1,acp),a,c,False))
        ], default=lambda: (a,b,c,True))
    return a,b,c,flag, FX1, FX2

@tf.function
def _cond_PickTriangle(a, b, c, flag, FX1, FX2):
    return tf.logical_not(flag)
    

@tf.function
def PickTriangleTF(a, b, FX1, FX2, IterationAllowed=6):
    flag = False

    # First try:
    ab = b-a
    ao = -a
    v = tf.linalg.cross(tf.linalg.cross(ab,ao),ab) # v is perpendicular to ab pointing in the general direction of the origin
    c = b
    b = a
    a = support(FX2,FX1,v)
    a,b,c,flag, _, _ = tf.while_loop(
        _cond_PickTriangle, _loop_PickTriangle, (a,b,c,flag, FX1, FX2), parallel_iterations=10, maximum_iterations=IterationAllowed
    )

    return a, b, c, flag

@tf.function
def _loop_pickTetrahedron(a, b, c, d, dist, flag, FX1, FX2):
    #Check the tetrahedron:
    ab = b-a
    ao = -a
    ac = c-a
    ad = d-a

    #We KNOW that the origin is not under the base of the tetrahedron based on
    #the way we picked a. So we need to check faces ABC, ABD, and ACD.

    #Normal to face of triangle
    abc = tf.linalg.cross(ab,ac)
    acd = tf.linalg.cross(ac,ad)
    adb = tf.linalg.cross(ad,ab)
    abco = K.sum(abc*ao)
    acdo = K.sum(acd*ao)
    adbo = K.sum(adb*ao)

    b,c,abc,flag = tf.cond(
        tf.greater(abco, 0), 
        lambda: (b,c,abc,flag), 
        lambda: tf.cond(
            tf.greater(acdo, 0),
            lambda: (c,d,acd,flag),
            lambda: tf.cond(
                tf.greater(adbo, 0),
                lambda: (d,b,adb,flag),
                lambda: (b,c,abc,True)
            )
        )
    )

    a,b,c,d = tf.cond(tf.greater(K.sum(abc*ao), 0), 
                        lambda: (support(FX2,FX1,abc),a,b,c), 
                        lambda: (support(FX2,FX1,-abc),a,c,b)
                       )
    return a, b, c, d, K.max([acdo, adbo]), flag, FX1, FX2

@tf.function
def _cond_pickTetrahedron(a, b, c, d, dist, flag, FX1, FX2):
    return tf.logical_not(flag)

@tf.function
def pickTetrahedronTF(a,b,c,FX1,FX2,IterationAllowed):
    flag = False
    ab = b-a
    ac = c-a

    # Normal to face of triangle
    abc = tf.linalg.cross(ab,ac)
    ao = -a

    a,b,c,d = tf.cond(tf.greater(K.sum(abc* ao), 0), 
                        lambda: (support(FX2,FX1,abc),a,b,c), 
                        lambda: (support(FX2,FX1,-abc),a,c,b)
                       )

    a, b, c, d, dist, flag, _, _ = tf.while_loop(
        _cond_pickTetrahedron, _loop_pickTetrahedron, (a,b,c,d,0.0,flag, FX1, FX2), 
        parallel_iterations=10, maximum_iterations=IterationAllowed
    )   
    return a,b,c,d,dist,flag

@tf.function
def test_collision(FX1, FX2, v, iterations):
    a, b = pickLineTF(v, FX2, FX1)
    a, b, c, flag = PickTriangleTF(a,b,FX2,FX1,iterations)
    a,b,c,d,dist,flag = tf.cond(flag, # Only bother if we could find a viable triangle.
                           lambda: pickTetrahedronTF(a,b,c,FX2,FX1,iterations),
                           lambda: (a,b,c,c,0.0,flag))
    return dist, flag


BATCH_DIM_DEFAULT = 2

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

@tf.function
def _loop_PickTriangle_batch(a, b, c, flag, FX1_batch, FX2_batch):
    ab = b-a;
    ao = -a;
    ac = c-a;
    abc = tf.linalg.cross(ab,ac)
    abp = tf.linalg.cross(ab,abc)
    acp = tf.linalg.cross(abc,ac)
    abpo = tf.greater(K.sum(abp*ao,axis=-1, keepdims=True), 0)
    acpo = tf.greater(K.sum(acp*ao,axis=-1, keepdims=True), 0)
    abpo = tf.logical_and(tf.logical_not(flag),abpo)
    acpo = tf.logical_and(tf.logical_not(tf.logical_or(flag, abpo)),acpo)
    def_case = tf.logical_not(tf.logical_or(abpo, acpo))
    flag = def_case
    abpo = tf.cast(abpo, tf.float32)
    acpo = tf.cast(acpo, tf.float32)
    def_case = tf.cast(def_case, tf.float32)
    c = abpo*b + acpo*c + def_case*c
    b = abpo*a + acpo*a + def_case*b
    v = tf.expand_dims(abpo*abp + acpo*acp + def_case*abc, axis=-2)
    spt = support_batch(FX2_batch, FX1_batch, v, BATCH_DIM_DEFAULT)
#     print(np.max(abpo+acpo+def_case))
#     print(np.min(abpo+acpo+def_case))
    a = (abpo+acpo) * spt + def_case*a
    return a,b,c,flag, FX1_batch, FX2_batch

@tf.function
def _cond_PickTriangle_batch(a, b, c, flag, FX1_batch, FX2_batch):
    return tf.reduce_all(tf.logical_not(flag))
    

@tf.function
def PickTriangleTF_batch(a, b, FX1_batch, FX2_batch, flag_default, IterationAllowed=6):
    flag = False

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
def _loop_pickTetrahedron_batch(a, b, c, d, dist, flag, FX1_batch, FX2_batch, iter):
    #Check the tetrahedron:
    ab = b-a
    ao = -a
    ac = c-a
    ad = d-a

    #We KNOW that the origin is not under the base of the tetrahedron based on
    #the way we picked a. So we need to check faces ABC, ABD, and ACD.

    #Normal to face of triangle
    abc = tf.linalg.cross(ab,ac)
    acd = tf.linalg.cross(ac,ad)
    adb = tf.linalg.cross(ad,ab)
    abco_val = K.sum(abc*ao, axis=-1, keepdims=True)
    acdo_val = K.sum(acd*ao, axis=-1, keepdims=True)
    adbo_val = K.sum(adb*ao, axis=-1, keepdims=True)
    dist = K.max([abco_val, acdo_val, adbo_val],axis=0)
#     dist = tf.math.reduce_mean([abco_val, acdo_val, adbo_val],axis=0)
#     dist = tf.cast(K.argmax([abco_val, acdo_val, adbo_val],axis=0),dtype=tf.float32)
    abco = tf.greater(abco_val, 0)
    acdo = tf.greater(acdo_val, 0)
    adbo = tf.greater(adbo_val, 0)
    abco = tf.logical_and(tf.logical_not(flag),abco)
    flag_abco = tf.logical_or(flag,abco)
    acdo = tf.logical_and(tf.logical_not(flag_abco),acdo)
    flag_abco_acdo = tf.logical_or(flag_abco,acdo)
    adbo = tf.logical_and(tf.logical_not(flag_abco_acdo),adbo)
    def_case = tf.logical_not(tf.logical_or(tf.logical_or(abco, acdo), adbo))
    flag = def_case
    abco = tf.cast(abco, tf.float32)
    acdo = tf.cast(acdo, tf.float32)
    adbo = tf.cast(adbo, tf.float32)
    def_case = tf.cast(def_case, tf.float32)
    
    
    b,c = (abco*b+acdo*c+adbo*d+def_case*b, abco*c+acdo*d+adbo*b+def_case*c)
    abc = abco*abc+acdo*acd+adbo*adb+def_case*abc

    abco = tf.greater(K.sum(abc*ao, axis=-1, keepdims=True), 0)
    abco = tf.cast(abco, tf.float32)
    abco_not = 1-abco
    def_case_not = 1-def_case
    d = def_case_not*(abco*c + abco_not*b) + def_case*d
    c = def_case_not*(abco*b + abco_not*c) + def_case*c
    b = def_case_not*a + def_case*b
    a = def_case_not*support_batch(FX2_batch,FX1_batch,tf.expand_dims(abco*abc+ abco_not*(-abc),axis=-2)) + \
        def_case*a
    
#     abco = tf.greater(K.sum(abc*ao, axis=-1, keepdims=True), 0)
#     abco = tf.cast(abco, tf.float32)
#     abco_not = 1-abco
#     d = abco*c + abco_not*b
#     c = abco*b + abco_not*c
#     b = a
#     a = support_batch(FX2_batch,FX1_batch,tf.expand_dims(abco*abc+ abco_not*(-abc),axis=-2))
    iter += 1
    return a, b, c, d, dist, flag, FX1_batch, FX2_batch, iter

@tf.function
def _cond_pickTetrahedron_batch(a, b, c, d, dist, flag, FX1_batch, FX2_batch, iter):
    return tf.reduce_any(tf.logical_not(flag))

@tf.function
def pickTetrahedronTF_batch(a,b,c,FX1_batch,FX2_batch,flag_default,dist_default,IterationAllowed=6):
    flag = False
    ab = b-a
    ac = c-a

    # Normal to face of triangle
    abc = tf.linalg.cross(ab,ac)
    ao = -a
    
    abco = tf.cast(tf.greater(K.sum(abc* ao,axis=-1,keepdims=True), 0), tf.float32)
    abco_not = 1-abco
    v = tf.expand_dims(abco*abc - abco_not*abc,axis=-2)
    
    d = abco*c + abco_not*b
    c = abco*b + abco_not*c
    b = a
    a = support_batch(FX2_batch,FX1_batch,v)

    a, b, c, d, dist, flag, _, _, iter = tf.while_loop(
        _cond_pickTetrahedron_batch, _loop_pickTetrahedron_batch, 
        (a,b,c,d,dist_default,flag_default, FX1_batch, FX2_batch, 0), 
        parallel_iterations=10, maximum_iterations=IterationAllowed
    )   
    return a,b,c,d,dist,flag, iter


@tf.function
def test_collision_batch(FX1_batch, FX2_batch, v_batch,flag_default,dist_default, iterations=6):
    a, b = pickLineTF_batch(v_batch, FX2_batch, FX1_batch)
    a, b, c, flag = PickTriangleTF_batch(a,b,FX2_batch,FX1_batch,flag_default,iterations)
    a,b,c,d,dist,flag, iter = pickTetrahedronTF_batch(a,b,c,FX2_batch,FX1_batch,flag_default,dist_default,iterations)
    return dist, flag

def get_flag_default(N_sim, N_col, val=False):
    return np.array([[[val]]*N_col]*N_sim)

def get_dist_default(N_sim,N_col,  val=0.0):
    return np.array([[[val]]*N_col]*N_sim, dtype=np.float32)