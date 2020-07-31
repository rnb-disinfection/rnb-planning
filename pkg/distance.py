from pkg.collision import *

def distance_mesh(mesh1, mesh2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=6):
    v_batch = K.mean(mesh1, axis=-2, keepdims=True)-K.mean(mesh2, axis=-2, keepdims=True)
    v_batch = tf.linalg.cross(v_batch, x_batch)+tf.linalg.cross(v_batch, y_batch)
    dist_, vec, flag = test_collision_batch(mesh1, mesh2, v_batch, flag_default, dist_default, IterationAllowed=IterationAllowed)
    dist = dist_ - (dist1+dist2)
    flag = tf.less_equal(dist, 0)
    return dist, vec, flag

def distance_pt_pt(pt1, pt2, dist1, dist2): # (N_sim, N_col, 1, 3) / (N_sim, N_col, 1)
    vec, dist_ = tf_normalize(pt2-pt1, axis=-1)
    dist = K.sum(dist_, axis=-1) - (dist1 + dist2)
    vec = K.sum(vec, axis=-2)
    flag = tf.less_equal(dist, 0)
    return dist, vec, flag

def distance_pt_ln(pt1, ln2, dist1, dist2): # (N_sim, N_col, 1, 3), (N_sim, N_col, 2, 3) / (N_sim, N_col, 1)
    dpts = ln2-pt1
    dpt_vec, dpt_nm = tf_normalize(dpts,axis=-1)
    # dpt_nm = K.min(dpt_nm, axis=-1, keepdims=True)
    i_dpt_nm = K.argmin(dpt_nm, axis=-2)
    dpt_nm = tf.gather_nd(dpt_nm, i_dpt_nm, batch_dims=2)
    dpt_vec = tf.gather_nd(dpt_vec, i_dpt_nm, batch_dims=2)

    a = tf.gather(dpts, [0], axis=-2)
    b = tf.gather(dpts, [1], axis=-2)

    ab = b-a
    ba = a-b
    ao = -a
    bo = -b

    inner = tf.logical_and(tf.greater(K.sum(ab*ao, axis=-1),0), tf.greater(K.sum(ba*bo, axis=-1),0))
    outer = tf.logical_not(inner)
    inner = tf.cast(inner, tf.float32)
    outer = tf.cast(outer, tf.float32)

    abo = tf.linalg.cross(tf.linalg.cross(ab,ao), ab)
    abo_v, abo_nm = tf_normalize(abo, axis=-1)

    vec = inner*K.sum(-abo_v, axis=-2) + outer*dpt_vec
    dist_ = inner*K.sum(abo_v*ao, axis=-1) + outer*K.min(dpt_nm, axis=-1, keepdims=True)
    dist = dist_ - (dist1 + dist2)
    flag = tf.less_equal(dist, 0)
    return dist, vec, flag

def distance_pt_pl(pt1, pl2, dist1, dist2, N_sim, N_col): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
    pl2_perm = tf.gather(pl2, [1,2,3,0], axis=-2)
    pl2_stack = tf.stack([pl2, pl2_perm], axis=-2)
    ln2 = tf.reshape(pl2_stack, (N_sim, N_col*4, 2, 3))
    dist_, vec_, flag_ = distance_pt_ln(tf.repeat(pt1, 4, axis=-3), ln2, 0, 0)
    dist_ = tf.reshape(dist_, (N_sim, N_col, 4, 1))
    vec_ = tf.reshape(vec_, (N_sim, N_col, 4, 3))
    i_dist_ = K.argmin(dist_, axis=-2)
    dist_ = tf.gather_nd(dist_, i_dist_, batch_dims=2)
    vec_ = tf.gather_nd(vec_, i_dist_, batch_dims=2)


    po = tf.concat([pt1-pl2, pt1-pl2_perm], axis=-2)
    pl_sides = tf.concat([pl2_perm-pl2, pl2-pl2_perm], axis=-2)
    pl2_perp = tf.linalg.cross(tf.gather(pl_sides, [0], axis=-2), tf.gather(pl_sides, [1], axis=-2))
    pl2_perp, _ = tf_normalize(pl2_perp, axis=-1)
    dist__ = K.sum(pl2_perp * tf.gather(po, [1], axis=-2), axis=-1)
    pl2_perp = -K.sum(pl2_perp, axis=-2)


    inside = K.all(tf.greater(K.sum(pl_sides*po, axis=-1), 0), axis=-1, keepdims=True)
    outside = tf.logical_not(inside)
    inside = tf.cast(inside, tf.float32)
    outside = tf.cast(outside, tf.float32)
    vec = inside*pl2_perp + outside*vec_
    dist_ = inside*dist__ + outside*dist_

    dist = dist_ - (dist1 + dist2)
    flag = tf.less_equal(dist, 0)
    return dist, vec, flag

def distance_pt_bx(pt1, bx2, dist1, dist2, N_sim, N_col): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
# def distance_pt_bx(pt1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
#     dist, flag = distance_mesh(pt1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=6)
#     return dist, flag
    bx2_perm1 = tf.gather(bx2, tf.constant([0,1,2,3, 4,5,6,7, 0,1,2,3]), axis=-2)
    bx2_perm2 = tf.gather(bx2, tf.constant([1,2,3,0, 5,6,7,4, 4,5,6,7]), axis=-2)
    bx2_stack = tf.stack([bx2_perm1, bx2_perm2], axis=-2)
    ln2 = tf.reshape(bx2_stack, (N_sim, N_col*12, 2, 3))
    dist_, vec_, flag_ = distance_pt_ln(tf.repeat(pt1, 12, axis=-3), ln2, 0, 0)
    dist_ = tf.reshape(dist_, (N_sim, N_col, 12))
    vec_ = tf.reshape(vec_, (N_sim, N_col, 12, 3))
    i_dist_ = tf.expand_dims(K.argmin(dist_, axis=-1), axis=-1)
    dist_ = tf.expand_dims(tf.gather_nd(dist_, i_dist_, batch_dims=2), axis=-1)
    vec_ = tf.gather_nd(vec_, i_dist_, batch_dims=2)
    #     dist_ = K.min(tf.reshape(dist_, (N_sim, N_col, 12)), axis=-1, keepdims=True)

    fc2_perm1 = tf.gather(bx2, tf.constant([3,2,1,0, 4,5,6,7, 0,1,5,4, 1,2,6,5, 2,3,7,6, 3,0,4,7]), axis=-2)
    fc2_perm2 = tf.gather(bx2, tf.constant([2,1,0,3, 5,6,7,4, 1,5,4,0, 2,6,5,1, 3,7,6,2, 0,4,7,3]), axis=-2)
    po = pt1-fc2_perm1
    po_res = tf.reshape(po, (N_sim, N_col, 6, 4, 3))
    fc2_sides = fc2_perm2 - fc2_perm1
    fc2_sides_res = tf.reshape(fc2_sides, (N_sim, N_col, 6, 4,3))
    fc2_perp = tf.linalg.cross(tf.gather(fc2_sides_res, [0], axis=-2), tf.gather(fc2_sides_res, [1], axis=-2))
    fc2_perp, _ = tf_normalize(fc2_perp, axis=-1)
    dist__ = tf.abs(K.sum(fc2_perp * tf.gather(po_res, [0], axis=-2), axis=-1))

    ivec = tf.linalg.cross(tf.tile(fc2_perp,[1,1,1,4,1]),fc2_sides_res)
    fco = tf.greater(K.sum(ivec*po_res, axis=-1), 0)
    inside = K.all(fco, axis=-1, keepdims=True)
    oncorner = tf.cast(K.all(tf.logical_not(inside), axis=-2), tf.float32)
    noncorner = 1-oncorner
    inbox = K.all(inside, axis=-2, keepdims=True)
    inside = tf.cast(tf.logical_and(tf.logical_not(inbox), inside), tf.float32)
    inbox = K.sum(tf.cast(inbox, tf.float32), axis=-2)

    side_dist = (1-inside)*1000+dist__
    i_dist__ = K.argmin(side_dist, axis=-2)
    dist_side = tf.gather_nd(side_dist, i_dist__, batch_dims=2)
    vec_side = K.sum(tf.gather_nd(fc2_perp, i_dist__, batch_dims=2), axis=-2)
    i_dist___ = K.argmin(dist__,axis=-2)
    dist_in = tf.gather_nd(dist__, i_dist___, batch_dims=2)
    vec_in = K.sum(tf.gather_nd(fc2_perp, i_dist___, batch_dims=2), axis=-2)
    vec = oncorner*vec_ + noncorner*vec_side - vec_in*inbox
    dist = oncorner*dist_ + noncorner*dist_side - dist_in*inbox
    dist = dist - (dist1+dist2)
    flag = tf.less_equal(dist, 0)
    return dist, vec, flag

def distance_ln_ln(ln1, ln2, dist1, dist2, N_sim, N_col, zeros_pt): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
    pl2 = tf.reshape(tf.expand_dims(ln2, axis=-2)-tf.expand_dims(ln1, axis=-3), (N_sim, N_col, 4, 3))
    pl2 = tf.gather(pl2, [0,1,3,2], axis=-2)
    dist, vec, flag = distance_pt_pl(zeros_pt, pl2, 0, 0, N_sim, N_col)
    dist = tf.abs(dist) - (dist1+dist2)
    flag = tf.less_equal(dist, 0)
    return dist, vec, flag

def distance_ln_pl(ln1, pl2, dist1, dist2, N_sim, N_col, zeros_pt): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
    bx2 = tf.reshape(tf.expand_dims(pl2, axis=-3)-tf.expand_dims(ln1, axis=-2), (N_sim, N_col, 8, 3))
    dist, vec, flag = distance_pt_bx(zeros_pt, bx2, dist1, dist2, N_sim, N_col)
    return dist, vec, flag


# def distance_ln_bx(ln1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=6): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
#     dist, flag = distance_mesh_2_8(ln1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=IterationAllowed)


def distance_ln_bx(ln1, bx2, dist1, dist2, N_sim, N_col, zeros_pt): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
    pt1 = tf.reshape(ln1, (N_sim, N_col*2, 1, 3))
    bx2_ = tf.repeat(bx2, 2, axis=-3)
    dist_, vec_, flag_ = distance_pt_bx(pt1, bx2_, 0, 0, N_sim, N_col*2)
    dist_ = tf.reshape(dist_, (N_sim, N_col, 2, 1))
    vec_ = tf.reshape(vec_, (N_sim, N_col, 2, 3))
    i_dist_ = K.argmin(dist_, axis=-2)
    dist_ = tf.gather_nd(dist_, i_dist_, batch_dims=2)
    vec_ = tf.gather_nd(vec_, i_dist_, batch_dims=2)
    # flag_ = tf.reduce_any(tf.reshape(flag_, (N_sim, N_col, 2)), axis=-1, keepdims=True)
    ln1_ = tf.tile(ln1, [1,12,1,1])
    bx2_perm1 = tf.gather(bx2, [0,1,2,3, 4,5,6,7, 0,1,2,3], axis=-2)
    bx2_perm2 = tf.gather(bx2, [1,2,3,0, 5,6,7,4, 4,5,6,7], axis=-2)
    ln2_ = tf.reshape(tf.stack([bx2_perm1, bx2_perm2], axis=-2), [N_sim, N_col*12, 2, 3])
    ln1_ = tf.repeat(ln1, 12, axis=-3)
    zeros_pt_ = tf.repeat(zeros_pt, 12, axis=-3)
    dist__, vec__, flag__ = distance_ln_ln(ln1_, ln2_, 0, 0, N_sim, N_col*12, zeros_pt_)
    dist__ = tf.reshape(dist__, (N_sim, N_col, 12, 1))
    vec__ = tf.reshape(vec__, (N_sim, N_col, 12, 3))
    i_dist__ = K.argmin(dist__, axis=-2)
    dist__ = tf.gather_nd(dist__, i_dist__, batch_dims=2)
    vec__ = tf.gather_nd(vec__, i_dist__, batch_dims=2)
    # flag__ = tf.reduce_any(tf.reshape(flag__, (N_sim, N_col, 12)), axis=-1, keepdims=True)
    dist_stack = tf.stack([dist_, dist__], axis=-2)
    vec_stack = tf.stack([vec_, vec__], axis=-2)
    i_dist = K.argmin(dist_stack, axis=-2)
    dist = tf.gather_nd(dist_stack, i_dist, batch_dims=2) - (dist1+dist2)
    vec = tf.gather_nd(vec_stack, i_dist, batch_dims=2)
    flag = tf.less_equal(dist, 0)
    return dist, vec, flag

#def distance_ln_bx(ln1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=6): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
#    dist, vec, flag = distance_mesh(ln1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=IterationAllowed)
#    return dist, vec, flag

def distance_pl_pl(pl1, pl2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=6): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
    dist, vec, flag = distance_mesh(pl1, pl2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=IterationAllowed)
    return dist, vec, flag

def distance_pl_bx(pl1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=6): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
    dist, vec, flag = distance_mesh(pl1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=IterationAllowed)
    return dist, vec, flag

def distance_bx_bx(bx1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=6): # (N_sim, N_col, 1, 3), (N_sim, N_col, 4, 3) / (N_sim, N_col, 1)
    dist, vec, flag = distance_mesh(bx1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch, IterationAllowed=IterationAllowed)
    return dist, vec, flag

@tf.function
def test_all(pt1, pt2, ln1, ln2, pl1, pl2, bx1, bx2, dist1, dist2, zeros_pt, N_sim, N_col, flag_default, dist_default, x_batch, y_batch):
    dist1, vec, flag = distance_pt_pt(pt1, pt2, dist1, dist2)
    dist2, vec, flag = distance_pt_ln(pt1, ln2, dist1, dist2)
    dist3, vec, flag = distance_pt_pl(pt1, pl2, dist1, dist2, N_sim, N_col)
    dist4, vec, flag = distance_pt_bx(pt1, bx2, dist1, dist2, N_sim, N_col)
    dist5, vec, flag = distance_ln_ln(ln1, ln2, dist1, dist2, N_sim, N_col, zeros_pt)
    dist6, vec, flag = distance_ln_pl(ln1, pl2, dist1, dist2, N_sim, N_col, zeros_pt)
    dist7, vec, flag = distance_ln_bx(ln1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch)
    dist8, vec, flag = distance_pl_pl(pl1, pl2, dist1, dist2, flag_default, dist_default, x_batch, y_batch)
    dist9, vec, flag = distance_pl_bx(pl1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch)
    dist10, vec, flag = distance_bx_bx(bx1, bx2, dist1, dist2, flag_default, dist_default, x_batch, y_batch)
    return dist1, dist2, dist3, dist4, dist5, dist6, dist7, dist8, dist9, dist10
    

def get_zero_points(N_sim, N_col):
    return tf.zeros((N_sim, N_col, 1, 3))