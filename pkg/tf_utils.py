import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras import backend as K

def clip_gradient(gradients, grad_max=1):
    mx_grad = K.maximum(K.max(K.abs(gradients),axis=0, keepdims=True),grad_max)
    gradients = tf.divide(gradients, mx_grad) * grad_max
    return gradients

def clip_gradient_elem_wise(gradients, grad_max=1):
    grad_rs = gradients/grad_max
    grad_abs = K.abs(grad_rs)+1e-20
    grad_nm = K.maximum(1.0, K.max(grad_abs, axis=-1, keepdims=True))
    return grad_rs/grad_nm*grad_max
    
#     mx_grad = K.max(gradients)
#     if mx_grad>grad_max:
#         for i_g in range(len(gradients)):
#             gradients[i_g] /= mx_grad
#     return gradients

def tf_normalize(X, axis=-1):
    norm = tf_norm(X, axis=-1, keepdims=True)
    vec = tf.divide(X, norm)
    return vec, norm 

def tf_norm(X, axis=-1, keepdims=False):
    return K.sqrt(K.sum(K.square(X),axis=axis,keepdims=keepdims)+1e-20)

def tf_R2w(R, N_sim):
    logR = tf.cast(tf.linalg.logm(tf.cast(R, dtype=tf.complex64)), dtype=tf.float32)
    return tf.gather_nd(logR, [[[2,1],[0,2],[1,0]]]*N_sim, batch_dims=1) 

def tf_T2V(T, N_sim):
    RP = tf.gather(T, [0,1,2], axis=-2) # N_sim, N_obj, 3,4
    R = tf.gather(RP, [0,1,2], axis=-1)
    P = tf.gather(RP, 3, axis=-1)
    w = tf_R2w(R, N_sim)
    return tf.concat([P,w], axis=-1)


    
def set_assign(inst, name, val, dtype=tf.float32):
    if hasattr(inst, name):
        getattr(inst, name).assign(val)
    else:
        setattr(inst, name, tf.Variable(val, dtype=dtype, trainable=False))
    