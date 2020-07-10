import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras import backend as K

@tf.function
def clip_gradient(gradients, grad_max=1):
    mx_grad = K.maximum(K.max(K.abs(gradients),axis=0, keepdims=True),grad_max)
    gradients = tf.divide(gradients, mx_grad) * grad_max
    return gradients
    
#     mx_grad = K.max(gradients)
#     if mx_grad>grad_max:
#         for i_g in range(len(gradients)):
#             gradients[i_g] /= mx_grad
#     return gradients