import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras import backend as K
import numpy as np

class RotMatLayer(layers.Layer):
    def __init__(self, axis, dim):
        self.axis = axis
        self.dim = dim
        super(RotMatLayer, self).__init__()
        if self.axis==1:
            self.r1 = tf.constant([[[1,0,0],
                                   [0,0,0],
                                   [0,0,0]]]*self.dim, dtype='float32')
            self.rc = tf.constant([[[0,0,0],
                                   [0,1,0],
                                   [0,0,1]]]*self.dim, dtype='float32')
            self.rs = tf.constant([[[0,0,0],
                                   [0,0,-1],
                                   [0,1,0]]]*self.dim, dtype='float32')
        elif self.axis==2:
            self.r1 = tf.constant([[[0,0,0],
                                   [0,1,0],
                                   [0,0,0]]]*self.dim, dtype='float32')
            self.rc = tf.constant([[[1,0,0],
                                   [0,0,0],
                                   [0,0,1]]]*self.dim, dtype='float32')
            self.rs = tf.constant([[[0,0,1],
                                   [0,0,0],
                                   [-1,0,0]]]*self.dim, dtype='float32')
        elif self.axis==3:
            self.r1 = tf.constant([[[0,0,0],
                                   [0,0,0],
                                   [0,0,1]]]*self.dim, dtype='float32')
            self.rc = tf.constant([[[1,0,0],
                                   [0,1,0],
                                   [0,0,0]]]*self.dim, dtype='float32')
            self.rs = tf.constant([[[0,-1,0],
                                   [1,0,0],
                                   [0,0,0]]]*self.dim, dtype='float32')
        else:
            raise(RuntimeError("axis should be in [1,2,3]"))
        
    # 변수를 만듭니다.
    def build(self, input_shape):
        pass

    # call 메서드가 그래프 모드에서 사용되면
    # training 변수는 텐서가 됩니다.
    @tf.function
    def call(self, input):
        return self.r1 + self.rc*K.reshape(K.cos(input), (-1,1,1)) + self.rs*K.reshape(K.sin(input), (-1,1,1))
    

class RotZYXLayer(layers.Layer):
    def __init__(self, dim):
        super(RotZYXLayer, self).__init__()
        self.Rz = RotMatLayer(3, dim)
        self.Ry = RotMatLayer(2, dim)
        self.Rx = RotMatLayer(1, dim)
        
    # 변수를 만듭니다.
    def build(self, input_shape):
        pass

    # call 메서드가 그래프 모드에서 사용되면
    # training 변수는 텐서가 됩니다.
    @tf.function
    def call(self, inputs):
        Rz = self.Rz(inputs[:,0])
        Ry = self.Ry(inputs[:,1])
        Rx = self.Rx(inputs[:,2])
        return tf.matmul(tf.matmul(Rz,Ry),Rx)
    
    
class JointMatLayer(layers.Layer):
    def __init__(self, axis, dim, ltype, *args, **kwargs):
        self.axis = axis
        self.dim = dim
        self.ltype = ltype
        super(JointMatLayer, self).__init__(*args, **kwargs)
        if self.ltype=='revolute':
            if self.axis==1:
                self.r1 = tf.constant([[[1,0,0,0],
                                       [0,0,0,0],
                                       [0,0,0,0],
                                       [0,0,0,1]]]*self.dim, dtype='float32')
                self.rc = tf.constant([[[0,0,0,0],
                                       [0,1,0,0],
                                       [0,0,1,0],
                                       [0,0,0,0]]]*self.dim, dtype='float32')
                self.rs = tf.constant([[[0,0,0,0],
                                       [0,0,-1,0],
                                       [0,1,0,0],
                                       [0,0,0,0]]]*self.dim, dtype='float32')
            elif self.axis==2:
                self.r1 = tf.constant([[[0,0,0,0],
                                       [0,1,0,0],
                                       [0,0,0,0],
                                       [0,0,0,1]]]*self.dim, dtype='float32')
                self.rc = tf.constant([[[1,0,0,0],
                                       [0,0,0,0],
                                       [0,0,1,0],
                                       [0,0,0,0]]]*self.dim, dtype='float32')
                self.rs = tf.constant([[[0,0,1,0],
                                       [0,0,0,0],
                                       [-1,0,0,0],
                                       [0,0,0,0]]]*self.dim, dtype='float32')
            elif self.axis==3:
                self.r1 = tf.constant([[[0,0,0,0],
                                       [0,0,0,0],
                                       [0,0,1,0],
                                       [0,0,0,1]]]*self.dim, dtype='float32')
                self.rc = tf.constant([[[1,0,0,0],
                                       [0,1,0,0],
                                       [0,0,0,0],
                                       [0,0,0,0]]]*self.dim, dtype='float32')
                self.rs = tf.constant([[[0,-1,0,0],
                                       [1,0,0,0],
                                       [0,0,0,0],
                                       [0,0,0,0]]]*self.dim, dtype='float32')
            else:
                raise(RuntimeError("axis should be in [1,2,3]"))
        elif self.ltype=='prismatic':
            self.r1 = tf.constant([[[1,0,0,0],
                                   [0,1,0,0],
                                   [0,0,1,0],
                                   [0,0,0,1]]]*self.dim, dtype='float32')
            if self.axis==1:
                self.rq = tf.constant([[[0,0,0,1],
                                       [0,0,0,0],
                                       [0,0,0,0],
                                       [0,0,0,0]]]*self.dim, dtype='float32')
            elif self.axis==2:
                self.rq = tf.constant([[[0,0,0,0],
                                       [0,0,0,1],
                                       [0,0,0,0],
                                       [0,0,0,0]]]*self.dim, dtype='float32')
            elif self.axis==3:
                self.rq = tf.constant([[[0,0,0,0],
                                       [0,0,0,0],
                                       [0,0,0,1],
                                       [0,0,0,0]]]*self.dim, dtype='float32')
            else:
                raise(RuntimeError("axis should be in [1,2,3]"))
        elif self.ltype == 'fixed':
            self.r1 = tf.constant([[[1,0,0,0],
                                   [0,1,0,0],
                                   [0,0,1,0],
                                   [0,0,0,1]]]*self.dim, dtype='float32')
        else:
            raise(RuntimeError("link type {} not implemented".format(self.ltype)))
        
    # 변수를 만듭니다.
    def build(self, input_shape):
        pass

    # call 메서드가 그래프 모드에서 사용되면
    # training 변수는 텐서가 됩니다.
    @tf.function
    def call(self, input=None):
        if self.ltype=='revolute':
            return self.r1 + self.rc*K.reshape(K.cos(input), (-1,1,1)) + self.rs*K.reshape(K.sin(input), (-1,1,1))
        elif self.ltype=='prismatic':
            return self.r1 + self.rq*K.reshape(input, (-1,1,1))
        elif self.ltype=='fixed':
            return self.r1