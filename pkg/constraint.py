import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras import backend as K


class JointConstraintLoss(layers.Layer):
    def __init__(self, robot, *args, **kwargs):
        if "name" not in kwargs:
            kwargs["name"] = "{}_{}".format(robot.rname, "jc")
        super(JointConstraintLoss, self).__init__(*args, **kwargs)
        self.robot = robot
        
    # 변수를 만듭니다.
    def build(self, input_shape):
        pass

    # call 메서드가 그래프 모드에서 사용되면
    # training 변수는 텐서가 됩니다.
    @tf.function
    def call(self, inputs):
        Qtar = inputs[0] # (NxM)
        bAct = inputs[1] # (N)
        return K.sum(bAct * K.sum(K.square((Qtar - self.robot.get_Q())), axis=1))
    
    
class FrameConstraintLoss(layers.Layer):
    def __init__(self, *args, l_frame=0.1, gamma_R=5, gamma_P=1, **kwargs):
        self.gamma_R, self.gamma_P = gamma_R, gamma_P
        self.DirRef = tf.constant([[[l_frame, 0, 0],
                                      [0, l_frame, 0],
                                      [0, 0, l_frame]
                                     ]], dtype='float32')
        super(FrameConstraintLoss, self).__init__(*args, **kwargs)
        
    # 변수를 만듭니다.
    def build(self, input_shape):
        pass

    # call 메서드가 그래프 모드에서 사용되면
    # training 변수는 텐서가 됩니다.
    @tf.function
    def call(self, inputs):
        Tcur = inputs[0] # (Nx4x4)
        Ttar = inputs[1] # (Nx4x4)
        bAct = inputs[2] # (N)
        DirTar = tf.matmul(Ttar[:,:3,:3], self.DirRef)
        DirCur = tf.matmul(Tcur[:,:3,:3], self.DirRef)
        dR_nm = K.sum(K.sum(K.square(DirTar-DirCur), axis=-1), axis=-1)
        dP_nm = K.sum(K.square(Ttar[:,:3,3]-Tcur[:,:3,3]), axis=-1)
        return K.sum(bAct * (self.gamma_R*dR_nm+self.gamma_P*dP_nm))