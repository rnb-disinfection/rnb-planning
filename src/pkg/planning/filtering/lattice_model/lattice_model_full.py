from .resnet import *
import tensorflow as tf

from tensorflow.keras import layers as KL
from tensorflow.keras import Model
import datetime
            
class ResNetModelTP_FULL(Model):
    def __init__(self, 
                 ConvLayer=KL.Conv3D, ZeroPadding=KL.ZeroPadding3D, MaxPool=KL.MaxPool3D, 
                 ConvBlock=ConvBlock3D, IdentityBlock=IdentityBlock3D, activation=tf.keras.activations.swish):
        super(ResNetModelTP_FULL, self).__init__()
        self.resnet = ResNet(stage5=[512, 512, 1024], ConvLayer=ConvLayer, ZeroPadding=ZeroPadding, MaxPool=MaxPool, 
                             ConvBlock=ConvBlock, IdentityBlock=IdentityBlock, activation=activation,
                             preconv_kernel=5, preconv_strides=2, preconv_padding="same", stride_2=2
                            )
        self.fl = KL.Flatten()
        dense_depth1, dense_depth2, dense_depth3= 512, 256, 64
        self.dens1 = KL.Dense(dense_depth1, name="dens1", activation=activation) # DenseBN(512, "dens1")
        self.dropout1 = KL.Dropout(0.2)
        self.dens2 = DenseBN(dense_depth2, name="dens2", activation=activation) # DenseBN(512, "dens1")
        self.dropout2 = KL.Dropout(0.2)
        self.dens3 = DenseBN(dense_depth3, "dens3", activation=activation)
        self.dropout3 = KL.Dropout(0.2)
        self.dens_out = KL.Dense(2, activation=tf.keras.activations.sigmoid)

    def call(self, input_image, training=False):
        C5 = self.resnet(input_image, training=training)
        
        x = self.fl(C5)
        x = self.dens1(x, training=training)
        x = self.dropout1(x, training=training)
        x = self.dens2(x, training=training)
        x = self.dropout2(x, training=training)
        x = self.dens3(x, training=training)
        x = self.dropout3(x, training=training)
        x = self.dens_out(x, training=training)
        return x

