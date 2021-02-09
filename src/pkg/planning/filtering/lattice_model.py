from .resnet import *
from tensorflow.keras import Model
            
class ResNetModelTP(Model):
    def __init__(self, 
                 ConvLayer=KL.Conv3D, ZeroPadding=KL.ZeroPadding3D, MaxPool=KL.MaxPool3D, 
                 ConvBlock=ConvBlock3D, IdentityBlock=IdentityBlock3D, activation=tf.keras.activations.swish):
        super(ResNetModelTP, self).__init__()
        self.resnet_grasp = ResNet(stage5=[512, 512, 256], ConvLayer=ConvLayer, ZeroPadding=ZeroPadding, MaxPool=MaxPool, 
                                   ConvBlock=ConvBlock, IdentityBlock=IdentityBlock, activation=activation)
        self.resnet_arm = ResNet(stage5=[512, 512, 1024], ConvLayer=ConvLayer, ZeroPadding=ZeroPadding, MaxPool=MaxPool, 
                                 ConvBlock=ConvBlock, IdentityBlock=IdentityBlock, activation=activation)
        self.fl_grasp = KL.Flatten()
        self.fl_arm = KL.Flatten()
        self.mult = KL.Multiply()
        dense_depth1, dense_depth2, dense_depth3= 256, 128, 64
        self.dens1_grasp = KL.Dense(dense_depth1, name="dens1_grasp", activation=activation) # DenseBN(512, "dens1")
        self.dens1_arm = KL.Dense(dense_depth1, name="dens1_arm", activation=activation) # DenseBN(512, "dens1")
        self.dropout1_grasp = KL.Dropout(0.2)
        self.dropout1_arm = KL.Dropout(0.2)
        self.concat = KL.Concatenate()
        self.dens2 = DenseBN(dense_depth2, name="dens2", activation=activation) # DenseBN(512, "dens1")
        self.dropout2 = KL.Dropout(0.2)
        self.dens3 = DenseBN(dense_depth3, "dens3", activation=activation)
        self.dropout3 = KL.Dropout(0.2)
        self.dens_out = KL.Dense(2, activation=tf.keras.activations.sigmoid)

    def call(self, inputs, training=False):
        img_grasp, img_arm, mask_arm = inputs
        C5_grasp = self.resnet_grasp(img_grasp, training=training)
        C5_arm = self.resnet_arm(img_arm, training=training)
        
        x_grasp = self.fl_grasp(C5_grasp)
        x_grasp = self.dens1_grasp(x_grasp, training=training)
        x_grasp = self.dropout1_grasp(x_grasp, training=training)
        
        x_arm = self.fl_arm(C5_arm)
        x_arm = self.mult([x_arm, mask_arm])
        x_arm = self.dens1_arm(x_arm, training=training)
        x_arm = self.dropout1_arm(x_arm, training=training)
        
        x = self.concat([x_grasp, x_arm])
        
        x = self.dens2(x, training=training)
        x = self.dropout2(x, training=training)
        x = self.dens3(x, training=training)
        x = self.dropout3(x, training=training)
        x = self.dens_out(x, training=training)
        return x

