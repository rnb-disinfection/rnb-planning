from .resnet import *
from tensorflow.keras import Model
            
class ResNetModelTP(Model):
    def __init__(self, 
                 ConvLayer=KL.Conv3D, ZeroPadding=KL.ZeroPadding3D, MaxPool=KL.MaxPool3D, 
                 ConvBlock=ConvBlock3D, IdentityBlock=IdentityBlock3D, activation=tf.keras.activations.swish):
        super(ResNetModelTP, self).__init__()
        self.resnet_grasp = ResNet(stage2=[32, 32, 64], stage3=[64, 64, 128], stage4=[128, 128, 256], 
                                   stage5=[256, 256, 512], ConvLayer=ConvLayer, ZeroPadding=ZeroPadding, MaxPool=MaxPool, 
                                   ConvBlock=ConvBlock, IdentityBlock=IdentityBlock, activation=activation)
        self.resnet_arm = ResNet(stage2=[32, 32, 64], stage3=[64, 64, 128], stage4=[128, 128, 256], 
                                   stage5=[256, 256, 512], ConvLayer=ConvLayer, ZeroPadding=ZeroPadding, MaxPool=MaxPool, 
                                 ConvBlock=ConvBlock, IdentityBlock=IdentityBlock, activation=activation)
        DROPOUT_RATIO = 0.2
        self.dens1_ee = KL.Dense(256, name="dens1_ee", activation=activation)
        self.dropout1_ee = KL.Dropout(DROPOUT_RATIO)
        self.dens2_ee = KL.Dense(512, name="dens2_ee", activation=tf.keras.activations.sigmoid)
        self.dropout2_ee = KL.Dropout(DROPOUT_RATIO)
        self.fl_grasp = KL.Flatten()
        self.fl_arm = KL.Flatten()
        self.mult = KL.Multiply()
        dense_depth1, dense_depth2, dense_depth3= 256, 128, 64
        self.dens1_grasp = KL.Dense(dense_depth1, name="dens1_grasp", activation=activation) # DenseBN(512, "dens1")
        self.dens1_arm = KL.Dense(dense_depth1, name="dens1_arm", activation=activation) # DenseBN(512, "dens1")
        self.dropout1_grasp = KL.Dropout(DROPOUT_RATIO)
        self.dropout1_arm = KL.Dropout(DROPOUT_RATIO)
        self.concat = KL.Concatenate()
        self.dens2 = DenseBN(dense_depth2, name="dens2", activation=activation) # DenseBN(512, "dens1")
        self.dropout2 = KL.Dropout(DROPOUT_RATIO)
        self.dens3 = DenseBN(dense_depth3, "dens3", activation=activation)
        self.dropout3 = KL.Dropout(DROPOUT_RATIO)
        self.dens_out = KL.Dense(2, activation=tf.keras.activations.sigmoid)

    def call(self, inputs, training=False):
        img_grasp, img_arm, mask_ee = inputs
        C5_grasp = self.resnet_grasp(img_grasp, training=training)
        C5_arm = self.resnet_arm(img_arm, training=training)
        
        C_ee = self.dens1_ee(mask_ee, training=training)
        C_ee = self.dropout1_ee(C_ee, training=training)
        C_ee = self.dens2_ee(C_ee, training=training)
        C_ee = self.dropout2_ee(C_ee, training=training)
        
        x_grasp = self.fl_grasp(C5_grasp)
        x_grasp = self.dens1_grasp(x_grasp, training=training)
        x_grasp = self.dropout1_grasp(x_grasp, training=training)
        
        x_arm = self.fl_arm(C5_arm)
        x_arm = self.mult([x_arm, C_ee])
        x_arm = self.dens1_arm(x_arm, training=training)
        x_arm = self.dropout1_arm(x_arm, training=training)
        
        x = self.concat([x_grasp, x_arm])
        
        x = self.dens2(x, training=training)
        x = self.dropout2(x, training=training)
        x = self.dens3(x, training=training)
        x = self.dropout3(x, training=training)
        x = self.dens_out(x, training=training)
        return x

