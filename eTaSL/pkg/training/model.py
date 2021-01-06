import tensorflow as tf

from tensorflow.keras import layers as KL
from tensorflow.keras import Model
import datetime

class DenseBN(KL.Layer):
    def __init__(self, units, name, use_bias=False, activation="relu"):
        super(DenseBN, self).__init__()
        self.dense = KL.Dense(units, name='dense_' + name, use_bias=use_bias)
        self.bn = KL.BatchNormalization(name='bn_' + name)
        self.ac = KL.Activation(activation)
        
    def call(self, x, training=False):
        x = self.dense(x)
        x = self.bn(x, training=training)
        x = self.ac(x)
        return x
        

class IdentityBlock(KL.Layer):
    def __init__(self, kernel_size, filters, stage, block,
                 use_bias=False, use_bn=True, ConvLayer=KL.Conv2D):
        super(IdentityBlock, self).__init__()
        self.use_bn = use_bn
        nb_filter1, nb_filter2, nb_filter3 = filters
        conv_name_base = 'res' + str(stage) + block + '_branch'
        bn_name_base = 'bn' + str(stage) + block + '_branch'
        
        self.conv1 = ConvLayer(nb_filter1, 1, name=conv_name_base + '2a',
                               use_bias=use_bias)
        if self.use_bn:
            self.bn1 = KL.BatchNormalization(name=bn_name_base + '2a')
        self.ac1 = KL.Activation('relu')
        
        self.conv2 = ConvLayer(nb_filter2, kernel_size, padding="same",
                               name=conv_name_base + '2b', use_bias=use_bias)
        if self.use_bn:
            self.bn2 = KL.BatchNormalization(name=bn_name_base + '2b')
        self.ac2 = KL.Activation('relu')
        
        self.conv3 = ConvLayer(nb_filter3, 1, name=conv_name_base + '2c',
                               use_bias=use_bias)
        if self.use_bn:
            self.bn3 = KL.BatchNormalization(name=bn_name_base + '2c')
        self.add = KL.Add()
        self.ac3 = KL.Activation('relu', name='res' + str(stage) + block + '_out')
        
    def call(self, input_tensor, training=False):
        x = self.conv1(input_tensor)
        if self.use_bn:
            x = self.bn1(x, training=training)
        x = self.ac1(x)

        x = self.conv2(x)
        if self.use_bn:
            x = self.bn2(x, training=training)
        x = self.ac2(x)

        x = self.conv3(x)
        if self.use_bn:
            x = self.bn3(x, training=training)

        x = self.add([x, input_tensor])
        x = self.ac3(x)
        return x
    
class IdentityBlock3D(IdentityBlock):
    def __init__(self, *args, **kwargs):
        super(IdentityBlock3D, self).__init__(*args, ConvLayer=KL.Conv3D, **kwargs)
    
    

class ConvBlock(KL.Layer):
    def __init__(self, kernel_size, filters, stage, block, strides=2,
                 use_bias=False, use_bn=True, ConvLayer=KL.Conv2D):
        super(ConvBlock, self).__init__()
        self.use_bn = use_bn
        nb_filter1, nb_filter2, nb_filter3 = filters
        conv_name_base = 'res' + str(stage) + block + '_branch'
        bn_name_base = 'bn' + str(stage) + block + '_branch'
        
        self.conv1 = ConvLayer(nb_filter1, 1, strides=strides, 
                               name=conv_name_base + '2a', use_bias=use_bias)
        if self.use_bn:
            self.bn1 = KL.BatchNormalization(name=bn_name_base + '2a')
        self.ac1 = KL.Activation('relu')
        
        self.conv2 = ConvLayer(nb_filter2, kernel_size, padding="same",
                               name=conv_name_base + '2b', use_bias=use_bias)
        if self.use_bn:
            self.bn2 = KL.BatchNormalization(name=bn_name_base + '2b')
        self.ac2 = KL.Activation('relu')
        
        self.conv3 = ConvLayer(nb_filter3, 1, name=conv_name_base + '2c',
                               use_bias=use_bias)
        if self.use_bn:
            self.bn3 = KL.BatchNormalization(name=bn_name_base + '2c')

        self.convs = ConvLayer(nb_filter3, 1, strides=strides,
                               name=conv_name_base + '1', use_bias=use_bias)
        if self.use_bn:
            self.bns = KL.BatchNormalization(name=bn_name_base + '1')
    
        self.add = KL.Add()
        self.ac3 = KL.Activation('relu', name='res' + str(stage) + block + '_out')
        
    def call(self, input_tensor, training=False):
        x = self.conv1(input_tensor)
        if self.use_bn:
            x = self.bn1(x, training=training)
        x = self.ac1(x)

        x = self.conv2(x)
        if self.use_bn:
            x = self.bn2(x, training=training)
        x = self.ac2(x)

        x = self.conv3(x)
        if self.use_bn:
            x = self.bn3(x, training=training)
        
        shortcut = self.convs(input_tensor)
        if self.use_bn:
            shortcut = self.bns(shortcut, training=training)

        x = self.add([x, shortcut])
        x = self.ac3(x)
        return x
    
class ConvBlock3D(ConvBlock):
    def __init__(self, *args, **kwargs):
        super(ConvBlock3D, self).__init__(*args, ConvLayer=KL.Conv3D, **kwargs)
    

class ResNet(KL.Layer):
    def __init__(self, architecture="resnet50", stage0=None, stage1=None, stage2=[64, 64, 256], stage3=[128, 128, 512], 
                 stage4=[256, 256, 1024], stage5=None, 
                 ConvLayer=KL.Conv2D, ZeroPadding=KL.ZeroPadding2D, MaxPool=KL.MaxPooling2D, 
                 ConvBlock=ConvBlock, IdentityBlock=IdentityBlock, input_size=(15,15,15), joint_num=13, batch_size=16):
        super(ResNet, self).__init__()
        """Build a ResNet graph.
            architecture: Can be resnet50 or resnet101
            stage5: recommanded default values = [512, 512, 2048]. If None, stage5 of the network is not created
        """
        assert architecture in ["resnet50", "resnet101"]
        self.stage1 = stage1 is not None
        self.stage5 = stage5 is not None
        self.stage0 = stage0 is not None
        self.input_size = input_size
        self.joint_num = joint_num
        self.batch_size = batch_size
        if self.stage0:
            self.pre_boxa = ConvBlock(1, stage0, stage=0, block='ba', strides=1, use_bias=True, use_bn=False)
            self.pre_boxb = IdentityBlock(1, stage0, stage=0, block='bb')
            self.pre_boxc = IdentityBlock(1, stage0, stage=0, block='bc')
            self.pre_cyla = ConvBlock(1, stage0, stage=0, block='ca', strides=1, use_bias=True, use_bn=False)
            self.pre_cylb = IdentityBlock(1, stage0, stage=0, block='cb')
            self.pre_cylc = IdentityBlock(1, stage0, stage=0, block='cc')
            self.pre_inia = ConvBlock(1, stage0, stage=0, block='ia', strides=1, use_bias=True, use_bn=False)
            self.pre_inib = IdentityBlock(1, stage0, stage=0, block='ib')
            self.pre_inic = IdentityBlock(1, stage0, stage=0, block='ic')
            self.pre_gola = ConvBlock(1, stage0, stage=0, block='ga', strides=1, use_bias=True, use_bn=False)
            self.pre_golb = IdentityBlock(1, stage0, stage=0, block='gb')
            self.pre_golc = IdentityBlock(1, stage0, stage=0, block='gc')
            self.concat = KL.Concatenate()
        
        if self.stage1:
            # Stage 1  output size = 1/1
            self.cb1a = ConvBlock(3, stage1, stage=1, block='a', strides=1)
            self.ib1b = IdentityBlock(3, stage1, stage=1, block='b')
            self.ib1c = IdentityBlock(3, stage1, stage=1, block='c')
            stride_2 = 2
        else:
            # Stage 1 original output size = 1/4
            self.cv1 = ConvLayer(64, 7, strides=2, 
                                 name='conv1', padding="same", use_bias=False)
            self.bn1 = KL.BatchNormalization(name='bn_conv1')
            self.ac1 = KL.Activation('relu')
            self.mp1 = MaxPool(3, strides=2, padding="same")
            stride_2 = 1
        
        # Stage 2  output size = 1/stride_2
        self.cb2a = ConvBlock(3, stage2, stage=2, block='a', strides=stride_2)
        self.ib2b = IdentityBlock(3, stage2, stage=2, block='b')
        self.ib2c = IdentityBlock(3, stage2, stage=2, block='c')
        
        # Stage 3  output size = 1/2
        self.cb3a = ConvBlock(3, stage3, stage=3, block='a')
        self.ib3b = IdentityBlock(3, stage3, stage=3, block='b')
        self.ib3c = IdentityBlock(3, stage3, stage=3, block='c')
        self.ib3d = IdentityBlock(3, stage3, stage=3, block='d')
        
        # Stage 4  output size = 1/2
        self.cb4a = ConvBlock(3, stage4, stage=4, block='a')
        block_count = {"resnet50": 5, "resnet101": 22}[architecture]
        self.ib4_list = []
        for i in range(block_count):
            self.ib4_list.append(
                IdentityBlock(3, stage4, stage=4, block=chr(98 + i))
            )
            
        # Stage 5  output size = 1/2
        if self.stage5:
            self.cb5a = ConvBlock(3, stage5, stage=5, block='a')
            self.ib5b = IdentityBlock(3, stage5, stage=5, block='b')
            self.ib5c = IdentityBlock(3, stage5, stage=5, block='c')

    def call(self, input_image, training=False):
        if self.stage0:
            cbox, cbox_m, ccyl, ccyl_m, ibox, ibox_m, gbox, gbox_m = input_image
            cbox = self.pre_boxa(cbox, training=training)
            cbox = self.pre_boxb(cbox, training=training)
            cbox = self.pre_boxc(cbox, training=training)*cbox_m
            ccyl = self.pre_cyla(ccyl, training=training)
            ccyl = self.pre_cylb(ccyl, training=training)
            ccyl = self.pre_cylc(ccyl, training=training)*ccyl_m
            ibox = self.pre_inia(ibox, training=training)
            ibox = self.pre_inib(ibox, training=training)
            ibox = self.pre_inic(ibox, training=training)*ibox_m
            gbox = self.pre_gola(gbox, training=training)
            gbox = self.pre_golb(gbox, training=training)
            gbox = self.pre_golc(gbox, training=training)*gbox_m
            x = self.concat([cbox, ccyl, ibox, gbox])
        else:
            x = input_image
        # Stage 1
        if self.stage1:
            x = self.cb1a(x, training=training)
            x = self.ib1b(x, training=training)
            C1 = x = self.ib1c(x, training=training)
        else:
            x = self.cv1(x)
            x = self.bn1(x, training=training)
            x = self.ac1(x)
            C1 = x = self.mp1(x)
        
        # Stage 2
        x = self.cb2a(x, training=training)
        x = self.ib2b(x, training=training)
        C2 = x = self.ib2c(x, training=training)
        
        # Stage 3
        x = self.cb3a(x, training=training)
        x = self.ib3b(x, training=training)
        x = self.ib3c(x, training=training)
        C3 = x = self.ib3d(x, training=training)
        
        # Stage 4
        x = self.cb4a(x, training=training)
        for ib4 in self.ib4_list:
            x = ib4(x, training=training)
        C4 = x
            
        # Stage 5
        if self.stage5:
            x = self.cb5a(x, training=training)
            x = self.ib5b(x, training=training)
            C5 = x = self.ib5c(x, training=training)
            return [C1, C2, C3, C4, C5]
        else:
            return [C1, C2, C3, C4]
            
class ResNetModelTP(Model):
    def __init__(self, 
                 ConvLayer=KL.Conv3D, ZeroPadding=KL.ZeroPadding3D, MaxPool=KL.GlobalMaxPool3D, 
                 ConvBlock=ConvBlock3D, IdentityBlock=IdentityBlock3D):
        super(ResNetModelTP, self).__init__()
        self.resnet = ResNet(architecture="resnet50", 
                             stage0 = [64,64,32], stage1=[64, 64, 64], 
                             stage2=[64, 64, 64], stage3=[64, 64, 64], 
                             stage4=[64, 64, 64], stage5=[64, 64, 64], 
                             ConvLayer=ConvLayer, ZeroPadding=ZeroPadding, MaxPool=MaxPool, 
                             ConvBlock=ConvBlock, IdentityBlock=IdentityBlock)
        dense_depth1, dense_depth2= 128, 64
        self.gp1 = MaxPool()
        self.gp2 = MaxPool()
        self.gp3 = MaxPool()
        self.gp4 = MaxPool()
        self.gp5 = MaxPool()
        self.dens11 = DenseBN(dense_depth1, "dens11")
        self.dens21 = DenseBN(dense_depth1, "dens21")
        self.dens31 = DenseBN(dense_depth1, "dens31")
        self.dens41 = DenseBN(dense_depth1, "dens41")
        self.dens51 = DenseBN(dense_depth1, "dens51")
        self.dropout11 = KL.Dropout(0.1)
        self.dropout21 = KL.Dropout(0.1)
        self.dropout31 = KL.Dropout(0.1)
        self.dropout41 = KL.Dropout(0.1)
        self.dropout51 = KL.Dropout(0.1)
        self.dens12 = KL.Dense(dense_depth2) # DenseBN(512, "dens1")
        self.dens22 = KL.Dense(dense_depth2) # DenseBN(512, "dens2")
        self.dens32 = KL.Dense(dense_depth2) # DenseBN(512, "dens3")
        self.dens42 = KL.Dense(dense_depth2) # DenseBN(512, "dens4")
        self.dens52 = KL.Dense(dense_depth2) # DenseBN(512, "dens5")
        self.dropout12 = KL.Dropout(0.1)
        self.dropout22 = KL.Dropout(0.1)
        self.dropout32 = KL.Dropout(0.1)
        self.dropout42 = KL.Dropout(0.1)
        self.dropout52 = KL.Dropout(0.1)
        self.concat = KL.Concatenate()
        self.dens_int1 = DenseBN(dense_depth1, "dens_int1")
        self.dropout1 = KL.Dropout(0.1)
        self.dens_int2 = DenseBN(dense_depth2, "dens_int2")
        self.dropout2 = KL.Dropout(0.1)
        self.dens_out = KL.Dense(2)

    def call(self, x, training=False):
        C1, C2, C3, C4, C5 = self.resnet(x, training=training)
        f1 = self.dens11(self.gp1(C1), training=training)
        f1 = self.dropout11(f1)
        f1 = self.dens12(f1, training=training)
        f1 = self.dropout12(f1)
        f2 = self.dens21(self.gp2(C2), training=training)
        f2 = self.dropout21(f2)
        f2 = self.dens22(f2, training=training)
        f2 = self.dropout22(f2)
        f3 = self.dens31(self.gp3(C3), training=training)
        f3 = self.dropout31(f3)
        f3 = self.dens32(f3, training=training)
        f3 = self.dropout32(f3)
        f4 = self.dens41(self.gp4(C4), training=training)
        f4 = self.dropout41(f4)
        f4 = self.dens42(f4, training=training)
        f4 = self.dropout42(f4)
        f5 = self.dens51(self.gp5(C5), training=training)
        f5 = self.dropout51(f5)
        f5 = self.dens52(f5, training=training)
        f5 = self.dropout52(f5)
        x = self.concat([f1,f2,f3,f4,f5])
        # x = f5
        x = self.dens_int1(x, training=training)
        x = self.dropout1(x, training=training)
        x = self.dens_int2(x, training=training)
        x = self.dropout2(x, training=training)
        x = self.dens_out(x)
        return x