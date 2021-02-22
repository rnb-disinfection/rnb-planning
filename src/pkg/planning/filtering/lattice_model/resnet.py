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
                 use_bias=False, use_bn=True, ConvLayer=KL.Conv2D, activation="relu"):
        super(IdentityBlock, self).__init__()
        self.use_bn = use_bn
        nb_filter1, nb_filter2, nb_filter3 = filters
        conv_name_base = 'res' + str(stage) + block + '_branch'
        bn_name_base = 'bn' + str(stage) + block + '_branch'
        
        self.conv1 = ConvLayer(nb_filter1, 1, name=conv_name_base + '2a',
                               use_bias=use_bias)
        if self.use_bn:
            self.bn1 = KL.BatchNormalization(name=bn_name_base + '2a')
        self.ac1 = KL.Activation(activation)
        
        self.conv2 = ConvLayer(nb_filter2, kernel_size, padding="same",
                               name=conv_name_base + '2b', use_bias=use_bias)
        if self.use_bn:
            self.bn2 = KL.BatchNormalization(name=bn_name_base + '2b')
        self.ac2 = KL.Activation(activation)
        
        self.conv3 = ConvLayer(nb_filter3, 1, name=conv_name_base + '2c',
                               use_bias=use_bias)
        if self.use_bn:
            self.bn3 = KL.BatchNormalization(name=bn_name_base + '2c')
        self.add = KL.Add()
        self.ac3 = KL.Activation(activation, name='res' + str(stage) + block + '_out')
        
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
                 use_bias=False, use_bn=True, ConvLayer=KL.Conv2D, activation='relu'):
        super(ConvBlock, self).__init__()
        self.use_bn = use_bn
        nb_filter1, nb_filter2, nb_filter3 = filters
        conv_name_base = 'res' + str(stage) + block + '_branch'
        bn_name_base = 'bn' + str(stage) + block + '_branch'
        
        self.conv1 = ConvLayer(nb_filter1, 1, strides=strides, 
                               name=conv_name_base + '2a', use_bias=use_bias)
        if self.use_bn:
            self.bn1 = KL.BatchNormalization(name=bn_name_base + '2a')
        self.ac1 = KL.Activation(activation)
        
        self.conv2 = ConvLayer(nb_filter2, kernel_size, padding="same",
                               name=conv_name_base + '2b', use_bias=use_bias)
        if self.use_bn:
            self.bn2 = KL.BatchNormalization(name=bn_name_base + '2b')
        self.ac2 = KL.Activation(activation)
        
        self.conv3 = ConvLayer(nb_filter3, 1, name=conv_name_base + '2c',
                               use_bias=use_bias)
        if self.use_bn:
            self.bn3 = KL.BatchNormalization(name=bn_name_base + '2c')

        self.convs = ConvLayer(nb_filter3, 1, strides=strides,
                               name=conv_name_base + '1', use_bias=use_bias)
        if self.use_bn:
            self.bns = KL.BatchNormalization(name=bn_name_base + '1')
    
        self.add = KL.Add()
        self.ac3 = KL.Activation(activation, name='res' + str(stage) + block + '_out')
        
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
    def __init__(self, architecture="resnet50", stage1=None, stage2=[64, 64, 128], stage3=[128, 128, 256], 
                 stage4=[256, 256, 512], stage5=None, 
                 ConvLayer=KL.Conv2D, ZeroPadding=KL.ZeroPadding2D, MaxPool=KL.MaxPooling2D, 
                 ConvBlock=ConvBlock, IdentityBlock=IdentityBlock, activation='relu', 
                 return_all_stage=False, preconv_kernel=5, preconv_strides=1, preconv_padding="valid", stride_2=None
                ):
        super(ResNet, self).__init__()
        """Build a ResNet graph.
            architecture: Can be resnet50 or resnet101
            stage5: recommanded default values = [512, 512, 2048]. If None, stage5 of the network is not created
        """
        assert architecture in ["resnet50", "resnet101"]
        self.stage1 = stage1 is not None
        self.stage5 = stage5 is not None
        self.return_all_stage = return_all_stage
        
        if self.stage1:
            # Stage 1  output size = 1/1
            self.cb1a = ConvBlock(3, stage1, stage=1, block='a', strides=1, activation=activation)
            self.ib1b = IdentityBlock(3, stage1, stage=1, block='b', activation=activation)
            self.ib1c = IdentityBlock(3, stage1, stage=1, block='c', activation=activation)
            stride_2 = 2 if stride_2 is None else stride_2
        else:
            # Stage 1 original output size = -4
            self.cv1 = ConvLayer(64, preconv_kernel, strides=preconv_strides, 
                                 name='conv1', padding=preconv_padding, use_bias=False)
            # # Stage 1 original output size = 1/2
            # self.cv1 = ConvLayer(64, 7, strides=2, 
            #                      name='conv1', padding="same", use_bias=False)
            self.bn1 = KL.BatchNormalization(name='bn_conv1')
            self.ac1 = KL.Activation(activation)
            # # Stage 1 output size = 1/2
            self.mp1 = MaxPool(2, strides=2, padding="valid")
            stride_2 = 1 if stride_2 is None else stride_2
        
        # Stage 2  output size = 1/stride_2
        self.cb2a = ConvBlock(3, stage2, stage=2, block='a', strides=stride_2, activation=activation)
        self.ib2b = IdentityBlock(3, stage2, stage=2, block='b', activation=activation)
        self.ib2c = IdentityBlock(3, stage2, stage=2, block='c', activation=activation)
        
        # Stage 3  output size = 1/2
        self.cb3a = ConvBlock(3, stage3, stage=3, block='a', activation=activation)
        self.ib3b = IdentityBlock(3, stage3, stage=3, block='b', activation=activation)
        self.ib3c = IdentityBlock(3, stage3, stage=3, block='c', activation=activation)
        self.ib3d = IdentityBlock(3, stage3, stage=3, block='d', activation=activation)
        
        # Stage 4  output size = 1/2
        self.cb4a = ConvBlock(3, stage4, stage=4, block='a', activation=activation)
        block_count = {"resnet50": 5, "resnet101": 22}[architecture]
        self.ib4_list = []
        for i in range(block_count):
            self.ib4_list.append(
                IdentityBlock(3, stage4, stage=4, block=chr(98 + i), activation=activation)
            )
            
        # Stage 5  output size = 1/2
        if self.stage5:
            self.cb5a = ConvBlock(3, stage5, stage=5, block='a', activation=activation)
            self.ib5b = IdentityBlock(3, stage5, stage=5, block='b', activation=activation)
            self.ib5c = IdentityBlock(3, stage5, stage=5, block='c', activation=activation)

    def call(self, input_image, training=False):
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
            if self.return_all_stage:
                return [C1, C2, C3, C4, C5]
            else:
                return C5
                
        else:
            if self.return_all_stage:
                return [C1, C2, C3, C4]
            else:
                return C4