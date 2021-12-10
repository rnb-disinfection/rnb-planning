from abc import *

##
# @class   CameraInterface
# @brief   interface for camera type sensors
class CameraInterface:
    ##
    # @brief   function prototype for camera initialization
    @abstractmethod
    def initialize(self):
        pass

    ##
    # @brief   function prototype to get camera configuration
    # @return  cameraMatrix 3x3 camera matrix in pixel units,
    # @return  distCoeffs distortion coefficients, 5~14 float array
    # @return  depth_scale scaling constant to be multiplied to depthmap(int) to get m scale values
    @abstractmethod
    def get_config(self):
        pass

    ##
    # @brief   function prototype to disconnect camera
    @abstractmethod
    def disconnect(self):
        pass

    ##
    # @brief   function prototype to get RGB image
    @abstractmethod
    def get_image(self):
        pass

    ##
    # @brief   function prototype to get depthmap
    @abstractmethod
    def get_depthmap(self):
        pass

    ##
    # @brief   function prototype to get RGB image and depthmap
    @abstractmethod
    def get_image_depthmap(self):
        pass

