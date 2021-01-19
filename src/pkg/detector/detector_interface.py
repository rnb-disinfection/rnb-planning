from abc import *


from enum import Enum

##
# @class   DetectionLevel
# @brief   type of interface
class DetectionLevel(Enum):
    ## @brief environment - detect only once
    ENVIRONMENT = 0
    ## @brief robot - detect only when robot config changes
    ROBOT = 1
    ## @brief movable - this can be moved from links to links. detect on all requests
    MOVABLE = 2
    ## @brief online - this is frequently moving, so detected on-line in execution mode
    ONLINE = 3

    @classmethod
    ##
    # @brief check if item is fixed in a link (detection level under ROBOT)
    def is_fixed(cls, item):
        return item.value <= DetectionLevel.ROBOT.value


##
# @class    DetectorInterface
# @brief    interface for detector modules
class DetectorInterface:
    ##
    @abstractmethod
    def __init__(self, *args, **kwargs):
        pass

    ##
    # @brief function prototype initialization
    @abstractmethod
    def initialize(self):
        pass

    ##
    # @brief function prototype for disconnection
    @abstractmethod
    def disconnnect(self):
        pass

    ##
    # @brief function prototype for calibration
    @abstractmethod
    def calibrate(self, N_trial=5):
        pass

    ##
    # @brief    function prototype for detection
    # @param    name_mask object names to detect
    # @return   object_pose_dict dictionary for object transformations
    @abstractmethod
    def detect(self, name_mask=None):
        pass