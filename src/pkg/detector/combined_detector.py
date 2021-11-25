from .detector_interface import DetectorInterface

##
# @class CombinedDetector
# @brief detector wrapper class to combine multiple detectors.
class CombinedDetector(DetectorInterface):
    ##
    # @param aruco_map   ArucoMap dictionary instance
    # @param detector_list list of subclass instances of DetectorInterface. results of later detector overwrites previous one.
    def __init__(self, detector_list):
        self.detector_list = detector_list

    ##
    # @brief initialize cameras
    def initialize(self):
        for detector in self.detector_list:
            detector.initialize()

    ##
    # @brief disconnect cameras
    def disconnect(self):
        for detector in self.detector_list:
            detector.disconnect()

    ##
    # @brief calibrate stereo offset
    # @return config_ref config of ref cam
    # @return config_sub config of sub cam
    # @return T_c12      Transformtation from sub cam to ref cam
    def calibrate(self, N_trial=5):
        for detector in self.detector_list:
            detector.calibrate(N_trial=N_trial)

    ##
    # @brief detect objects. results of later detector overwrites previous one.
    # @param    name_mask   object names to detect
    # @param    level_mask  list of rnb-planning.src.pkg.detector.detector_interface.DetectionLevel
    # @param    visualize   visualize the detection result
    # @return object_pose_dict dictionary for object transformations
    def detect(self, name_mask=None, level_mask=None, visualize=False):
        objectPose_dict = {}
        for detector in self.detector_list:
            objectPose_dict.update(detector.detect(name_mask=name_mask, level_mask=level_mask, visualize=visualize))
        return objectPose_dict

    ##
    # @brief    list registered targets of specific detection level
    # @param    detection_level list of target detection levels
    # @return   names target names
    def get_targets_of_levels(self, detection_levels=None):
        targets = []
        for detector in self.detector_list:
            targets += detector.target(detection_levels=detection_levels)
        return sorted(set(targets))

    ##
    # @brief    Acquire geometry kwargs of item
    # @param    name    item name
    # @return   kwargs  kwargs
    def get_geometry_kwargs(self, name):
        gkwargs = {}
        for detector in self.detector_list:
            gkwargs.update(detector.get_geometry_kwargs(name=name))
        return gkwargs

    ##
    # @brief    add axis marker to GeometryHandle
    def add_item_axis(self, gscene, hl_key, item, axis_name=None):
        for detector in self.detector_list:
            detector.add_item_axis(gscene, hl_key, item, axis_name=axis_name)