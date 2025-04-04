from abc import *
__metaclass__ = type


##
# @class    MotionFilterInterface
# @brief    Base class for motion planning filters
class MotionFilterInterface:

    BEFORE_IK = False

    ##
    # @brief check feasibility by calling check_T_loal defined in each class
    # @param btf    BindingTransorm instance
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @return True if feasible, else False
    def check(self, btf, Q_dict, **kwargs):
        raise(NotImplementedError("check is not implemented for {}".format(self.__class__.__name__)))

    ##
    # @brief define lock if it needs lock in multiprocess calls
    def prepare_multiprocess_lock(self, manager):
        pass

import os
import sys
from copy import deepcopy
from ...utils.utils import *
from ..state import State

SCENE_PATH = os.path.join(os.environ['RNB_PLANNING_DIR'], "data/checker_scenes")
try_mkdir(SCENE_PATH)

def save_scene(cname, pscene, btf, Q_dict, error_state, result, **kwargs):
    path_dir = os.path.join(SCENE_PATH, cname)
    try_mkdir(path_dir)
    scene_data = {}
    scene_data["scene_args"] = pscene.get_scene_args(list2dict(Q_dict, pscene.gscene.joint_names))

    binding_state, state_param = pscene.get_object_state()
    state = State(binding_state, state_param, dict2list(Q_dict, pscene.gscene.joint_names), pscene)
    scene_data["state"] = state

    scene_data["btf"] = btf
    scene_data["kwargs"] = kwargs
    scene_data["error_state"] = error_state
    scene_data["result"] = result

    save_pickle(
        os.path.join(path_dir,
                     "{0:08d}-{1}.pkl".format(
                         len(os.listdir(path_dir)),
                         "ERROR" if error_state else "OK"
                     )), scene_data)

def load_unpack_scene_args(pscene, scene_data):
    pscene_args = scene_data["scene_args"]
    pscene.recover_scene_args(pscene_args)
    state = scene_data["state"]
    pscene.set_object_state(state)
    pscene.gscene.update_markers_all()
    pscene.gscene.show_pose(state.Q)
    Q_dict = list2dict(state.Q, pscene.gscene.joint_names)

    btf = scene_data["btf"]
    kwargs = scene_data["kwargs"]
    if "ignore" in kwargs:
        kwargs["ignore"] = [pscene.gscene.NAME_DICT[ig_name] for ig_name in kwargs["ignore"]]

    error_state = scene_data["error_state"]
    result = scene_data["result"]
    return state, btf, Q_dict, kwargs, error_state, result