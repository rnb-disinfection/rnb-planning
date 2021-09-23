from abc import *
__metaclass__ = type


##
# @class    MotionFilterInterface
# @brief    Base class for motion planning filters
class MotionFilterInterface:

    BEFORE_IK = False

    ##
    # @brief check feasibility by calling check_T_loal defined in each class
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param btf    BindingTransorm instance
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @return True if feasible, else False
    def check(self, actor, obj, handle, btf, Q_dict, **kwargs):
        raise(NotImplementedError("check is not implemented for {}".format(self.__class__.__name__)))

    ##
    # @brief define lock if it needs lock in multiprocess calls
    def prepare_multiprocess_lock(self, manager):
        pass

import os
from copy import deepcopy

SCENE_PATH = os.path.join(os.environ['RNB_PLANNING_DIR'], "data/checker_scenes")
try_mkdir(SCENE_PATH)

def save_scene(cname, pscene, actor, obj, handle, btf, Q_dict, error_state, result, **kwargs):
    path_dir = os.path.join(SCENE_PATH, cname)
    try_mkdir(path_dir)
    query_args = {}
    query_args["scene_args"] = pscene.get_scene_args()
    query_args["actor"] = actor.name
    query_args["obj"] = obj.oname
    query_args["handle"] = handle.name
    query_args["btf"] = btf
    query_args["Q_dict"] = Q_dict
    query_args["kwargs"] = kwargs
    query_args["error_state"] = error_state
    query_args["result"] = result

    save_pickle(
        os.path.join(path_dir,
                     "{0:08d}-{1}.pkl".format(
                         len(os.listdir(SCENE_PATH)),
                         "ERROR" if error_state else "OK"
                     )), query_args)