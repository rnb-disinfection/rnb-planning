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