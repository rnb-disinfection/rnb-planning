from abc import *
__metaclass__ = type


##
# @class    MotionFilterInterface
# @brief    Base class for motion planning filters
class MotionFilterInterface:

    BEFORE_IK = False

    ##
    # @brief (prototype) check feasibility
    # @param actor  rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    # @param obj    rnb-planning.src.pkg.planning.constraint.constraint_subject.Subject
    # @param handle rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint
    # @param redundancy_values calculated redundancy values in dictionary format {(object name, point name): (xyz, rpy)}
    # @param Q_dict joint configuration in dictionary format {joint name: radian value}
    # @return True if feasible, else False
    @abstractmethod
    def check(self, actor, obj, handle, redundancy_values, Q_dict, **kwargs):
        point_add_handle, rpy_add_handle = redundancy_values[(obj.oname, handle.name)]
        point_add_actor, rpy_add_actor = redundancy_values[(obj.oname, actor.name)]
        T_handle_lh = np.matmul(handle.Toff_lh, SE3(Rot_rpy(rpy_add_handle), point_add_handle))
        T_actor_lh = np.matmul(actor.Toff_lh, SE3(Rot_rpy(rpy_add_actor), point_add_actor))
        T_loal = np.matmul(T_handle_lh, SE3_inv(T_actor_lh))

        return self.check_T_loal(actor, obj, T_loal, Q_dict, **kwargs)

    ##
    # @brief define lock if it needs lock in multiprocess calls
    def prepare_multiprocess_lock(self, manager):
        pass