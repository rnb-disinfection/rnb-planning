from abc import *
import numpy as np
from ...utils.utils import list2dict

__metaclass__ = type


##
# @class MotionInterface
# @brief Motion planner class interface
class MotionInterface:
    NAME = None

    ##
    # @brief    Basic initiailization of motion planner
    # @remark   Constructors of all child class should call MotionInterface.__init__(self, pscene)
    # @param    pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    def __init__(self, pscene, *args, **kwargs):
        ##
        # @brief pscene rnb-planning.src.pkg.planning.scene.PlanningScene
        self.pscene = pscene
        self.gscene = pscene.gscene
        self.combined_robot = pscene.combined_robot
        self.joint_names, self.link_names, self.urdf_path, self.urdf_content\
            = self.gscene.joint_names, self.gscene.link_names, self.gscene.urdf_path, self.gscene.urdf_content
        self.joint_num = self.gscene.joint_num

    ##
    # @brief (prototype) update changes in geometric scene
    @abstractmethod
    def update_gcene(self):
        pass

    ##
    # @brief plan transition
    # @param from_state         starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state           goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param redundancy_dict    redundancy in dictionary format {object name: {axis: value}}
    # @return Traj      Full trajectory as array of Q
    # @return LastQ     Last joint configuration as array
    # @return error     planning error
    # @return success   success/failure of planning result
    def plan_transition(self, from_state, to_state, redundancy_dict=None, **kwargs):
        binding_list = self.pscene.get_slack_bindings(from_state, to_state)

        success = True
        for binding in binding_list:
            if not self.pscene.binder_dict[binding[2]].check_available(
                    list2dict(from_state.Q, self.pscene.gscene.joint_names)):
                success = False
        if success:
            Traj, LastQ, error, success = self.plan_algorithm(from_state, to_state, binding_list,
                                                              redundancy_dict=redundancy_dict, **kwargs)
        else:
            Traj, LastQ, error, success = [], [], 1e10, False
        return Traj, LastQ, error, success

    ##
    # @brief (prototype) planning algorithm implementation for each planner
    # @param from_state     starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state       goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param binding_list   list of bindings to pursue
    # @param redundancy_dict    redundancy in dictionary format {object name: {axis: value}}
    # @return Traj      Full trajectory as array of Q
    # @return LastQ     Last joint configuration as array
    # @return error     planning error
    # @return success   success/failure of planning result
    def plan_algorithm(self, from_state, to_state, binding_list, redundancy_dict=None, **kwargs):
        return [], [], 1e10, False

    ##
    # @brief (prototype) initialize online planning
    # @remark  call step_online_plan to get each step plans
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    @abstractmethod
    def init_online_plan(self, from_state, to_state, binding_list, T_step, control_freq, playback_rate=0.5, **kwargs):
        pass

    ##
    # @brief get step plan
    # @param i_q current step count from init_online_plan
    # @param pos previous joint configuration in dictionary format
    @abstractmethod
    def step_online_plan(self, i_q, pos, wp_action=False):
        pass

    @abstractmethod
    def update_online(self, obsPos_dict):
        pass

    @abstractmethod
    def update_target_joint(self, idx_cur, traj, joint_cur):
        pass

def downample_traj(traj_full, traj_count):
    traj_step = max(1, len(traj_full) / traj_count)
    return np.array(list(reversed(traj_full[::-traj_step])))