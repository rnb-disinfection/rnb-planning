from abc import *
import numpy as np
from ...utils.utils import list2dict

__metaclass__ = type


##
# @class MotionInterface
# @brief Motion planner class interface
# @remark   Child classes should be implemented with update_gscene and plan_algorithm.
#           Child classes' constructor should call MotionInterface.__init__(self, pscene).
#           To use online planning, additionally implement init_online_algorithm, step_online_plan, update_online and update_target_joint.
class MotionInterface:
    NAME = None

    ##
    # @brief    Basic initiailization of motion planner
    # @remark   Constructors of all child class should call MotionInterface.__init__(self, pscene)
    # @param    pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    def __init__(self, pscene, *args, **kwargs):
        ## @brief an instance of planning scene (rnb-planning.src.pkg.planning.scene.PlanningScene)
        self.pscene = pscene
        ## @brief geometry scene of the planning scene (rnb-planning.src.pkg.geometry.geometry.GeometryScene)
        self.gscene = pscene.gscene
        ## @brief combined robot in the planning scene (rnb-planning.src.pkg.controller.combined_robot.CombinedRobot)
        self.combined_robot = pscene.combined_robot
        self.joint_names = self.gscene.joint_names
        self.link_names = self.gscene.link_names
        self.urdf_path = self.gscene.urdf_path
        self.urdf_content = self.gscene.urdf_content
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
        binding_list, success = self.pscene.get_slack_bindings(from_state, to_state)

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
    @abstractmethod
    def plan_algorithm(self, from_state, to_state, binding_list, redundancy_dict=None, **kwargs):
        return [], [], 1e10, False

    ##
    # @brief initialize online planning
    # @remark  call step_online_plan to get each step plans
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    def init_online_plan(self, from_state, to_state, T_step, control_freq, playback_rate=0.5, **kwargs):
        binding_list, success = self.pscene.get_slack_bindings(from_state, to_state)
        return self.init_online_algorithm(from_state, to_state, binding_list=binding_list,
                                          T_step=T_step, control_freq=control_freq, playback_rate=playback_rate, **kwargs)


    ##
    # @brief (prototype) initialize online planning algorithm
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    @abstractmethod
    def init_online_algorithm(self, from_state, to_state, T_step, control_freq, playback_rate=0.5, **kwargs):
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