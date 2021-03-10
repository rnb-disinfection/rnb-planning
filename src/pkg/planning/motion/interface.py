from abc import *
import numpy as np
from ...utils.utils import list2dict, GlobalTimer
from ..constraint.constraint_common import calc_redundancy

__metaclass__ = type


##
# @class MotionInterface
# @brief Motion planner class interface
# @remark   Child classes should be implemented with update_gscene and plan_algorithm.
#           Child classes' constructor should call MotionInterface.__init__(self, pscene, motion_filters).
#           Specify NAME = "" for a child class as a class variable
#           To use online planning, additionally implement init_online_algorithm, step_online_plan, update_online and update_target_joint.
class MotionInterface:
    NAME = None

    ##
    # @brief    Basic initiailization of motion planner
    # @remark   Constructors of all child class should call MotionInterface.__init__(self, pscene)
    # @param    pscene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param    motion_filters list of child-class of rnb-planning.src.pkg.planning.motion.filtering.filter_interface.MotionFilterInterface
    def __init__(self, pscene, motion_filters=[], *args, **kwargs):
        ## @brief an instance of planning scene (rnb-planning.src.pkg.planning.scene.PlanningScene)
        self.pscene = pscene
        ## @brief motion_filters list of child-class of rnb-planning.src.pkg.planning.motion.filtering.filter_interface.MotionFilterInterface
        self.motion_filters = motion_filters
        ## @brief geometry scene of the planning scene (rnb-planning.src.pkg.geometry.geometry.GeometryScene)
        self.gscene = pscene.gscene
        ## @brief combined robot in the planning scene (rnb-planning.src.pkg.controller.combined_robot.CombinedRobot)
        self.combined_robot = pscene.combined_robot
        self.joint_names = self.gscene.joint_names
        self.link_names = self.gscene.link_names
        self.urdf_path = self.gscene.urdf_path
        self.urdf_content = self.gscene.urdf_content
        self.joint_num = self.gscene.joint_num
        self.gtimer = GlobalTimer.instance()
        ## @brief log of plan results
        self.result_log = {}
        self.reset_log()

    ##
    # @brief reset log data and set log flag
    # @param flag_log set this value True to log filter results
    def reset_log(self, flag_log=False):
        self.flag_log = flag_log
        self.result_log = {mfilter.__class__.__name__: [] for mfilter in self.motion_filters}
        self.result_log["planning"] = []
        self.gtimer.reset()

    ##
    # @brief (prototype) update changes in geometric scene
    @abstractmethod
    def update_gscene(self):
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
    # @return binding_list  list of binding
    def plan_transition(self, from_state, to_state, redundancy_dict=None, **kwargs):
        if from_state is not None:
            self.pscene.set_object_state(from_state)
        binding_list, success = self.pscene.get_slack_bindings(from_state, to_state)

        redundancy_values = {}

        if success:
            for binding in binding_list:
                obj_name, ap_name, binder_name, binder_geometry_name = binding
                actor, obj = self.pscene.actor_dict[binder_name], self.pscene.subject_dict[obj_name]
                handle = obj.action_points_dict[ap_name]
                if obj_name not in redundancy_dict:
                    print("========== obj_name {} not in redundancy_dict {} -> {} =============".format(obj_name, from_state.node, to_state.node))
                    success = False
                    break
                redundancy, Q_dict = redundancy_dict[obj_name], list2dict(from_state.Q, self.joint_names)
                redundancy_values[(obj_name, handle.name)] = calc_redundancy(redundancy[handle.name], handle)
                redundancy_values[(obj_name, actor.name)] = calc_redundancy(redundancy[actor.name], actor)

                for mfilter in self.motion_filters:
                    if self.flag_log:
                        self.gtimer.tic(mfilter.__class__.__name__)
                    success = mfilter.check(actor, obj, handle, redundancy_values, Q_dict)
                    if self.flag_log:
                        self.gtimer.toc(mfilter.__class__.__name__)
                        self.result_log[mfilter.__class__.__name__].append(success)
                    if not success:
                        break

        if success:
            if self.flag_log:
                self.gtimer.tic('planning')
            Traj, LastQ, error, success = self.plan_algorithm(from_state, to_state, binding_list,
                                                              redundancy_values=redundancy_values, **kwargs)
            if self.flag_log:
                self.gtimer.toc('planning')
                self.result_log['planning'].append(success)
        else:
            Traj, LastQ, error, success = [], [], 1e10, False
        return Traj, LastQ, error, success, binding_list

    ##
    # @brief (prototype) planning algorithm implementation for each planner
    # @param from_state     starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state       goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param binding_list   list of bindings to pursue
    # @param redundancy_values calculated redundancy values in dictionary format {(object name, point name): (xyz, rpy)}
    # @return Traj      Full trajectory as array of Q
    # @return LastQ     Last joint configuration as array
    # @return error     planning error
    # @return success   success/failure of planning result
    @abstractmethod
    def plan_algorithm(self, from_state, to_state, binding_list, redundancy_values=None, **kwargs):
        return [], [], 1e10, False

    ##
    # @brief initialize online planning
    # @remark  call step_online_plan to get each step plans
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    def init_online_plan(self, from_state, to_state, T_step, control_freq, playback_rate=0.5,
                         redundancy_dict=None, **kwargs):
        binding_list, success = self.pscene.get_slack_bindings(from_state, to_state)
        redundancy_values = {}
        if success:
            if redundancy_dict is not None:
                for binding in binding_list:
                    obj_name, ap_name, binder_name, binder_geometry_name = binding
                    actor, obj = self.pscene.actor_dict[binder_name], self.pscene.subject_dict[obj_name]
                    handle = obj.action_points_dict[ap_name]
                    redundancy, Q_dict = redundancy_dict[obj_name], list2dict(from_state.Q, self.joint_names)
                    redundancy_values[(obj_name, handle.name)] = calc_redundancy(redundancy[handle.name], handle)
                    redundancy_values[(obj_name, actor.name)] = calc_redundancy(redundancy[actor.name], actor)
            return self.init_online_algorithm(from_state, to_state, binding_list=binding_list,
                                              T_step=T_step, control_freq=control_freq, playback_rate=playback_rate,
                                              redundancy_values=redundancy_values**kwargs)
        else:
            raise(RuntimeError("init online plan fail - get_slack_bindings"))


    ##
    # @brief (prototype) initialize online planning algorithm
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    @abstractmethod
    def init_online_algorithm(self, from_state, to_state, T_step, control_freq, redundancy_values=None,
                              playback_rate=0.5, **kwargs):
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