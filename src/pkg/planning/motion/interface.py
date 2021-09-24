from abc import *
import numpy as np
from ...utils.utils import list2dict, GlobalTimer, DummyBlock, save_pickle, try_mkdir, TextColors
from ..constraint.constraint_common import BindingTransform
from collections import defaultdict
import time
import os

__metaclass__ = type

MOTION_PATH = os.path.join(os.environ['RNB_PLANNING_DIR'], "data/motion_scenes")
try_mkdir(MOTION_PATH)

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
        self.result_log = defaultdict(list)
        self.reset_log()

    ##
    # @brief reset log data and set log flag
    # @param flag_log set this value True to log filter results
    def reset_log(self, flag_log=False, manager=None):
        self.flag_log = flag_log
        if manager is None:
            self.result_log = defaultdict(list)
            self.log_lock = None
        else:
            self.result_log = manager.dict()
            self.log_lock = manager.Lock()
            for mfilter in self.motion_filters:
                self.result_log[mfilter.__class__.__name__] = []
            self.result_log["planning"] = []


    ##
    # @brief (prototype) update changes in geometric scene
    @abstractmethod
    def update_gscene(self):
        pass

    ##
    # @brief plan transition
    # @param from_state         starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state           goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param show_state         show intermediate state on RVIZ
    # @return Traj      Full trajectory as array of Q
    # @return LastQ     Last joint configuration as array
    # @return error     planning error
    # @return success   success/failure of planning result
    # @param binding_list   list of bindings to pursue [(subject name, handle name, actor name, actor root geometry name)]
    def plan_transition(self, from_state, to_state, verbose=False, test_filters_only=False,
                        show_state=False, **kwargs):
        if from_state is not None:
            self.pscene.set_object_state(from_state)
        subject_list, success = self.pscene.get_changing_subjects(from_state, to_state)

        if success:
            for sname in subject_list:
                btf = to_state.binding_state[sname]
                actor, obj = self.pscene.actor_dict[btf.actor_name], self.pscene.subject_dict[sname]
                handle = obj.action_points_dict[btf.handle_name]

                for i_f, mfilter in enumerate(self.motion_filters):
                    fname = mfilter.__class__.__name__
                    if self.flag_log:
                        self.gtimer.tic(fname)
                    success = mfilter.check(actor, obj, handle, btf, Q_dict)
                    if self.flag_log:
                        self.gtimer.toc(fname)
                        if self.log_lock is not None:
                            with self.log_lock:
                                rlog = self.result_log[fname]
                                rlog.append(success)
                                self.result_log[fname] = rlog
                        else:
                            self.result_log[fname].append(success)
                    if not success:
                        if verbose or show_state:
                            print("Motion Filer Failure: {}".format(fname))
                        if show_state:
                            if from_state.Q is not None:
                                self.gscene.show_pose(from_state.Q)
                            vis_bak = self.gscene.highlight_robot(self.gscene.COLOR_LIST[i_f])
                            time.sleep(0.5)
                            self.gscene.recover_robot(vis_bak)
                        break
            if test_filters_only:
                return success
        else:
            try:
                motion_dat = {}
                motion_dat['from_state'] = from_state
                motion_dat['to_state'] = to_state
                motion_dat['gtem_args'] = self.gscene.get_gtem_args()
                save_pickle(
                    os.path.join(MOTION_PATH,
                                 "{0:08d}.pkl".format(
                                     len(os.listdir(MOTION_PATH)))), motion_dat)
            except Exception as e:
                save_pickle(
                    os.path.join(MOTION_PATH,
                                 "{0:08d}-EXCEPTION.pkl".format(
                                     len(os.listdir(MOTION_PATH)))), str(e))


        if success:
            if self.flag_log:
                self.gtimer.tic('planning')
            Traj, LastQ, error, success = self.plan_algorithm(from_state, to_state, subject_list,
                                                              verbose=verbose, **kwargs)
            if self.flag_log:
                self.gtimer.toc('planning')
                if self.log_lock is not None:
                    with self.log_lock:
                        rlog = self.result_log['planning']
                        rlog.append(success)
                        self.result_log['planning'] = rlog
                else:
                    self.result_log['planning'].append(success)
            if not success:
                if verbose or show_state:
                    print("Motion Plan Failure")
                if show_state:
                    if from_state.Q is not None:
                        self.gscene.show_pose(from_state.Q)
                    vis_bak = self.gscene.highlight_robot(self.gscene.COLOR_LIST[-1])
                    time.sleep(0.5)
                    self.gscene.recover_robot(vis_bak)
        else:
            Traj, LastQ, error, success = [from_state.Q], from_state.Q, 1e10, False
        return Traj, LastQ, error, success, [to_state.binding_state[sname].get_chain() for sname in subject_list]

    ##
    # @brief (prototype) planning algorithm implementation for each planner
    # @param from_state     starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state       goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param subject_list   list of changed subjects
    # @return Traj      Full trajectory as array of Q
    # @return LastQ     Last joint configuration as array
    # @return error     planning error
    # @return success   success/failure of planning result
    @abstractmethod
    def plan_algorithm(self, from_state, to_state, subject_list, verbose=False, **kwargs):
        return [], [], 1e10, False

    ##
    # @brief initialize online planning
    # @remark  call step_online_plan to get each step plans
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    def init_online_plan(self, from_state, to_state, T_step, control_freq, playback_rate=0.5, **kwargs):
        subject_list, success = self.pscene.get_changing_subjects(from_state, to_state)
        btf_dict = {}
        if success:
            return self.init_online_algorithm(from_state, to_state, subject_list=subject_list,
                                              T_step=T_step, control_freq=control_freq, playback_rate=playback_rate,
                                              **kwargs)
        else:
            raise(RuntimeError("init online plan fail - get_changing_subjects"))


    ##
    # @brief (prototype) initialize online planning algorithm
    # @param from_state starting state (rnb-planning.src.pkg.planning.scene.State)
    # @param to_state   goal state (rnb-planning.src.pkg.planning.scene.State)
    # @param subject_list   list of changed subject names
    @abstractmethod
    def init_online_algorithm(self, from_state, to_state, subject_list, T_step, control_freq,
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