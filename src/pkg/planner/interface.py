from abc import *
import numpy as np

__metaclass__ = type

class PlannerInterface:
    NAME = None

    def set_object_dict(self, object_dict):
        self.object_dict = object_dict

    def set_binder_dict(self, binder_dict):
        self.binder_dict = binder_dict

    def update(self, graph):
        self.update_gtems()
        self.set_object_dict(graph.object_dict)
        self.set_binder_dict(graph.binder_dict)

    @abstractmethod
    def update_gtems(self):
        pass

    @abstractmethod
    def plan_transition(self, from_state, to_state, binding_list, **kwargs):
        pass

    @abstractmethod
    def init_online_plan(self, from_state, to_state, binding_list, T_step, control_freq, playback_rate=0.5, **kwargs):
        pass

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