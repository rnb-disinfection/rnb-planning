from abc import *

__metaclass__ = type

class PlannerInterface:
    def set_object_dict(self, object_dict):
        self.object_dict = object_dict

    def set_binder_dict(self, binder_dict):
        self.binder_dict = binder_dict

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
    def step_online_plan(self, i_q, pos):
        pass

    @abstractmethod
    def update_online(self, obsPos_dict):
        pass

    @abstractmethod
    def update_target_joint(self, idx_cur, traj, joint_cur):
        pass