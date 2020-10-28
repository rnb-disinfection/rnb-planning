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