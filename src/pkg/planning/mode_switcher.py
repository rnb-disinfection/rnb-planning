from constraint.constraint_subject import AbstractObject, AbstractTask
from enum import Enum

class ModeSwitcherTemplate:
    def __init__(self, pscene,switch_delay=0.5):
        self.pscene = pscene
        self.crob = pscene.combined_robot
        self.switch_delay = switch_delay

    def init(self, snode_init):
        pass

    ##
    # @return custom switch_state
    def switch_in(self, snode_pre, snode_new):
        switch_state = None
        return switch_state

    def switch_out(self, switch_state, snode_new):
        pass

class CombinedSwitcher(ModeSwitcherTemplate):
    def __init__(self, switch_list):
        self.switch_list = switch_list

    def init(self, snode_init):
        for switch in self.switch_list:
            switch.init(snode_init)

    ##
    # @return switch_state in list
    def switch_in(self, snode_pre, snode_new):
        switch_state = []
        for switch in self.switch_list:
            switch_state.append(switch.switch_in(snode_pre, snode_new))
        return switch_state

    def switch_out(self, switch_state, snode_new):
        for switch, s_state in zip(self.switch_list,switch_state):
            switch.switch_out(s_state, snode_new)

class GraspModeSwitcher(ModeSwitcherTemplate):
    def init(self, initial_state):
        grasp_dict = self.__get_grasp_dict(initial_state)
        self.pscene.combined_robot.grasp(**grasp_dict)

    def switch_in(self, snode_pre, snode_new):
        grasp_dict_pre = self.__get_grasp_dict(snode_pre.state)
        grasp_dict_new = self.__get_grasp_dict(snode_new.state)
        switch_state = {rname: grasp_dict_new[rname] for rname in grasp_dict_new.keys()
                      if grasp_dict_pre[rname] != grasp_dict_new[rname]} # extract changing states only
        return switch_state

    def switch_out(self, switch_state, snode_new):
        self.pscene.combined_robot.grasp(**switch_state)

    ##
    # @brief get changing grasping states
    def __get_grasp_dict(self, state):
        grasp_dict = {rname : False for rname in self.pscene.combined_robot.robot_names}
        for btf in state.binding_state.values():
            oname, bpoint, binder, bgeo = btf.get_chain()
            if isinstance(self.pscene.subject_dict[oname], AbstractObject) \
                    and binder in self.pscene.actor_robot_dict:
                rname = self.pscene.actor_robot_dict[binder]
                if rname is not None:
                    grasp_dict[rname] = True
        return grasp_dict

