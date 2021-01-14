from ..moveit.moveit_planner import MoveitPlanner
from ..etasl.etasl import etasl_planner
from ..interface import PlannerInterface

class HybridPlanner(PlannerInterface):
    NAME = "Hybrid"

    def __init__(self, joint_names, link_names, urdf_path, urdf_content, robot_names, binder_links, ghnd):
        self.eplan = etasl_planner(ghnd=ghnd, joint_names=joint_names, link_names=link_names, urdf_path=urdf_path)
        self.mplan = MoveitPlanner(ghnd=ghnd, joint_names=joint_names, link_names=link_names, urdf_path=urdf_path,
                                   urdf_content=urdf_content,robot_names=robot_names,binder_links=binder_links)

    def set_object_dict(self, object_dict):
        self.object_dict = object_dict
        self.eplan.set_object_dict(object_dict)
        self.mplan.set_object_dict(object_dict)

    def set_binder_dict(self, binder_dict):
        self.binder_dict = binder_dict
        self.eplan.set_binder_dict(binder_dict)
        self.mplan.set_binder_dict(binder_dict)

    def update_gtems(self):
        self.eplan.update_gtems()
        self.mplan.update_gtems()
        self.online_names = self.eplan.online_names

    def plan_transition(self, from_state, to_state, binding_list, redundancy_dict=None, **kwargs):
        if len(binding_list)!=1:
            raise(RuntimeError("simultaneous tasking not supported"))
        else:
            oname, hname, bname = binding_list[0]
            handle = self.object_dict[oname].action_points_dict[hname]
            binder = self.binder_dict[bname]

            group_name_handle = [gname for gname in self.mplan.planner.group_names if gname in handle.object.link_name]
            group_name_binder = [gname for gname in self.mplan.planner.group_names if gname in binder.object.link_name]
            if group_name_handle and group_name_binder:
                print("===================== plan dual manipulation =====================")
                return self.eplan.plan_transition(from_state, to_state, binding_list,
                                              redundancy_dict=redundancy_dict, **kwargs)
            else:
                return self.mplan.plan_transition(from_state, to_state, binding_list,
                                                  redundancy_dict=redundancy_dict,
                                                  group_name_handle=group_name_handle,
                                                  group_name_binder=group_name_binder, **kwargs)

    def init_online_plan(self, from_state, to_state, binding_list, T_step, control_freq, playback_rate=0.5, **kwargs):
        self.eplan.init_online_plan(from_state, to_state, binding_list, T_step, control_freq, playback_rate=0.5, **kwargs)

    def step_online_plan(self, i_q, pos, wp_action=False):
        self.eplan.step_online_plan(i_q, pos, wp_action)

    def update_online(self, obsPos_dict):
        self.eplan.update_online(obsPos_dict)

    def update_target_joint(self, idx_cur, traj, joint_cur):
        self.eplan.update_online(idx_cur, traj, joint_cur)
