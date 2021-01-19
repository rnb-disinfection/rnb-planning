from .table_interface import *
from ...tmp_framework import *
from ...sampler.handle_a_star import *

class PlanListTable(TableInterface):
    HEADS = [IDENTIFY_COL, "Node", "Parent", "Depth", "Cost", "EDepth", "Trajectory"]
    HILIGHT_KEY = 'plan'
    CUSTOM_BUTTONS = ['Execute', 'Replay']

    def get_items(self):
        sampler = self.graph.sampler
        snode_list = [tem[1] for tem in sorted(sampler.snode_dict.items(), key=lambda x: x[0])] \
            if hasattr(sampler, "snode_dict") else []
        for snode in snode_list:
            snode.traj_tot = snode.traj_length + (snode_list[snode.parents[-1]].traj_tot if snode.parents else 0)
        return snode_list

    def get_items_dict(self):
        return self.graph.sampler.snode_dict if hasattr(self.graph.sampler,"snode_dict") else {}

    def serialize(self, gtem):
        return [str(gtem.idx), str(gtem.state.node), gtem.parents[-1] if gtem.parents else "None",
                gtem.depth, gtem.edepth-gtem.depth, gtem.edepth, "%.2f"%(gtem.traj_tot)]

    def select(self, selected_row_ids, active_row, active_col):
        self.selected_row_ids = selected_row_ids

    def add_item(self, value):
        raise(RuntimeError("Cannot add or delete plan"))

    def delete_item(self, active_row):
        raise(RuntimeError("Cannot add or delete plan"))

    def update_item(self, atem, active_col, value):
        res, msg = False, "cannot change plan content"
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            if args[0]:
                if self.selected_row_ids:
                    selected = int(self.selected_row_ids[0])
                    graph = self.graph
                    sampler = graph.sampler
                    snode_selected = sampler.snode_dict[selected]
                    schedule = snode_selected.parents + [snode_selected.idx]
                    planner = graph.planner
                    snode_schedule = sampler.idxSchedule2SnodeScedule(schedule, self.graph.combined_robot.home_pose)
                    planner.update(graph)

                    with DynamicDetector(planner.online_names, graph.cam.aruco_map, graph.cam.dictionary, graph.cam.rs_config,
                                         graph.cam.T_c12, graph.cam.ref_tuple[1]) as dynamic_detector, \
                            RvizPublisher(graph, planner.online_names) as rviz_pub:
                        e_sim = graph.execute_schedule_online(snode_schedule, planner, control_freq=DEFAULT_TRAJ_FREQUENCY,
                                                              playback_rate=0.5,
                                                              vel_conv=0, err_conv=1e-3, T_step=100,
                                                              on_rviz=not all([v is not None for v in graph.combined_robot.robot_dict.values()]),
                                                              dynamic_detector=dynamic_detector, rviz_pub=rviz_pub,
                                                              obs_K="40")
                else:
                    print("===================================================")
                    print("============= PLAN NOT SELECTED ===================")
                    print("===================================================")
            elif args[1]:
                if self.selected_row_ids:
                    selected = int(self.selected_row_ids[0])
                    graph = self.graph
                    snode_selected = graph.sampler.snode_dict[selected]
                    schedule = snode_selected.parents + [snode_selected.idx]
                    N_fullstep = 500
                    dt_sim = 0.04
                    initial_state = graph.sampler.snode_dict[0].state
                    graph.set_object_state(initial_state)
                    graph.show_pose(initial_state.Q)
                    timer.sleep(0.1)
                    e = graph.replay(schedule, N=N_fullstep, dt=dt_sim, vel_conv=0.5e-2, err_conv=1e-3, error_skip=0)
                else:
                    print("===================================================")
                    print("============= PLAN NOT SELECTED ===================")
                    print("===================================================")
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")