from .table_interface import *
from ...geometry.builder.scene_builder import *
from ...controller.repeater.repeater import DEFAULT_TRAJ_FREQUENCY


class PlanListTable(TableInterface):
    HEADS = [IDENTIFY_COL, "Node", "Parent", "Depth", "Cost", "EDepth", "Trajectory"]
    HILIGHT_KEY = 'plan'
    CUSTOM_BUTTONS = ['Execute', 'Replay']

    def get_items(self):
        planning_pipeline = self.planning_pipeline
        snode_list = [tem[1] for tem in sorted(planning_pipeline.snode_dict.items(), key=lambda x: x[0])] \
            if hasattr(planning_pipeline, "snode_dict") else []
        for snode in snode_list:
            snode.traj_tot = snode.traj_length + (snode_list[snode.parents[-1]].traj_tot if snode.parents else 0)
        return snode_list

    def get_items_dict(self):
        return self.planning_pipeline.snode_dict if hasattr(self.planning_pipeline,"snode_dict") else {}

    def serialize(self, gtem):
        return [str(gtem.idx), str(gtem.state.binding_state), gtem.parents[-1] if gtem.parents else "None",
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
                    planning_pipeline = self.planning_pipeline
                    snode_selected = planning_pipeline.snode_dict[selected]
                    schedule = snode_selected.parents + [snode_selected.idx]
                    planner = planning_pipeline.mplan
                    snode_schedule = planning_pipeline.idxSchedule2SnodeScedule(schedule, planning_pipeline.combined_robot.home_pose)
                    planner.update_gscene()

                    with DynamicDetector(self.s_builder, self.s_builder.detector.get_targets_of_levels([DetectionLevel.ONLINE])) as dynamic_detector, \
                            RvizPublisher(planning_pipeline.pscene, planner.online_names) as rviz_pub:
                        e_sim = planning_pipeline.execute_schedule_online(snode_schedule, planner, control_freq=DEFAULT_TRAJ_FREQUENCY,
                                                              playback_rate=0.5,
                                                              vel_conv=0, err_conv=1e-3, T_step=100,
                                                              on_rviz=not all([v is not None for v in planning_pipeline.combined_robot.robot_dict.values()]),
                                                              dynamic_detector=dynamic_detector, rviz_pub=rviz_pub,
                                                              obs_K="40")
                else:
                    print("===================================================")
                    print("============= PLAN NOT SELECTED ===================")
                    print("===================================================")
            elif args[1]:
                if self.selected_row_ids:
                    selected = int(self.selected_row_ids[0])
                    planning_pipeline = self.planning_pipeline
                    snode_selected = planning_pipeline.snode_dict[selected]
                    schedule = snode_selected.parents + [snode_selected.idx]
                    initial_state = planning_pipeline.snode_dict[0].state
                    planning_pipeline.pscene.set_object_state(initial_state)
                    planning_pipeline.pscene.gscene.show_pose(initial_state.Q)
                    time.sleep(0.1)
                    snode_schedule = planning_pipeline.idxSchedule2SnodeScedule(schedule)
                    planning_pipeline.play_schedule(snode_schedule, period=0.01)
                else:
                    print("===================================================")
                    print("============= PLAN NOT SELECTED ===================")
                    print("===================================================")
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")