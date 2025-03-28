from .table_interface import *
from ...geometry.builder.scene_builder import *
from ...controller.trajectory_client.trajectory_client import DEFAULT_TRAJ_FREQUENCY


class PlanListTable(TableInterface):
    HEADS = [IDENTIFY_COL, "Node", "Parent", "Depth", "Trajectory"]
    HILIGHT_KEY = 'plan'
    CUSTOM_BUTTONS = ['Execute', 'Replay']

    def get_items(self):
        planning_pipeline = self.planning_pipeline
        if hasattr(planning_pipeline, "tplan") and hasattr(planning_pipeline.tplan, "snode_dict"):
            snode_list = [tem[1] for tem in sorted(planning_pipeline.tplan.snode_dict.items(), key=lambda x: x[0])]
        else:
            snode_list = []
        for snode in snode_list:
            snode.traj_tot = snode.traj_length + (snode_list[snode.parents[-1]].traj_tot if snode.parents else 0)
        return snode_list

    def get_items_dict(self):
        return self.planning_pipeline.tplan.snode_dict if hasattr(self.planning_pipeline,"snode_dict") else {}

    def serialize(self, gtem):
        return [str(gtem.idx), str(gtem.state.node), gtem.parents[-1] if gtem.parents else "None",
                gtem.depth, "%.2f"%(gtem.traj_tot)]

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
                    snode_selected = planning_pipeline.tplan.snode_dict[selected]
                    schedule = snode_selected.parents + [snode_selected.idx]
                    planner = planning_pipeline.mplan
                    snode_schedule = planning_pipeline.tplan.idxSchedule2SnodeScedule(schedule)
                    planner.update_gscene()

                    planning_pipeline.execute_schedule(snode_schedule, vel_scale=0.5, acc_scale=0.5)

                    # with DynamicDetector(self.s_builder, self.s_builder.detector.get_targets_of_levels([DetectionLevel.ONLINE])) as dynamic_detector, \
                    #         RvizPublisher(planning_pipeline.gscene, planner.online_names) as rviz_pub:
                    #     e_sim = planning_pipeline.execute_schedule_online(snode_schedule, planner, control_freq=DEFAULT_TRAJ_FREQUENCY,
                    #                                           playback_rate=0.5,
                    #                                           vel_conv=0, err_conv=1e-3, T_step=100,
                    #                                           on_rviz=not all([v is not None for v in planning_pipeline.pscene.combined_robot.robot_dict.values()]),
                    #                                           dynamic_detector=dynamic_detector, rviz_pub=rviz_pub,
                    #                                           obs_K="40")
                else:
                    print("===================================================")
                    print("============= PLAN NOT SELECTED ===================")
                    print("===================================================")
            elif args[1]:
                if self.selected_row_ids:
                    selected = int(self.selected_row_ids[0])
                    planning_pipeline = self.planning_pipeline
                    snode_selected = planning_pipeline.tplan.snode_dict[selected]
                    schedule = snode_selected.parents + [snode_selected.idx]
                    initial_state = planning_pipeline.tplan.snode_dict[0].state
                    planning_pipeline.pscene.set_object_state(initial_state)
                    planning_pipeline.pscene.gscene.show_pose(initial_state.Q)
                    time.sleep(0.1)
                    snode_schedule = planning_pipeline.tplan.idxSchedule2SnodeScedule(schedule)
                    planning_pipeline.play_schedule(snode_schedule, period=0.01)
                else:
                    print("===================================================")
                    print("============= PLAN NOT SELECTED ===================")
                    print("===================================================")
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")