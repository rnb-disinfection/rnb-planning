from .table_interface import *
from ...environment_builder import *
from ...constraint_graph import *

class TaskPlanTable(TableInterface):
    HEADS = [IDENTIFY_COL]
    HILIGHT_KEY = 'task'
    CUSTOM_BUTTONS = ['Replay', 'Plan', 'Initialize', 'BuildGraph']

    def get_items(self):
        return [[k] for k in sorted(self.graph.task_plan_candi.keys())]

    def get_items_dict(self):
        return {item[0]: item for item in self.get_items()}

    def serialize(self, gtem):
        return gtem

    def highlight_item(self, gtem, color=None):
        if color == (0.3, 0.3, 1, 0.5):
            self.graph.task_plan_fun = self.graph.task_plan_candi[gtem[0]]

    def add_item(self, value):
        raise(RuntimeError("Cannot add or delete planner"))

    def delete_item(self, active_row):
        raise(RuntimeError("Cannot add or delete planner"))

    def update_item(self, atem, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, "cannot change planner name"
        return res, msg

    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.CUSTOM:
            if args[0]:
                graph = self.graph
                schedule_dict = graph.find_schedules()
                schedule_sorted = graph.sort_schedule(schedule_dict)
                N_fullstep = 500
                dt_sim = 0.04
                # dt_sim = 0.04
                for schedule in schedule_sorted:
                    print(schedule)
                for schedule, i_s in zip(schedule_sorted[:], range(len(schedule_sorted))):
                    traj, end_state, error, success = graph.test_transition(
                        graph.snode_dict[0].state, graph.snode_dict[0].state,
                        N=10, dt=dt_sim, vel_conv=1e-2, err_conv=5e-4, print_expression=False)
                    timer.sleep(0.1)
                    #     try:
                    e = graph.replay(schedule, N=N_fullstep, dt=dt_sim,
                                     vel_conv=1e-3, err_conv=1e-3, error_skip=0)
                #     except Exception as e:
                #         print(e)
            elif args[1]:
                if hasattr(self, 'initial_state') and hasattr(self, 'goal_nodes'):
                    graph = self.graph
                    graph.task_plan_fun(self.initial_state, self.goal_nodes)
                else:
                    res, msg = False, "not initialized"
            elif args[2]:
                graph = self.graph
                OBJECT_DICT = {k: dict(_type=v.__class__) for k, v in graph.object_dict.items()}
                objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv = \
                    detect_objects(graph.cam.aruco_map, graph.cam.dictionary)
                xyz_rvec_mv_dict, put_point_dict, up_point_dict = calc_put_point(
                    objectPose_dict_mv, graph.cam.aruco_map, OBJECT_DICT, graph.cam.ref_tuple)
                update_geometries(objectPose_dict_mv.keys(), objectPose_dict_mv, graph.cam.ref_tuple[1])
                initial_state = State(tuple([(oname, put_point_dict[oname], 'floor') for oname in graph.object_list]),
                                      {oname: graph.object_dict[oname].object.get_offset_tf() for oname in
                                       graph.object_list},
                                      graph.get_real_robot_pose())
                binding_dict = match_point_binder(graph, initial_state, objectPose_dict_mv)
                self.initial_state = State(
                    tuple([(oname, put_point_dict[oname], binding_dict[oname]) for oname in graph.object_list]),
                    {oname: graph.object_dict[oname].object.get_offset_tf() for oname in graph.object_list},
                    graph.get_real_robot_pose())
                goal_nodes_1 = get_goal_nodes(initial_state.node, "box1", "goal_bd")
                self.goal_nodes = []
                for gnode in goal_nodes_1:
                    self.goal_nodes += get_goal_nodes(gnode, "box2", "floor")
            elif args[3]:
                graph = self.graph
                graph.build_graph()
            else:
                print("Unknown button")
        else:
            TableInterface.button(self, button, *args, **kwargs)