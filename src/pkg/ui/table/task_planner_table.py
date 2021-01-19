from .table_interface import *
from ...tmp_framework import *
from ...sampler.handle_a_star import *
from ...sampler.object_a_star import *


class TaskPlanTable(TableInterface):
    HEADS = [IDENTIFY_COL, "MultiProcess"]
    HILIGHT_KEY = 'task'
    CUSTOM_BUTTONS = ['Plan', 'Initialize']
    task_plan_candi = {"ObjectAstarSampler": {"MultiProcess":True},
                       "HandleAstarSampler": {"MultiProcess":True}}
    task_plan_names = ["ObjectAstarSampler", "HandleAstarSampler"]
    task_plan_fun = None

    def get_items(self):
        return [[k, self.task_plan_candi[k]['MultiProcess']] for k in self.task_plan_names]

    def get_items_dict(self):
        return {item[0]: item for item in self.get_items()}

    def serialize(self, gtem):
        return gtem

    def highlight_item(self, gtem, color=None):
        if color == (0.3, 0.3, 1, 0.5):
            self.pl_kwargs = {}
            if "Astar" in gtem[0]:
                dt_sim = 0.04
                T_step = 10
                N_fullstep = int(T_step / dt_sim)
                self.pl_kwargs = dict(tree_margin=2, depth_margin=2, terminate_on_first=True,
                                      N_search=100000, display=True, dt_vis=dt_sim / 4,
                                      verbose=True, print_expression=False, error_skip=0,
                                      ** dict(N=N_fullstep, dt=dt_sim, vel_conv=0.5e-2, err_conv=1e-3))
                if gtem[0] == "ObjectAstarSampler":
                    sampler = ObjectAstarSampler(self.graph)
                    print("Set ObjectAstarSampler")
                elif gtem[0] == "HandleAstarSampler":
                    sampler = HandleAstarSampler(self.graph)
                    print("Set HandleAstarSampler")
                else:
                    raise(RuntimeError("Undefined sampler"))
                self.graph.set_sampler(sampler)

    def add_item(self, value):
        raise(RuntimeError("Cannot add or delete planner"))

    def delete_item(self, active_row):
        raise(RuntimeError("Cannot add or delete planner"))

    def update_item(self, atem, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, "cannot change planner name"
        elif active_col == "MultiProcess":
            self.task_plan_candi[atem[0]]["MultiProcess"] = value.lower() in ["true", "t"]
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            if args[0]:
                if hasattr(self, 'initial_state') and hasattr(self, 'goal_nodes'):
                    graph = self.graph
                    self.pl_kwargs["multiprocess"] = self.task_plan_candi[graph.sampler.__class__.__name__]["MultiProcess"]
                    graph.sampler.search_graph(self.initial_state, self.goal_nodes, **self.pl_kwargs)
                else:
                    print("not initialized")
            elif args[1]:
                graph = self.graph
                graph.sampler.build_graph()
                sampler = graph.sampler
                OBJECT_DICT = {k: dict(_type=v.__class__) for k, v in graph.object_dict.items()}
                objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv = \
                    detect_objects(graph.cam.aruco_map, graph.cam.dictionary)
                xyz_rvec_mv_dict, put_point_dict, up_point_dict = calc_put_point(graph.ghnd,
                    objectPose_dict_mv, graph.cam.aruco_map, OBJECT_DICT, graph.cam.ref_tuple)
                update_geometries(graph.ghnd, objectPose_dict_mv.keys(), objectPose_dict_mv, graph.cam.ref_tuple[1])
                initial_state = State(tuple([(oname, put_point_dict[oname], 'floor') for oname in graph.object_list]),
                                      {oname: graph.object_dict[oname].object.Toff for oname in
                                       graph.object_list},
                                      graph.get_real_robot_pose(), graph)
                binding_dict = match_point_binder(graph, initial_state, objectPose_dict_mv)
                self.initial_state = State(
                    tuple([(oname, put_point_dict[oname], binding_dict[oname]) for oname in graph.object_list]),
                    {oname: graph.object_dict[oname].object.Toff for oname in graph.object_list},
                    graph.get_real_robot_pose(), graph)
                goal_nodes_1 = get_goal_nodes(initial_state.node, "box1", "goal_bd")
                self.goal_nodes = []
                for gnode in goal_nodes_1:
                    self.goal_nodes += get_goal_nodes(gnode, "box2", "floor")
            else:
                print("Unknown button")
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")