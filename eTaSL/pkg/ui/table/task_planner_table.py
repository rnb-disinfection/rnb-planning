from .table_interface import *
from ...environment_builder import *
from ...constraint_graph import *
from ...sampler.a_star.a_star import *



class TaskPlanTable(TableInterface):
    HEADS = [IDENTIFY_COL]
    HILIGHT_KEY = 'task'
    CUSTOM_BUTTONS = ['Execute', 'Replay', 'Plan', 'Initialize', 'BuildGraph']
    task_plan_candi = {"Astar_single": None, "Astar_multi": None}
    task_plan_fun = None

    def get_items(self):
        return [[k] for k in sorted(self.task_plan_candi.keys())]

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
                self.pl_kwargs = dict(tree_margin=2, depth_margin=2, joint_motion_num=10, terminate_on_first=True,
                                      N_search=100, N_loop=1000, display=False, dt_vis=dt_sim / 4,
                                      verbose=True, print_expression=False, error_skip=0, traj_count=3,
                                      ** dict(N=N_fullstep, dt=dt_sim, vel_conv=0.5e-2, err_conv=1e-3))
                if gtem[0] == "Astar_single":
                    sampler = AStarSampler(self.graph)
                    self.pl_kwargs["multiprocess"] = False
                elif gtem[0] == "Astar_multi":
                    sampler = AStarSampler(self.graph)
                    self.pl_kwargs["multiprocess"] = True
                self.graph.set_sampler(sampler)

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
                sampler = graph.sampler
                planner = graph.planner
                schedule_dict = sampler.find_schedules()
                schedule_sorted = sampler.sort_schedule(schedule_dict)
                schedule = schedule_sorted[0]
                snode_schedule = sampler.idxSchedule2SnodeScedule(schedule, graph.combined_robot.get_real_robot_pose())
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
            elif args[1]:
                graph = self.graph
                schedule_dict = graph.sampler.find_schedules()
                schedule_sorted = graph.sampler.sort_schedule(schedule_dict)
                N_fullstep = 500
                dt_sim = 0.04
                # dt_sim = 0.04
                for schedule in schedule_sorted:
                    print(schedule)
                for schedule, i_s in zip(schedule_sorted[:], range(len(schedule_sorted))):
                    traj, end_state, error, success = graph.test_transition(
                        graph.sampler.snode_dict[0].state, graph.sampler.snode_dict[0].state,
                        N=10, dt=dt_sim, vel_conv=1e-2, err_conv=5e-4, print_expression=False)
                    timer.sleep(0.1)
                    e = graph.replay(schedule, N=N_fullstep, dt=dt_sim,
                                     vel_conv=1e-3, err_conv=1e-3, error_skip=0)
            elif args[2]:
                if hasattr(self, 'initial_state') and hasattr(self, 'goal_nodes'):
                    graph = self.graph
                    graph.sampler.search_graph(self.initial_state, self.goal_nodes, **self.pl_kwargs)
                else:
                    print("not initialized")
            elif args[3]:
                graph = self.graph
                sampler = graph.sampler
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
            elif args[4]:
                graph = self.graph
                graph.sampler.build_graph()
            else:
                print("Unknown button")
        else:
            TableInterface.button(self, button, *args, **kwargs)