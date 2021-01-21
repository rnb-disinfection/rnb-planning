from .table_interface import *
from ...planning.task.handle_a_star import *
from ...planning.task.object_a_star import *


class TaskPlanTable(TableInterface):
    HEADS = [IDENTIFY_COL, "MultiProcess"]
    HILIGHT_KEY = 'task'
    CUSTOM_BUTTONS = ['Plan', 'Initialize']
    task_plan_candi = {"ObjectAstar": {"MultiProcess":True},
                       "HandleAstar": {"MultiProcess":True}}
    task_plan_names = ["ObjectAstar", "HandleAstar"]
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
                if gtem[0] == "ObjectAstar":
                    sampler = ObjectAstar(self.planning_pipeline.pscene)
                    print("Set ObjectAstar")
                elif gtem[0] == "HandleAstar":
                    sampler = HandleAstar(self.planning_pipeline.pscene)
                    print("Set HandleAstar")
                else:
                    raise(RuntimeError("Undefined sampler"))
                self.planning_pipeline.pscene.set_sampler(sampler)

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
                    planning_pipeline = self.planning_pipeline
                    self.pl_kwargs["multiprocess"] = self.task_plan_candi[planning_pipeline.tplan.__class__.__name__]["MultiProcess"]
                    planning_pipeline.tplan.search(self.initial_state, self.goal_nodes, **self.pl_kwargs)
                else:
                    print("not initialized")
            elif args[1]:
                planning_pipeline = self.planning_pipeline
                planning_pipeline.tplan.prepare()
                tplan = planning_pipeline.tplan
                self.initial_state = planning_pipeline.pscene.get_state(planning_pipeline.combined_robot.get_real_robot_pose())
                self.goal_nodes = planning_pipeline.pscene.get_goal_nodes(self.initial_state.node, "box1", "goal_bd")
            else:
                print("Unknown button")
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")