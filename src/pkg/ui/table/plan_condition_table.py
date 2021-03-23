from .table_interface import *
from ...geometry.builder.scene_builder import *
from ...controller.trajectory_client.trajectory_client import DEFAULT_TRAJ_FREQUENCY
from enum import Enum


class PlanConditionType(Enum):
    INITIAL = 0
    GOAL = 1


class PlanConditionItem:
    def __init__(self, idx, type, node):
        self.idx, self.type, self.node = idx, type, node


def encode_node_from_str(str_node):
    gtem_list = str_node.split(",")
    encoded = []
    for gtem in gtem_list:
        if (gtem.startswith('"') and gtem.endswith('"')) or (gtem.startswith("'") and gtem.endswith("'")):
            encoded.append(gtem[1:-1])
        elif gtem=="" or gtem==" ":
            pass
        else:
            try:
                encoded.append(int(gtem))
            except:
                encoded.append(gtem)
    encoded = tuple(encoded)
    return encoded

class PlanConditionTable(TableInterface):
    HEADS = [IDENTIFY_COL, "Type", "Node"]
    HILIGHT_KEY = 'condition'
    CUSTOM_BUTTONS = ['Plan', 'Initialize']

    def __init__(self, *args, **kwargs):
        self.initial_state = None
        self.goals = []
        TableInterface.__init__(self, *args, **kwargs)

    def get_items(self):
        item_list = []
        if self.initial_state is not None:
            item_list.append(PlanConditionItem(0, PlanConditionType.INITIAL, self.initial_state.node))
        for i_g, goal in enumerate(self.goals):
            item_list.append(PlanConditionItem(i_g+1, PlanConditionType.GOAL, goal))
        return item_list

    def get_items_dict(self):
        items = self.get_items()
        return {str(item.idx): item for item in items}

    def serialize(self, item):
        return [str(item.idx), item.type.name, str(item.node)[1:-1]]

    def select(self, selected_row_ids, active_row, active_col):
        self.selected_row_ids = selected_row_ids

    def add_item(self, value):
        if self.initial_state is None:
            raise(RuntimeError("Initialize first!"))
        if value["Type"] == PlanConditionType.INITIAL.name:
            raise(RuntimeError("Cannot add a INITIAL condition"))
        elif value["Type"] == PlanConditionType.GOAL.name:
            encoded = encode_node_from_str(value["Node"])
            self.goals.append(encoded)
        else:
            raise(RuntimeError("Only GOAL type condition can be added"))

    def delete_item(self, active_row):
        idx = int(active_row)
        if idx == 0:
            raise(RuntimeError("INITIAL condition cannot be removed"))
        else:
            self.goals.pop(idx-1)

    def update_item(self, item, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, IDENTIFY_COL + " is not changeable"
        elif active_col == "Type":
            if item.type == PlanConditionType.INITIAL or value == PlanConditionType.INITIAL:
                res, msg = False, "INITIAL condition is not changeable"
            else:
                item.type = PlanConditionType.GOAL
        elif active_col == "Node":
            encoded = encode_node_from_str(value["Node"])
            item.node = encoded
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            if args[0]:
                if self.initial_state is not None and len(self.goals) > 0:
                    planning_pipeline = self.planning_pipeline
                    planning_pipeline.search(self.initial_state, self.goals, verbose=True,
                                             timeout=3, timeout_constrained=30, multiprocess=True)
                else:
                    print("not initialized")
            elif args[1]:
                self.planning_pipeline.mplan.update_gscene()
                self.planning_pipeline.tplan.prepare()
                self.initial_state = self.planning_pipeline.pscene.update_state(
                    self.planning_pipeline.pscene.combined_robot.home_pose)
            else:
                print("Unknown button")
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")