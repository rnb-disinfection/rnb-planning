from .table_interface import *
from ...environment_builder import *
from ...planner.etasl.etasl import *

class MotionPlanTable(TableInterface):
    HEADS = [IDENTIFY_COL]
    HILIGHT_KEY = 'motion'
    CUSTOM_BUTTONS = ['ReadPose']

    def get_items(self):
        return [("eTaSL",)]

    def get_items_dict(self):
        return {item[0]: item for item in self.get_items()}

    def serialize(self, gtem):
        return gtem

    def highlight_item(self, gtem, color=None):
        if color == (0.3, 0.3, 1, 0.5):
            if gtem[0] == "eTaSL":
                eplan = etasl_planner(joint_names = self.graph.joint_names, link_names = self.graph.link_names, urdf_path = self.graph.urdf_path)
                self.graph.set_planner(eplan)

    def add_item(self, value):
        raise(RuntimeError("Cannot add or delete planner"))

    def delete_item(self, active_row):
        raise(RuntimeError("Cannot add or delete planner"))

    def update_item(self, atem, active_col, value):
        cbot = self.graph.combined_robot
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, "cannot change planner name"
        return res, msg

    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.CUSTOM:
            self.graph.show_pose(self.graph.get_real_robot_pose())
        else:
            TableInterface.button(self, button, *args, **kwargs)