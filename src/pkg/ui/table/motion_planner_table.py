from .table_interface import *
from ...planning.motion.etasl.etasl_planner import *
from ...planning.motion.moveit.moveit_planner import *

class MotionPlanTable(TableInterface):
    HEADS = [IDENTIFY_COL]
    HILIGHT_KEY = 'motion'
    CUSTOM_BUTTONS = ['ReadPose']

    def get_items(self):
        return [( "MoveIt",), ("eTaSL",)]

    def get_items_dict(self):
        return {item[0]: item for item in self.get_items()}

    def serialize(self, gtem):
        return gtem

    def highlight_item(self, gtem, color=None):
        if color == (0.3, 0.3, 1, 0.5):
            if gtem[0] == "eTaSL":
                planning_pipeline = self.planning_pipeline
                eplan = EtaslPlanner(self.planning_pipeline.pscene)
                planning_pipeline.set_motion_planner(eplan)
            if gtem[0] == "MoveIt":
                planning_pipeline = self.planning_pipeline
                mplan = MoveitPlanner(self.planning_pipeline.pscene)
                planning_pipeline.set_motion_planner(mplan)

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
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            self.planning_pipeline.pscene.gscene.show_pose(
                self.planning_pipeline.pscene.combined_robot.get_real_robot_pose())
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")