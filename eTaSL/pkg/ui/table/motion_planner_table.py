from .table_interface import *
from ...environment_builder import *
from ...planner.etasl.etasl import *
from ...planner.moveit.moveit_planner import *
from ...planner.hybrid.hybrid_planner import *

class MotionPlanTable(TableInterface):
    HEADS = [IDENTIFY_COL]
    HILIGHT_KEY = 'motion'
    CUSTOM_BUTTONS = ['ReadPose']

    def get_items(self):
        return [("HybridPlanner",), ( "MoveIt",), ("eTaSL",)]

    def get_items_dict(self):
        return {item[0]: item for item in self.get_items()}

    def serialize(self, gtem):
        return gtem

    def highlight_item(self, gtem, color=None):
        if color == (0.3, 0.3, 1, 0.5):
            if gtem[0] == "eTaSL":
                graph = self.graph
                eplan = etasl_planner(joint_names = graph.joint_names, link_names = graph.link_names, urdf_path = graph.urdf_path)
                graph.set_planner(eplan)
            if gtem[0] == "MoveIt":
                graph = self.graph
                mplan = MoveitPlanner(joint_names=graph.joint_names, link_names=graph.link_names,
                                      urdf_path=graph.urdf_path, urdf_content=graph.urdf_content,
                                      robot_names=graph.combined_robot.robot_names,
                                      binder_links=[v.object.link_name for v in graph.binder_dict.values()])
                graph.set_planner(mplan)
            if gtem[0] == "HybridPlanner":
                graph = self.graph
                hplan = HybridPlanner(joint_names=graph.joint_names, link_names=graph.link_names,
                                      urdf_path=graph.urdf_path, urdf_content=graph.urdf_content,
                                      robot_names=graph.combined_robot.robot_names,
                                      binder_links=[v.object.link_name for v in graph.binder_dict.values()])
                graph.set_planner(hplan)

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
            self.graph.show_pose(self.graph.combined_robot.get_real_robot_pose())
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")