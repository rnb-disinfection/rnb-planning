from .table_interface import *
from ...environment_builder import *

class RobotTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'RType', 'Position', 'Direction']
    HILIGHT_KEY = 'marker_group'
    CUSTOM_BUTTONS = ["Apply", "Detect"]

    def get_items(self):
        cbot = self.graph.combined_robot
        return [(rbt[0], rbt[1].name,
                 round_it_str(cbot.xyz_rpy_robots[rbt[0]][0], 4),
                 round_it_str(cbot.xyz_rpy_robots[rbt[0]][1], 4)) for rbt in cbot.robots_on_scene]

    def get_items_dict(self):
        return {item[0]: item for item in self.get_items()}

    def serialize(self, gtem):
        return gtem

    def highlight_item(self, gtem, color=None):
        pass

    def add_item(self, value):
        raise(RuntimeError("Cannot add or delete robot"))

    def delete_item(self, active_row):
        raise(RuntimeError("Cannot add or delete robot"))

    def update_item(self, atem, active_col, value):
        cbot = self.graph.combined_robot
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, "cannot change robot name"
        elif active_col == 'RType':
            res, msg = False, "cannot change robot type"
        elif active_col == 'Position':
            name = atem[0]
            xyz_rpy_prev = cbot.xyz_rpy_robots[name]
            cbot.xyz_rpy_robots[name] = (str_num_it(value), xyz_rpy_prev[1])
        elif active_col == 'Direction':
            name = atem[0]
            xyz_rpy_prev = cbot.xyz_rpy_robots[name]
            cbot.xyz_rpy_robots[name] = (xyz_rpy_prev[0], str_num_it(value))
        return res, msg

    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.CUSTOM:
            if args[0]:
                graph = self.graph
                cbot = self.graph.combined_robot
                xcustom, JOINT_NAMES, LINK_NAMES, urdf_content = set_custom_robots(cbot.robots_on_scene, cbot.xyz_rpy_robots, cbot.joint_names)
                graph.clear_markers()
                graph.clear_highlight()
                graph.ghnd.clear()
                time.sleep(1)
                graph.__init__(urdf_path=URDF_PATH, joint_names=JOINT_NAMES, link_names=LINK_NAMES,
                               urdf_content=urdf_content, combined_robot=cbot)
                add_geometry_items(graph.urdf_content, color=(0, 1, 0, 0.3), display=True, collision=True,
                                   exclude_link=["panda1_link7"])
                graph.set_cam_robot_collision()
                graph.set_rviz()
            elif args[1]:
                self.graph.combined_robot.detect_robots(self.graph.cam)
            else:
                print("Unknown button")
        else:
            TableInterface.button(self, button, *args, **kwargs)