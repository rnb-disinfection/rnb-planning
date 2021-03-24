from .table_interface import *
from ...geometry.builder.scene_builder import *

class RobotTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'RType', 'Position', 'Direction']
    HILIGHT_KEY = 'robot'
    CUSTOM_BUTTONS = ["Apply", "Detect"]

    def get_items(self):
        cbot = self.planning_pipeline.pscene.combined_robot
        return [(rbt_cfg.get_indexed_name(), rbt_cfg.type.name,
                 round_it_str(rbt_cfg.xyzrpy[0], 4),
                 round_it_str(rbt_cfg.xyzrpy[1], 4)) for rbt_cfg in cbot.robots_on_scene]

    def get_items_dict(self):
        return {item[0]: item for item in self.get_items()}

    def serialize(self, gtem):
        return gtem

    def highlight_item(self, gtem, color=None):
        pass

    def select(self, selected_row_ids, active_row, active_col):
        connection_list = [rbt_cfg.get_indexed_name() in selected_row_ids for rbt_cfg in self.planning_pipeline.pscene.combined_robot.robots_on_scene]
        self.planning_pipeline.pscene.combined_robot.reset_connection(*connection_list)

    def add_item(self, value):
        raise(RuntimeError("Cannot add or delete robot"))

    def delete_item(self, active_row):
        raise(RuntimeError("Cannot add or delete robot"))

    def update_item(self, atem, active_col, value):
        cbot = self.planning_pipeline.pscene.combined_robot
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
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            if args[0]:
                crob = self.planning_pipeline.pscene.combined_robot
                self.s_builder.create_gscene(crob, gscene_from=self.gscene)
            elif args[1]:
                xyz_rpy_robots = self.s_builder.detect_items(level_mask=[DetectionLevel.ROBOT])
                self.planning_pipeline.pscene.combined_robot.update_robot_pos_dict(xyz_rpy_robots=xyz_rpy_robots)
            else:
                print("Unknown button")
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")