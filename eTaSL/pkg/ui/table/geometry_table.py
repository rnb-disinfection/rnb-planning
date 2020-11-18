from .table_interface import *
from ...utils.rotation_utils import *
from ...geometry.geometry import *

class GeometryTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'GType', 'Link', 'Dims', 'Center', 'Rpy', 'Color', 'Disp', 'Coll', 'Fix', 'Soft', 'Online']
    HILIGHT_KEY = 'geometry'
    CUSTOM_BUTTONS = ["Apply"]

    def get_items(self):
        return self.graph.ghnd

    def get_items_dict(self):
        return self.graph.ghnd.NAME_DICT

    def serialize(self, gtem):
        return [gtem.name, gtem.gtype.name, gtem.link_name,
                round_it_str(gtem.dims), round_it_str(gtem.center),
                round_it_str(Rot2rpy(gtem.orientation_mat)), round_it_str(gtem.color, dec=1),
                str(gtem.display), str(gtem.collision), str(gtem.fixed), str(gtem.soft), str(gtem.online)]

    def highlight_item(self, gtem, color=None):
        self.graph.highlight_geometry(self.HILIGHT_KEY, gtem.name, color=color)

    def add_item(self, value):
        self.graph.add_geometry(
            self.graph.ghnd.create_safe(name=value[IDENTIFY_COL], gtype=getattr(GEOTYPE, value['GType']),
                                  link_name=value["Link"], center=str_num_it(value["Center"]),
                                  dims=str_num_it(value["Dims"]), rpy=str_num_it(value["Rpy"]),
                                  color=str_num_it(value["Color"]),
                                  display=value["Disp"].lower()=="true",
                                  collision=value["Coll"].lower()=="true",
                                  fixed=value["Fix"].lower()=="true",
                                  soft=value["Soft"].lower()=="true"))

    def delete_item(self, active_row):
        gtem = self.graph.ghnd.NAME_DICT[active_row]
        self.graph.remove_geometry(gtem)

    def update_item(self, gtem, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, IDENTIFY_COL + " is not changeable"
        elif active_col == "GType":
            gtem.gtype = getattr(GEOTYPE, value)
        elif active_col == "Link":
            gtem.set_link(value)
        elif active_col == "Dims":
            gtem.set_dims(map(float, value.split(',')))
        elif active_col == 'Center':
            gtem.set_center(map(float, value.split(',')))
            gtem.set_offset_tf()
        elif active_col == 'Rpy':
            gtem.set_orientation_mat(Rot_rpy(map(float, value.split(','))))
            gtem.set_offset_tf()
        elif active_col == 'Color':
            gtem.color = map(float, value.split(','))
        elif active_col == 'Disp':
            gtem.display = value.lower() == 'true'
        elif active_col == 'Coll':
            gtem.collision = value.lower() == 'true'
        elif active_col == 'Fix':
            gtem.fixed = value.lower() == 'true'
        elif active_col == 'Soft':
            gtem.soft = value.lower() == 'true'
        elif active_col == 'Online':
            gtem.online = value.lower() == 'true'
        self.graph.update_marker(gtem)
        return res, msg

    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.CUSTOM:
            self.graph.set_rviz(self.graph.joints.position)
            if self.graph.planner:
                self.graph.planner.update_gtems()
        else:
            TableInterface.button(self, button, *args, **kwargs)