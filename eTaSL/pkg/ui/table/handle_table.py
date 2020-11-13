from .table_interface import *
from ...constraint.constraint_object import ctype_to_htype

class HandleTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'Object', 'Handle', 'CType', 'Point', 'Direction']
    HILIGHT_KEY = 'handle'

    def get_items(self):
        return self.graph.get_all_handles()

    def get_items_dict(self):
        return self.graph.get_all_handle_dict()

    def serialize(self, htem):
        return [htem.name_constraint, htem.object.name,
                htem.name, htem.ctype.name,
                round_it_str(htem.point_dir[0]), round_it_str(htem.point_dir[1])]

    def highlight_item(self, handle, color=None):
        self.graph.add_handle_axis(self.HILIGHT_KEY, handle, color=color)
        self.graph.highlight_geometry(self.HILIGHT_KEY, handle.handle.object.name, color=color)

    def add_item(self, value):
        otem = self.graph.object_dict[value["Object"]]
        cname = value[IDENTIFY_COL]
        hname = value['Handle']
        otem.action_points_dict[hname] = ctype_to_htype(value['CType'])(hname, otem.object,
                                                                        (str_num_it(value["Point"]),
                                                                         str_num_it(value["Direction"])),
                                                                         name_constraint=cname)

    def delete_item(self, active_row):
        hdict = self.graph.get_all_handle_dict()
        htem = hdict[active_row]
        otem = self.graph.object_dict[htem.object.name]
        self.graph.delete_handle(htem)
        if not otem.action_points_dict.keys():
            self.graph.remove_object(htem.object.name)

    def update_item(self, htem, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, IDENTIFY_COL + " is not changeable"
        elif active_col == "Object":
            res, msg = False, "Object is not changeable"
        elif active_col == "Handle":
            res, msg = False, "Handle name is not changeable"
        elif active_col == "CType":
            res, msg = False, "Constraint Type is not changeable"
        elif active_col == 'Point':
            htem.set_point_dir(((map(float, value.split(','))), htem.point_dir[1]))
            htem.update_handle()
        elif active_col == 'Direction':
            htem.set_point_dir((htem.point_dir[0], (map(float, value.split(',')))))
            htem.update_handle()
        return res, msg

    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.APPLY:
            self.graph.update_handles()
            if hasattr(self.graph, "planner"):
                self.graph.planner.set_object_dict(self.graph.object_dict)
        else:
            TableInterface.button(self, button, *args, **kwargs)