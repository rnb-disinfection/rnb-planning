from .table_interface import *
from ...constraint.constraint_object import ctype_to_htype

class HandleTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'Object', 'Handle', 'CType', 'Point', 'Direction']

    def get_items(self):
        return self.graph.get_all_handles()

    def get_items_dict(self):
        return self.graph.get_all_handle_dict()

    def serialize(self, htem):
        return [htem.name_constraint, htem.object.name,
                htem.name, htem.ctype.name,
                round_it_str(htem.point_dir[0]), round_it_str(htem.point_dir[1])]

    def select(self, selected_row_ids, active_row, active_col):
        self.graph.clear_highlight(['handle'])
        handle_dict = self.graph.get_all_handle_dict()
        if active_row in handle_dict:
            handle = handle_dict[active_row]
            self.graph.add_handle_axis('handle', handle)
            self.graph.highlight_geometry('handle', handle.handle.object.name, color=(1, 0.3, 0.3, 0.5))
        for row in selected_row_ids:
            if row != active_row:
                handle = handle_dict[row]
                self.graph.add_handle_axis('handle', handle, color=(0, 0, 1, 0.5))
                self.graph.highlight_geometry('handle', handle.handle.object.name, color=(0.3, 0.3, 1, 0.5))

    def update(self, active_row, active_col, value, add=False, delete=False):
        res, msg = True, ""
        hdict = self.graph.get_all_handle_dict()
        if add:
            otem = self.graph.object_dict[value["Object"]]
            cname = value[IDENTIFY_COL]
            hname = value['Handle']
            otem.action_points_dict[hname] = ctype_to_htype(value['CType'])(hname, otem.object,
                                                                            (str_num_it(value["Point"]),
                                                                             str_num_it(value["Direction"])),
                                                                             name_constraint=cname)
            return res, msg

        if active_row not in hdict:
            return True, ""
        htem = hdict[active_row]

        if delete:
            otem = self.graph.object_dict[htem.object.name]
            self.graph.delete_handle(htem)
            if not otem.action_points_dict.keys():
                self.graph.remove_object(htem.object.name)
        else:
            col_idx = self.HEADS.index(active_col)
            val_cur = self.serialize(htem)[col_idx]
            if val_cur != value:
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