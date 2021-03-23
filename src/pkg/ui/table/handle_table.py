from .table_interface import *
from ...planning.constraint.constraint_subject import ctype_to_htype

class HandleTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'Object', 'Handle', 'CType', 'Point', 'RPY']
    HILIGHT_KEY = 'handle'
    CUSTOM_BUTTONS = ["Apply"]

    def get_items(self):
        return self.planning_pipeline.pscene.get_all_handles()

    def get_items_dict(self):
        return self.planning_pipeline.pscene.get_all_handle_dict()

    def serialize(self, htem):
        return [htem.name_full, htem.geometry.name,
                htem.name, htem.ctype.name,
                round_it_str(htem.point) if htem.point is not None else "None",
                round_it_str(htem.rpy_point) if htem.rpy_point is not None else "None"]

    def highlight_item(self, handle, color=None):
        self.planning_pipeline.pscene.add_handle_axis(self.HILIGHT_KEY, handle)
        self.planning_pipeline.pscene.gscene.highlight_geometry(self.HILIGHT_KEY, handle.geometry.name, color=color)

    def add_item(self, value):
        otem = self.planning_pipeline.pscene.subject_dict[value["Object"]]
        cname = value[IDENTIFY_COL]
        hname = value['Handle']
        otem.action_points_dict[hname] = ctype_to_htype(value['CType'])(hname, otem.geometry,
                                                                        (str_num_it(value["Point"]),
                                                                         str_num_it(value["RPY"])),
                                                                         name_full=cname)

    def delete_item(self, active_row):
        hdict = self.planning_pipeline.pscene.get_all_handle_dict()
        htem = hdict[active_row]
        otem = self.planning_pipeline.pscene.subject_dict[htem.geometry.name]
        self.planning_pipeline.pscene.delete_handle(htem)
        if not otem.action_points_dict.keys():
            self.planning_pipeline.pscene.remove_subject(htem.geometry.name)

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
            htem.set_point_rpy((map(float, value.split(','))), htem.rpy_point)
            htem.update_handle()
        elif active_col == 'RPY':
            htem.set_point_rpy(htem.point, (map(float, value.split(','))))
            htem.update_handle()
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            self.planning_pipeline.pscene.update_subjects()
            self.planning_pipeline.update()
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")