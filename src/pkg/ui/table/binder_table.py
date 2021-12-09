from .table_interface import *
from ...planning.constraint.constraint_actor import ctype_to_btype

class BinderTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'CType', 'Geometry', 'RPY', 'Point', 'Control', 'Multi']
    HILIGHT_KEY = 'binder'
    CUSTOM_BUTTONS = ["Apply"]

    def get_items(self):
        return map(lambda x: x[1], sorted(self.planning_pipeline.pscene.actor_dict.items(), key=lambda x:x[0]))

    def get_items_dict(self):
        return self.planning_pipeline.pscene.actor_dict

    def serialize(self, binder):
        return [binder.name, binder.ctype.name, binder.geometry.name,
                round_it_str(binder.rpy_point, 3), "n" if binder.point is None else round_it_str(binder.point, 3),
                str(binder.active), str(binder.multiple)]

    def highlight_item(self, binder, color=None):
        self.planning_pipeline.pscene.add_handle_axis(self.HILIGHT_KEY, binder)
        self.planning_pipeline.pscene.gscene.highlight_geometry(self.HILIGHT_KEY, binder.geometry.name, color=color)

    def add_item(self, value):
        try:
            self.planning_pipeline.pscene.create_binder(bname=value[IDENTIFY_COL], gname=value["Geometry"],
                                       _type=ctype_to_btype(value['CType']),
                                       point=str_num_it(value['Point']), rpy=str_num_it(value['RPY']))
        except Exception as e:
            print(e)

    def delete_item(self, active_row):
        self.planning_pipeline.pscene.remove_binder(active_row)

    def update_item(self, binder, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, IDENTIFY_COL + " is not changeable"
        elif active_col == "CType":
            res, msg = False, "Constraint Type is not changeable"
        elif active_col == "Geometry":
            res, msg = False, "Geometry is not changeable"
        elif active_col == "RPY":
            binder.set_point_rpy(binder.point, str_num_it(value))
        elif active_col == 'Point':
            value = str_num_it(value) if "," in value else None
            binder.set_point_rpy(value, binder.rpy_point)
        elif active_col == 'Control':
            binder.active = value.lower() in ["true", "t"]
        elif active_col == 'Multi':
            binder.multiple = value.lower() in ["true", "t"]
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            self.planning_pipeline.update()
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")