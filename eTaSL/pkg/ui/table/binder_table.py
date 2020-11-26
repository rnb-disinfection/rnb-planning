from .table_interface import *
from ...constraint.constraint_action import ctype_to_btype

class BinderTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'CType', 'Geometry', 'Link', 'RPY', 'Point', 'Control', 'Multi']
    HILIGHT_KEY = 'binder'
    CUSTOM_BUTTONS = ["Apply"]

    def get_items(self):
        return map(lambda x: x[1], sorted(self.graph.binder_dict.items(), key=lambda x:x[0]))

    def get_items_dict(self):
        return self.graph.binder_dict

    def serialize(self, binder):
        return [binder.name, binder.ctype.name, binder.object.name, binder.object.link_name,
                round_it_str(binder.rpy_point, 3), "n" if binder.point is None else round_it_str(binder.point, 3),
                str(binder.controlled), str(binder.multiple)]

    def highlight_item(self, binder, color=None):
        self.graph.add_handle_axis(self.HILIGHT_KEY, binder)
        self.graph.highlight_geometry(self.HILIGHT_KEY, binder.object.name, color=color)

    def add_item(self, value):
        try:
            self.graph.register_binder(name=value[IDENTIFY_COL], object_name=value["Geometry"],
                                       _type=ctype_to_btype(value['CType']), link_name=value['Link'],
                                       point=str_num_it(value['Point']), direction=str_num_it(value['RPY']))
        except Exception as e:
            print(e)

    def delete_item(self, active_row):
        self.graph.remove_binder(active_row)

    def update_item(self, binder, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, IDENTIFY_COL + " is not changeable"
        elif active_col == "CType":
            res, msg = False, "Constraint Type is not changeable"
        elif active_col == "Geometry":
            res, msg = False, "Geometry is not changeable"
        elif active_col == "Link":
            res, msg = False, "Link is not changeable"
        elif active_col == "RPY":
            binder.set_point_rpy(binder.point, str_num_it(value))
        elif active_col == 'Point':
            value = str_num_it(value) if "," in value else None
            binder.set_point_rpy(value, binder.rpy_point)
        elif active_col == 'Control':
            binder.controlled = value.lower() in ["true", "t"]
        elif active_col == 'Multi':
            binder.multiple = value.lower() in ["true", "t"]
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            if self.graph.planner:
                self.graph.planner.set_binder_dict(self.graph.binder_dict)
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")