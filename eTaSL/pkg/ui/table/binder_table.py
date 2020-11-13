from .table_interface import *
from ...constraint.constraint_action import ctype_to_btype

class BinderTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'CType', 'Geometry', 'Link', 'Direction', 'Point', 'Control', 'Multi']
    HILIGHT_KEY = 'binder'
    CUSTOM_BUTTONS = ["Apply"]

    def get_items(self):
        return map(lambda x: x[1], sorted(self.graph.binder_dict.items(), key=lambda x:x[0]))

    def get_items_dict(self):
        return self.graph.binder_dict

    def serialize(self, binder):
        return [binder.name, binder.ctype.name, binder.object.name, binder.object.link_name,
                round_it_str(binder.direction, 0), "n" if binder.point_offset is None else round_it_str(binder.point_offset, 3),
                str(binder.controlled), str(binder.multiple)]

    def highlight_item(self, handle, color=None):
        self.graph.add_binder_axis(self.HILIGHT_KEY, handle)
        self.graph.highlight_geometry(self.HILIGHT_KEY, handle.effector.object.name, color=color)

    def add_item(self, value):
        try:
            self.graph.register_binder(name=value[IDENTIFY_COL], object_name=value["Geometry"],
                                       _type=ctype_to_btype(value['CType']), link_name=value['Link'],
                                       point=str_num_it(value['Point']), direction=str_num_it(value['Direction']))
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
        elif active_col == "Direction":
            binder.direction = str_num_it(value)
            binder.effector.set_binding_direction(binder.direction)
        elif active_col == 'Point':
            if "," in value:
                binder.point_offset = str_num_it(value)
                if binder.point is None:
                    binder.object.set_offset_tf(center=binder.point_offset)
                else:
                    binder.point = binder.point_offset
            else:
                if binder.point is not None:
                    res, msg = False, "Invalid point"
        elif active_col == 'Control':
            binder.controlled = value.lower() == 'true'
        elif active_col == 'Multi':
            binder.multiple = value.lower() == 'true'
        return res, msg

    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.CUSTOM:
            if hasattr(self.graph, "planner"):
                self.graph.planner.set_binder_dict(self.graph.binder_dict)
        else:
            TableInterface.button(self, button, *args, **kwargs)