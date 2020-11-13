from .table_interface import *
from ...constraint.constraint_action import ctype_to_btype

class BinderTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'CType', 'Geometry', 'Link', 'Direction', 'Point', 'Control', 'Multi']

    def get_items(self):
        return map(lambda x: x[1], sorted(self.graph.binder_dict.items(), key=lambda x:x[0]))

    def get_items_dict(self):
        return self.graph.binder_dict

    def serialize(self, binder):
        return [binder.name, binder.ctype.name, binder.object.name, binder.object.link_name,
                round_it_str(binder.direction, 0), "n" if binder.point is None else round_it_str(binder.point, 0),
                str(binder.controlled), str(binder.multiple)]

    def select(self, selected_row_ids, active_row, active_col):
        self.graph.clear_highlight(['binder'])
        binder_dict = self.graph.binder_dict
        if active_row in binder_dict:
            handle = binder_dict[active_row]
            self.graph.add_binder_axis('binder', handle)
            self.graph.highlight_geometry('binder', handle.effector.object.name, color=(1, 0.3, 0.3, 0.5))
        for row in selected_row_ids:
            if row != active_row:
                handle = binder_dict[row]
                self.graph.add_binder_axis('binder', handle, color=(0, 0, 1, 0.5))
                self.graph.highlight_geometry('binder', handle.effector.object.name, color=(0.3, 0.3, 1, 0.5))

    def update(self, active_row, active_col, value, add=False, delete=False):
        res, msg = True, ""
        if add:
            self.graph.register_binder(name=value[IDENTIFY_COL], object_name=value["Geometry"],
                                       _type=ctype_to_btype(value['CType']), link_name=value['Link'],
                                       point=str_num_it(value['Point']), direction=str_num_it(value['Direction']))
            return res, msg

        if active_row not in self.graph.binder_dict:
            return True, ""
        binder = self.graph.binder_dict[active_row]

        res, msg = True, ""

        if delete:
            self.graph.remove_binder(active_row)
        else:
            col_idx = self.HEADS.index(active_col)
            val_cur = self.serialize(binder)[col_idx]
            if val_cur != value:
                if active_col == IDENTIFY_COL:
                    res, msg = False, IDENTIFY_COL + " is not changeable"
                elif active_col == "CType":
                    res, msg = False, "Constraint Type is not changeable"
                elif active_col == "Geometry":
                    res, msg = False, "Geometry is not changeable"
                elif active_col == "Link":
                    res, msg = False, "Link is not changeable"
                elif active_col == "Direction":
                    binder.direction = (map(float, value.split(',')))
                    binder.effector.set_binding_direction(binder.direction)
                elif active_col == 'Point':
                    if "," in value:
                        binder.point = (map(float, value.split(',')))
                    else:
                        if binder.point is not None:
                            res, msg = False, "Invalid point"
                elif active_col == 'Control':
                    binder.controlled = value.lower() == 'true'
                elif active_col == 'Multi':
                    binder.multiple = value.lower() == 'true'
        return res, msg

    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.APPLY:
            if hasattr(self.graph, "planner"):
                self.graph.planner.set_binder_dict(self.graph.binder_dict)
        else:
            TableInterface.button(self, button, *args, **kwargs)