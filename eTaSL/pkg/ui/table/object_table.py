from .table_interface import *
from ...constraint.constraint_object import otype_to_class
from ...utils.joint_utils import joint_list2dict

class ObjectTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'OType', 'Binding', 'Binder']

    def get_items(self):
        return map(lambda x: x[1], sorted(self.graph.object_dict.items(), key=lambda x:x[0]))

    def get_items_dict(self):
        return self.graph.object_dict

    def serialize(self, otem):
        return [otem.object.name, otem.__class__.__name__, otem.binding[0], otem.binding[1]]

    def select(self, selected_row_ids, active_row, active_col):
        self.graph.clear_highlight(['object'])
        object_dict = self.graph.object_dict
        if active_row in object_dict:
            otem = object_dict[active_row]
            self.graph.highlight_geometry('object', otem.object.name, color=(1, 0.3, 0.3, 0.5))
        for row in selected_row_ids:
            if row != active_row:
                otem = object_dict[row]
                self.graph.highlight_geometry('object', otem.object.name, color=(0.3, 0.3, 1, 0.5))

    def update(self, active_row, active_col, value, add=False, delete=False):
        res, msg = True, ""
        if add:
            try:
                self.graph.register_object(name=value[IDENTIFY_COL], _type=otype_to_class(value['OType']),
                                           binding=(value['Binding'], value['Binder']))
            except Exception as e:
                res, msg = False, str(e)
            return res, msg

        col_idx = self.HEADS.index(active_col)
        if active_row not in self.graph.object_dict:
            return True, ""
        otem = self.graph.object_dict[active_row]
        val_cur = self.serialize(otem)[col_idx]

        res, msg = True, ""

        if delete:
            self.graph.remove_object(active_row)
        elif val_cur != value:
            if active_col == IDENTIFY_COL:
                res, msg = False, IDENTIFY_COL + " is not changeable"
            elif active_col == "OType":
                res, msg = False, "Object Type is not changeable"
            elif active_col == "Binding":
                binding = (otem.object.name,value, otem.binding[1])
                joint_dict = joint_list2dict(self.graph.joints.position, self.graph.joint_names)
                self.graph.rebind(binding, joint_dict)
            elif active_col == "Binder":
                binding = (otem.object.name, otem.binding[0],value)
                joint_dict = joint_list2dict(self.graph.joints.position, self.graph.joint_names)
                self.graph.rebind(binding, joint_dict)
        return res, msg

    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.APPLY:
            self.graph.update_handles()
            if hasattr(self.graph, "planner"):
                self.graph.planner.set_object_dict(self.graph.object_dict)
        else:
            TableInterface.button(self, button, *args, **kwargs)