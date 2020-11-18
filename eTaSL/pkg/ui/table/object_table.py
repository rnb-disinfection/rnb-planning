from .table_interface import *
from ...constraint.constraint_object import otype_to_class
from ...utils.joint_utils import joint_list2dict

class ObjectTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'OType', 'Binding', 'Binder']
    HILIGHT_KEY = 'object'
    CUSTOM_BUTTONS = ["Apply"]

    def get_items(self):
        return map(lambda x: x[1], sorted(self.graph.object_dict.items(), key=lambda x:x[0]))

    def get_items_dict(self):
        return self.graph.object_dict

    def serialize(self, otem):
        return [otem.object.name, otem.__class__.__name__, otem.binding[0], otem.binding[1]]

    def highlight_item(self, otem, color=None):
        self.graph.highlight_geometry(self.HILIGHT_KEY, otem.object.name, color=color)

    def add_item(self, value):
        try:
            self.graph.register_object(name=value[IDENTIFY_COL], _type=otype_to_class(value['OType']),
                                       binding=(value['Binding'], value['Binder']))
        except Exception as e:
            print(e)

    def delete_item(self, active_row):
        self.graph.remove_object(active_row)

    def update_item(self, otem, active_col, value):
        res, msg = True, ""
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
        if button == TAB_BUTTON.CUSTOM:
            self.graph.update_handles()
            if self.graph.planner:
                self.graph.planner.set_object_dict(self.graph.object_dict)
        else:
            TableInterface.button(self, button, *args, **kwargs)