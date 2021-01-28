from .table_interface import *
from ...planning.constraint.constraint_subject import otype_to_class
from ...utils.utils import list2dict

class ObjectTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'OType', 'Binding', 'Binder']
    HILIGHT_KEY = 'object'
    CUSTOM_BUTTONS = ["Apply"]

    def get_items(self):
        return map(lambda x: x[1], sorted(self.planning_pipeline.pscene.subject_dict.items(), key=lambda x:x[0]))

    def get_items_dict(self):
        return self.planning_pipeline.pscene.subject_dict

    def serialize(self, otem):
        return [otem.geometry.name, otem.__class__.__name__, otem.binding[0], otem.binding[1]]

    def highlight_item(self, otem, color=None):
        self.planning_pipeline.pscene.gscene.highlight_geometry(self.HILIGHT_KEY, otem.geometry.name, color=color)

    def add_item(self, value):
        try:
            self.planning_pipeline.pscene.create_object(oname=value[IDENTIFY_COL], gname=value[IDENTIFY_COL],
                                                        _type=otype_to_class(value['OType']),
                                                        binding=(value['Binding'], value['Binder']))
        except Exception as e:
            print(e)

    def delete_item(self, active_row):
        self.planning_pipeline.pscene.remove_object(active_row)

    def update_item(self, otem, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, IDENTIFY_COL + " is not changeable"
        elif active_col == "OType":
            res, msg = False, "Object Type is not changeable"
        elif active_col == "Binding":
            binding = (otem.geometry.name,value, otem.binding[1], self.planning_pipeline.pscene.actor_dict[otem.binding[1]].geometry.name)
            joint_dict = list2dict(self.planning_pipeline.pscene.gscene.joints.position,
                                   self.planning_pipeline.pscene.combined_robot.joint_names)
            self.planning_pipeline.pscene.rebind(binding, joint_dict)
        elif active_col == "Binder":
            binding = (otem.geometry.name, otem.binding[0],value, self.planning_pipeline.pscene.actor_dict[value].geometry.name)
            joint_dict = list2dict(self.planning_pipeline.pscene.gscene.joints.position,
                                   self.planning_pipeline.pscene.combined_robot.joint_names)
            self.planning_pipeline.pscene.rebind(binding, joint_dict)
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            self.planning_pipeline.pscene.update_subjects()
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")