from .table_interface import *
from ...planning.constraint.constraint_subject import otype_to_class, BindingChain
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
        return [otem.oname, otem.__class__.__name__, otem.binding.chain.handle_name, otem.binding.chain[2]]

    def highlight_item(self, otem, color=None):
        self.planning_pipeline.pscene.gscene.highlight_geometry(self.HILIGHT_KEY, otem.geometry.name, color=color)

    def add_item(self, value):
        try:
            binder_geometry = self.planning_pipeline.pscene.actor_dict[value['Binder']].geometry.name
            self.planning_pipeline.pscene.create_subject(oname=value[IDENTIFY_COL], gname=value[IDENTIFY_COL],
                                                        _type=otype_to_class(value['OType']),
                                                         chain=BindingChain(value[IDENTIFY_COL], value['Binding'], value['Binder'], binder_geometry))
        except Exception as e:
            print(e)

    def delete_item(self, active_row):
        self.planning_pipeline.pscene.remove_subject(active_row)

    def update_item(self, otem, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            res, msg = False, IDENTIFY_COL + " is not changeable"
        elif active_col == "OType":
            res, msg = False, "Object Type is not changeable"
        elif active_col == "Binding":
            chain = BindingChain(otem.oname, value, otem.binding.chain.actor_name, otem.binding.chain.actor_root_gname)
            joint_dict = list2dict(self.planning_pipeline.pscene.gscene.joints.position,
                                   self.planning_pipeline.pscene.combined_robot.joint_names)
            self.planning_pipeline.pscene.rebind(chain, joint_dict)
        elif active_col == "Binder":
            chain = BindingChain(otem.oname, otem.binding.chain.handle_name, value, otem.binding.chain.actor_root_gname)
            joint_dict = list2dict(self.planning_pipeline.pscene.gscene.joints.position,
                                   self.planning_pipeline.pscene.combined_robot.joint_names)
            self.planning_pipeline.pscene.rebind(chain, joint_dict)
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            self.planning_pipeline.pscene.update_subjects()
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")