from ..utils.utils import *
from ..utils.rotation_utils import *
from ..geometry.geometry import *
from ..constraint.constraint_object import ctype_to_htype, otype_to_class
from ..constraint.constraint_action import ctype_to_btype
from ..utils.joint_utils import joint_list2dict

from ..ui import dash_launcher
from ..ui.dash_launcher import TabInfo, TableInfo, table_updater_default, IDENTIFY_COL, TAB_BUTTON
from abc import *

__metaclass__ = type

def start_ui_server():
    dash_launcher.set_tabs([
        TabInfo("Instances", [TableInfo("Geometry", '570px'),
                             TableInfo("Object", '270px')]),
        TabInfo("Binding", [TableInfo("Handle", '570px'),
                            TableInfo("Binder", '270px')]),
    ])
    dash_launcher.run_server(on_background=True, debug=False)

CONFIG_DIR = os.path.join(os.getcwd(), 'configs')
try: os.mkdir(CONFIG_DIR)
except: pass

class UIBroker:

    def __init__(self, graph):
        self.graph = graph
        self.table_dict = {
            "Geometry": GeometryTable(graph),
            "Object": ObjectTable(graph),
            "Handle": HandleTable(graph),
            "Binder": BinderTable(graph),
        }

    def set_tables(self):
        dash_launcher.set_tables(
            table_loader_dict={k: v.get_table for k,v in self.table_dict.items()},
            table_selector_dict={k: v.select for k,v in self.table_dict.items()},
            table_updater_dict={k: v.update for k,v in self.table_dict.items()},
            table_button_dict={k: v.button for k,v in self.table_dict.items()},
        )

class TableInterface:
    HEADS = None
    def __init__(self, graph):
        self.graph = graph

    @abstractmethod
    def get_items(self):
        pass

    @abstractmethod
    def get_items_dict(self):
        pass

    @abstractmethod
    def serialize(self, item):
        pass

    @abstractmethod
    def select(self, selected_row_ids, active_row, active_col):
        pass

    @abstractmethod
    def update(self, active_row, active_col, value, add=False, delete=False):
        pass

    def button(self, button, filename=None, data=None):
        if button == TAB_BUTTON.SAVE:
            save_json(os.path.join(CONFIG_DIR, filename), data)
            pass
        elif button == TAB_BUTTON.LOAD:
            data_new = load_json(os.path.join(CONFIG_DIR, filename))
            dict_old = {dtem[IDENTIFY_COL]: dtem for dtem in data}
            dict_new = {dtem[IDENTIFY_COL]: dtem for dtem in data_new}
            names_old, names_new = set(dict_old.keys()), set(dict_new.keys())
            names_add, names_del = names_new-names_old, names_old-names_new
            names_update = names_new - names_add - names_del
            names_update, names_add, names_del = list(names_update), list(names_add), list(names_del)
            for name in names_del:
                print("del {}".format(name))
                self.update(name, None, None, delete=True)
            for name in names_add:
                print("add {}".format(name))
                self.update(None, None, dict_new[name], add=True)
            items_dict = self.get_items_dict()
            for name in names_update:
                tem_new = dict_new[name]
                arr_old = self.serialize(items_dict[name])
                for v_old, col in zip(arr_old, self.HEADS):
                    v_new = tem_new[col]
                    if v_old != v_new:
                        print("update {} - {} : {} -> {}".format(name, col, v_old, v_new))
                        self.update(name, col, v_new)

    def get_table(self):
        return (self.HEADS, [self.serialize(item) for item in self.get_items()])

class GeometryTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'GType', 'Link', 'Dims', 'Center', 'Rpy', 'Color', 'Disp', 'Coll', 'Fix', 'Soft', 'Online']

    def get_items(self):
        return self.graph.ghnd

    def get_items_dict(self):
        return self.graph.ghnd.NAME_DICT

    def serialize(self, gtem):
        return [gtem.name, gtem.gtype.name, gtem.link_name,
                round_it_str(gtem.dims), round_it_str(gtem.center),
                round_it_str(Rot2rpy(gtem.orientation_mat)), round_it_str(gtem.color, dec=1),
                str(gtem.display), str(gtem.collision), str(gtem.fixed), str(gtem.soft), str(gtem.online)]

    def select(self, selected_row_ids, active_row, active_col):
        self.graph.clear_highlight(['geometry'])
        if active_row is not None:
            self.graph.highlight_geometry('geometry', active_row, color=(1, 0.3, 0.3, 0.5))
        for row in selected_row_ids:
            if row != active_row:
                self.graph.highlight_geometry('geometry', row, color=(0.3, 0.3, 1, 0.5))

    def update(self, active_row, active_col, value, add=False, delete=False):
        res, msg = True, ""
        if add:
            self.graph.add_geometry(GeometryItem(name=value[IDENTIFY_COL], gtype=getattr(GEOTYPE, value['GType']),
                                                 link_name=value["Link"], center=str_num_it(value["Center"]),
                                                 dims=str_num_it(value["Dims"]), rpy=str_num_it(value["Rpy"]),
                                                 color=str_num_it(value["Color"]),
                                                 display=value["Disp"].lower()=="true",
                                                 collision=value["Coll"].lower()=="true",
                                                 fixed=value["Fix"].lower()=="true",
                                                 soft=value["Soft"].lower()=="true"))
            return res, msg

        if active_row not in self.graph.ghnd.NAME_DICT:
            return True, ""
        gtem = self.graph.ghnd.NAME_DICT[active_row]

        if delete:
            self.graph.remove_geometry(gtem)
        else:
            col_idx = self.HEADS.index(active_col)
            val_cur = self.serialize(gtem)[col_idx]
            if val_cur != value:
                if active_col == IDENTIFY_COL:
                    res, msg = False, IDENTIFY_COL + " is not changeable"
                elif active_col == "GType":
                    gtem.gtype = getattr(GEOTYPE, value)
                elif active_col == "Link":
                    gtem.set_link(value)
                elif active_col == "Dims":
                    gtem.set_dims(map(float, value.split(',')))
                elif active_col == 'Center':
                    gtem.set_center(map(float, value.split(',')))
                    gtem.set_offset_tf()
                elif active_col == 'Rpy':
                    gtem.set_orientation_mat(Rot_rpy(map(float, value.split(','))))
                    gtem.set_offset_tf()
                elif active_col == 'Color':
                    gtem.color = map(float, value.split(','))
                elif active_col == 'Disp':
                    gtem.display = value.lower() == 'true'
                elif active_col == 'Coll':
                    gtem.collision = value.lower() == 'true'
                elif active_col == 'Fix':
                    gtem.fixed = value.lower() == 'true'
                elif active_col == 'Soft':
                    gtem.soft = value.lower() == 'true'
                elif active_col == 'Online':
                    gtem.online = value.lower() == 'true'
                self.graph.update_marker(gtem)

        return res, msg

    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.APPLY:
            self.graph.set_rviz()
            if hasattr(self.graph, "planner"):
                self.graph.planner.update_gtems()
        else:
            TableInterface.button(self, button, *args, **kwargs)


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