from ..utils.utils import *
from ..utils.rotation_utils import *
from ..geometry.geometry import *
from ..constraint.constraint_object import ctype_to_htype
from ..constraint.constraint_action import ctype_to_btype

from ..ui import dash_launcher
from ..ui.dash_launcher import TabInfo, TableInfo, table_updater_default, IDENTIFY_COL, TAB_BUTTON

def start_ui_server():
    dash_launcher.set_tabs([
        TabInfo("Geometry", [TableInfo("Geometry", '850px')]),
        TabInfo("Binding", [TableInfo("Handle", '570px'),
                            TableInfo("Binder", '850px')]),
    ])
    dash_launcher.run_server(on_background=True, debug=False)

class UIBroker:
    GEOMETRY_HEADS = [IDENTIFY_COL, 'GType', 'Link', 'Dims', 'Center', 'Rpy', 'Color', 'Disp', 'Coll', 'Fix', 'Soft', 'Online']
    HANDLE_HEADS = [IDENTIFY_COL, 'Object', 'Handle', 'CType', 'Point', 'Direction']
    BINDER_HEADS = [IDENTIFY_COL, 'CType', 'Geometry', 'Link', 'Direction', 'Point', 'Control', 'Multi']
    
    
    def __init__(self, graph):
        self.graph = graph

    ############################### TABLE GENERATERS ######################################
    @classmethod
    def serialize_geometry(cls, gtem):
        return [gtem.name, gtem.gtype.name, gtem.link_name,
                round_it_str(gtem.dims), round_it_str(gtem.center),
                round_it_str(Rot2rpy(gtem.orientation_mat)), round_it_str(gtem.color, dec=1),
                str(gtem.display), str(gtem.collision), str(gtem.fixed), str(gtem.soft), str(gtem.online)]

    @classmethod
    def serialize_handle(cls, htem):
        return [htem.name_constraint, htem.object.name,
                htem.name, htem.ctype.name,
                round_it_str(htem.point_dir[0]), round_it_str(htem.point_dir[1])]

    @classmethod
    def serialize_binder(cls, binder):
        return [binder.name, binder.ctype.name, binder.object.name, binder.object.link_name,
                round_it_str(binder.direction, 0), "n" if binder.point is None else round_it_str(binder.point, 0),
                str(binder.controlled), str(binder.multiple)]


    def get_geometry_table(self):
        return (UIBroker.GEOMETRY_HEADS, [UIBroker.serialize_geometry(gtem) for gtem in self.graph.ghnd])


    def get_handle_table(self):
        return (UIBroker.HANDLE_HEADS, [UIBroker.serialize_handle(htem) for htem in self.graph.get_all_handles()])


    def get_binder_table(self):
        return (UIBroker.BINDER_HEADS, [UIBroker.serialize_binder(binder) for binder in self.graph.binder_dict.values()]
                )


    ############################### AXIS ADDER ######################################

    def add_handle_axis(self, hl_key, handle, color=None):
        hobj = handle.handle.object
        if hasattr(handle, 'orientation_mat'):
            orientation_mat = np.matmul(hobj.orientation_mat, handle.orientation_mat)
            axis = "xyz"
            color = None
        elif hasattr(handle, 'direction'):
            orientation_mat = np.matmul(hobj.orientation_mat,
                                        Rotation.from_rotvec(calc_rotvec_vecs([1, 0, 0], handle.direction)).as_dcm())
            axis = "x"
            color = color
        else:
            raise (RuntimeError("direction or orientation not specified for handle"))
        self.graph.add_highlight_axis(hl_key, hobj.name, hobj.link_name, hobj.center, orientation_mat, color=color, axis=axis)


    def add_binder_axis(self, hl_key, binder, color=None):
        bobj = binder.effector.object
        if hasattr(binder, 'orientation'):
            orientation_mat = np.matmul(bobj.orientation_mat, binder.effector.orientation_mat)
            axis = "xyz"
            axis = "xyz"
        elif hasattr(binder, 'direction'):
            orientation_mat = np.matmul(bobj.orientation_mat, Rotation.from_rotvec(
                calc_rotvec_vecs([1, 0, 0], binder.effector.direction)).as_dcm())
            axis = "x"
            color = color
        else:
            raise (RuntimeError("direction or orientation not specified for handle"))
        self.graph.add_highlight_axis(hl_key, bobj.name, bobj.link_name, bobj.center, orientation_mat, color=color, axis=axis)


    ############################### SELECTORS ######################################

    def geometry_table_selector(self, selected_row_ids, active_row, active_col):
        self.graph.clear_highlight(['geometry'])
        if active_row is not None:
            self.graph.highlight_geometry('geometry', active_row, color=(1, 0.3, 0.3, 0.5))
        for row in selected_row_ids:
            if row != active_row:
                self.graph.highlight_geometry('geometry', row, color=(0.3, 0.3, 1, 0.5))


    def handle_table_selector(self, selected_row_ids, active_row, active_col):
        self.graph.clear_highlight(['handle'])
        handle_dict = self.graph.get_all_handle_dict()
        if active_row in handle_dict:
            handle = handle_dict[active_row]
            self.add_handle_axis('handle', handle)
            self.graph.highlight_geometry('handle', handle.handle.object.name, color=(1, 0.3, 0.3, 0.5))
        for row in selected_row_ids:
            if row != active_row:
                handle = handle_dict[row]
                self.add_handle_axis('handle', handle, color=(0, 0, 1, 0.5))
                self.graph.highlight_geometry('handle', handle.handle.object.name, color=(0.3, 0.3, 1, 0.5))


    def binder_table_selector(self, selected_row_ids, active_row, active_col):
        self.graph.clear_highlight(['binder'])
        binder_dict = self.graph.binder_dict
        if active_row in binder_dict:
            handle = binder_dict[active_row]
            self.add_binder_axis('binder', handle)
            self.graph.highlight_geometry('binder', handle.effector.object.name, color=(1, 0.3, 0.3, 0.5))
        for row in selected_row_ids:
            if row != active_row:
                handle = binder_dict[row]
                self.add_binder_axis('binder', handle, color=(0, 0, 1, 0.5))
                self.graph.highlight_geometry('binder', handle.effector.object.name, color=(0.3, 0.3, 1, 0.5))


    ############################### UPDATERS ######################################

    def marker_updater(self, gtem):
        joint_dict = {self.graph.joints.name[i]: self.graph.joints.position[i] for i in range(len(self.graph.joint_names))}
        marks = [mk for mk in self.graph.marker_list if mk.geometry == gtem]
        for mk in marks:
            mk.set_marker(joint_dict, create=False)
        return marks


    def geometry_table_updater(self, active_row, active_col, value, add=False, delete=False):
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
        col_idx = UIBroker.GEOMETRY_HEADS.index(active_col)
        gtem = self.graph.ghnd.NAME_DICT[active_row]
        val_cur = UIBroker.serialize_geometry(gtem)[col_idx]

        if delete:
            self.graph.remove_geometry(gtem)
        elif val_cur != value:
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
            self.marker_updater(gtem)

        return res, msg


    def handle_table_updater(self, active_row, active_col, value, add=False, delete=False):
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

        col_idx = UIBroker.HANDLE_HEADS.index(active_col)
        if active_row not in hdict:
            return True, ""
        htem = hdict[active_row]
        val_cur = UIBroker.serialize_handle(htem)[col_idx]

        if delete:
            otem = self.graph.object_dict[htem.object.name]
            self.graph.delete_handle(htem)
            if not otem.action_points_dict.keys():
                self.remove_object(htem.object.name)
        elif val_cur != value:
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


    def binder_table_updater(self, active_row, active_col, value, add=False, delete=False):
        res, msg = True, ""
        if add:
            self.graph.register_binder(name=value[IDENTIFY_COL], object_name=value["Geometry"],
                                       _type=ctype_to_btype(value['CType']), link_name=value['Link'],
                                       point=str_num_it(value['Point']), direction=str_num_it(value['Direction']))
            return res, msg

        col_idx = UIBroker.BINDER_HEADS.index(active_col)
        if active_row not in self.graph.binder_dict:
            return True, ""
        binder = self.graph.binder_dict[active_row]
        val_cur = UIBroker.serialize_binder(binder)[col_idx]

        res, msg = True, ""

        if delete:
            self.graph.remove_binder(active_row)
        elif val_cur != value:
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

    def geometry_table_button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.APPLY:
            self.graph.set_rviz()
        elif button == TAB_BUTTON.SAVE:
            print("save button clicked")
            pass
        elif button == TAB_BUTTON.LOAD:
            print("load button clicked")
            pass

    def handle_table_button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.APPLY:
            print("apply button clicked")
            pass
        elif button == TAB_BUTTON.SAVE:
            print("save button clicked")
            pass
        elif button == TAB_BUTTON.LOAD:
            print("load button clicked")
            pass

    def binder_table_button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.APPLY:
            print("apply button clicked")
            pass
        elif button == TAB_BUTTON.SAVE:
            print("save button clicked")
            pass
        elif button == TAB_BUTTON.LOAD:
            print("load button clicked")
            pass

    def set_tables(self):
        dash_launcher.set_tables(
            table_loader_dict={
                "Geometry": self.get_geometry_table,
                "Handle": self.get_handle_table,
                "Binder": self.get_binder_table
            },
            table_selector_dict={
                "Geometry": self.geometry_table_selector,
                "Handle": self.handle_table_selector,
                "Binder": self.binder_table_selector,
            },
            table_updater_dict={
                "Geometry": self.geometry_table_updater,
                "Handle": self.handle_table_updater,
                "Binder": self.binder_table_updater,
            },
            table_button_dict={
                "Geometry": self.geometry_table_button,
                "Handle": self.handle_table_button,
                "Binder": self.binder_table_button,
            }
        )