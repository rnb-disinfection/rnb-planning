from ..utils.utils import *
from ..utils.rotation_utils import *
from ..geometry.geometry import *

############################### TABLE GENERATERS ######################################

GEOMETRY_HEADS = ['Name', 'GType', 'Link', 'Dims', 'Center', 'Rpy', 'Disp', 'Coll', 'Fix', 'Soft', 'Online']


def serialize_geometry(gtem):
    return [gtem.name, gtem.gtype.name, gtem.link_name,
            round_it_str(gtem.dims), round_it_str(gtem.center), round_it_str(Rot2rpy(gtem.orientation_mat)),
            str(gtem.display), str(gtem.collision), str(gtem.fixed), str(gtem.soft), str(gtem.online)]


HANDLE_HEADS = ['Name', 'Object', 'Handle', 'CType', 'Point', 'Direction']


def serialize_handle(htem):
    return [htem.name_constraint, htem.object.name,
            htem.name, htem.ctype.name,
            round_it_str(htem.point_dir[0]), round_it_str(htem.point_dir[1])]


BINDER_HEADS = ['Name', 'CType', 'Geometry', 'Direction', 'Point', 'Control', 'Multi']


def serialize_binder(binder):
    return [binder.name, binder.ctype.name, binder.object.name,
            round_it_str(binder.direction, 0), "" if binder.point is None else round_it_str(binder.point, 0),
            str(binder.controlled), str(binder.multiple)]


def get_geometry_table(graph):
    return (GEOMETRY_HEADS, [serialize_geometry(gtem) for gtem in graph.ghnd])


def get_handle_table(graph):
    return (HANDLE_HEADS, [serialize_handle(htem) for htem in graph.get_all_handles()])


def get_binder_table(graph):
    return (BINDER_HEADS, [serialize_binder(binder) for binder in graph.binder_dict.values()]
            )


############################### AXIS ADDER ######################################

def add_handle_axis(graph, hl_key, handle, color=None):
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
    graph.add_highlight_axis(hl_key, hobj.name, hobj.link_name, hobj.center, orientation_mat, color=color, axis=axis)


def add_binder_axis(graph, hl_key, binder, color=None):
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
    graph.add_highlight_axis(hl_key, bobj.name, bobj.link_name, bobj.center, orientation_mat, color=color, axis=axis)


############################### SELECTORS ######################################

def geometry_table_selector(graph, selected_row_ids, active_row, active_col):
    graph.clear_highlight(['geometry'])
    if active_row is not None:
        graph.highlight_geometry('geometry', active_row, color=(1, 0.3, 0.3, 0.5))
    for row in selected_row_ids:
        if row != active_row:
            graph.highlight_geometry('geometry', row, color=(0.3, 0.3, 1, 0.5))


def handle_table_selector(graph, selected_row_ids, active_row, active_col):
    graph.clear_highlight(['handle'])
    handle_dict = graph.get_all_handle_dict()
    if active_row is not None:
        handle = handle_dict[active_row]
        add_handle_axis(graph, 'handle', handle)
        graph.highlight_geometry('handle', handle.handle.object.name, color=(1, 0.3, 0.3, 0.5))
    for row in selected_row_ids:
        if row != active_row:
            handle = handle_dict[row]
            add_handle_axis(graph, 'handle', handle, color=(0, 0, 1, 0.5))
            graph.highlight_geometry('handle', handle.handle.object.name, color=(0.3, 0.3, 1, 0.5))


def binder_table_selector(graph, selected_row_ids, active_row, active_col):
    graph.clear_highlight(['binder'])
    binder_dict = graph.binder_dict
    if active_row is not None:
        handle = binder_dict[active_row]
        add_binder_axis(graph, 'binder', handle)
        graph.highlight_geometry('binder', handle.effector.object.name, color=(1, 0.3, 0.3, 0.5))
    for row in selected_row_ids:
        if row != active_row:
            handle = binder_dict[row]
            add_binder_axis(graph, 'binder', handle, color=(0, 0, 1, 0.5))
            graph.highlight_geometry('binder', handle.effector.object.name, color=(0.3, 0.3, 1, 0.5))


############################### UPDATERS ######################################

def marker_updater(graph, gtem):
    joint_dict = {graph.joints.name[i]: graph.joints.position[i] for i in range(len(graph.joint_names))}
    marks = [mk for mk in graph.marker_list if mk.geometry == gtem]
    for mk in marks:
        mk.set_marker(joint_dict, create=False)
    return marks


def geometry_table_updater(graph, active_row, active_col, value):
    col_idx = GEOMETRY_HEADS.index(active_col)
    gtem = graph.ghnd.NAME_DICT[active_row]
    val_cur = serialize_geometry(gtem)[col_idx]

    res, msg = True, ""

    if val_cur != value:
        if active_col == "Name":
            res, msg = False, "Name is not changeable"
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
        marker_updater(graph, gtem)

    return res, msg


def handle_table_updater(graph, active_row, active_col, value):
    col_idx = HANDLE_HEADS.index(active_col)
    htem = graph.get_all_handle_dict()[active_row]
    val_cur = serialize_handle(htem)[col_idx]

    res, msg = True, ""
    if val_cur != value:
        if active_col == "Name":
            res, msg = False, "Name is not changeable"
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


def binder_table_updater(graph, active_row, active_col, value):
    col_idx = BINDER_HEADS.index(active_col)
    binder = graph.binder_dict[active_row]
    val_cur = serialize_binder(binder)[col_idx]

    res, msg = True, ""
    if val_cur != value:
        if active_col == "Name":
            res, msg = False, "Name is not changeable"
        elif active_col == "CType":
            res, msg = False, "Constraint Type is not changeable"
        elif active_col == "Geometry":
            res, msg = False, "Geometry is not changeable"
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