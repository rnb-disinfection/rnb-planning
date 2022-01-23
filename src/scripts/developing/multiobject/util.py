import numpy as np
from pkg.geometry.geometry import *


# add camera geometry
def add_cam(gscene, tool_link="indy0_tcp"):
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="cam", link_name=tool_link,
                       dims=(0.061, 0.061, 0.026), center=(-0.0785, 0, 0.013), rpy=(0, 0, 0),
                       color=(0.8, 0.8, 0.8, 0.5), display=True, fixed=True, collision=False)

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="cam_col", link_name=tool_link,
                       dims=(0.081, 0.081, 0.046), center=(-0.0785, 0, 0.013), rpy=(0, 0, 0),
                       color=(0.8, 0.8, 0.8, 0.2), display=True, fixed=True, collision=True)

    viewpoint = gscene.create_safe(gtype=GEOTYPE.SPHERE, name="viewpoint", link_name=tool_link,
                                   dims=(0.01, 0.01, 0.01), center=(-0.013, 0, 0), rpy=(0, 0, -np.pi / 2),
                                   color=(1, 0, 0, 0.3), display=True, fixed=True, collision=False, parent="cam")

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="body", link_name=tool_link,
                       dims=(0.067, 0.067, 0.0335), center=(-0.0785, 0, -0.01675), rpy=(0, 0, 0),
                       color=(0.8, 0.8, 0.8, 1), display=True, fixed=True, collision=False)

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="body_col", link_name=tool_link,
                       dims=(0.087, 0.087, 0.0535), center=(-0.0785, 0, -0.01675), rpy=(0, 0, 0),
                       color=(0.8, 0.8, 0.8, 0.2), display=True, fixed=True, collision=True)

    gscene.create_safe(gtype=GEOTYPE.SPHERE, name="backhead", link_name=tool_link,
                       dims=(0.067, 0.067, 0.067), center=(-0.0785, 0, -0.0335), rpy=(0, 0, 0),
                       color=(0.8, 0.8, 0.8, 1), display=True, fixed=True, collision=False)

    gscene.create_safe(gtype=GEOTYPE.SPHERE, name="backhead_col", link_name=tool_link,
                       dims=(0.087, 0.087, 0.087), center=(-0.0785, 0, -0.0335), rpy=(0, 0, 0),
                       color=(0.8, 0.8, 0.8, 0.2), display=True, fixed=True, collision=True)
    return viewpoint


# add object
def add_carrier(gscene, name, carrier_center, carrier_rpy):
    obj_vis = gscene.create_safe(GEOTYPE.MESH, name, link_name="base_link",
                                 dims=(0.1, 0.1, 0.1), center=carrier_center, rpy=carrier_rpy,
                                 color=(0.8, 0.8, 0.8, 1), display=True, fixed=False, collision=True,
                                 uri="package://my_mesh/meshes/stl/carrier.STL", scale=(1e-3, 1e-3, 1e-3))
    return obj_vis


def add_clock(gscene, name, clock_center, clock_rpy):
    obj_vis = gscene.create_safe(GEOTYPE.MESH, name, link_name="base_link",
                                 dims=(0.1, 0.1, 0.1), center=clock_center, rpy=clock_rpy,
                                 color=(0.8, 0.8, 0.8, 1), display=True, fixed=False, collision=True,
                                 uri="package://my_mesh/meshes/stl/tableclock.STL", scale=(1e-3, 1e-3, 1e-3))
    return obj_vis


def add_table(gscene, name, clock_center, clock_rpy):
    obj_vis = gscene.create_safe(GEOTYPE.MESH, name, link_name="base_link",
                                 dims=(0.1, 0.1, 0.1), center=clock_center, rpy=clock_rpy,
                                 color=(0.8, 0.8, 0.8, 1), display=True, fixed=True, collision=True,
                                 uri="package://my_mesh/meshes/stl/table.STL", scale=(1e-3, 1e-3, 1e-3))
    return obj_vis


# move object(pose update)
def move_carrier(gscene, name, carrier_center, carrier_rpy):
    obj_vis = gscene.NAME_DICT[name]
    obj_vis.set_offset_tf(center=carrier_center, orientation_mat=Rot_rpy(carrier_rpy))
    gscene.update_markers_all()


def move_clock(gscene, name, clock_center, clock_rpy):
    obj_vis = gscene.NAME_DICT[name]
    obj_vis.set_offset_tf(center=clock_center, orientation_mat=Rot_rpy(clock_rpy))
    gscene.update_markers_all()


def move_table(gscene, name, table_center, table_rpy):
    obj_vis = gscene.NAME_DICT[name]
    obj_vis.set_offset_tf(center=table_center, orientation_mat=Rot_rpy(table_rpy))
    gscene.update_markers_all()


def pose_refine(obj_type, T, obj_height=0.734):
    T_new = align_z(T)

    center = T_new[:3, 3]
    if obj_type == "suitcase" or obj_type == "dining table":
        center[2] = 0
    elif obj_type == "clock":
        center[2] = obj_height
    rpy = Rot2rpy(T_new[:3, :3])

    return center, rpy



##
# @brief add & update for suitcase, clock
# @param gscene geometry scene
# @param crob combined robot class
# @param obj_type object name (e.g suitcase, clock)
# @param pose_dict detection result dictionary from detector
# @param separate_dict distance to distinguish whether object already detected or not
def add_update_object(gscene, crob, obj_type, pose_dict, separate_dist = 0.5, height = 0):
    # check num of object in the scene
    obj_count = 0
    for i in range(10):
        name = "{}_{:01}".format(obj_type, i)
        if name in gscene.NAME_DICT.keys():
            obj_count += 1
    print("Total {} {} in the scene".format(obj_count, obj_type))

    if obj_count > 0: # If there are objects, add and update objects from detected result
        not_update_list = []
        for name in pose_dict.keys():
            if "_" in name:
                name_cat = name.split("_")[0]
            else:
                name_cat = name
            if name_cat != "suitcase" and name_cat !="clock":
                pass
            else:
                T = pose_dict[name]
                center, rpy = pose_refine(obj_type, T, obj_height=height)

                check_update = False
                for i in range(obj_count):
                    obj_name = "{}_{:01}".format(obj_type, i)
                    T_ = gscene.NAME_DICT[obj_name].get_tf(crob.get_real_robot_pose())
                    center_, rpy_ = pose_refine(obj_type, T_, obj_height=height)

                    if np.linalg.norm(center-center_) < separate_dist: # consider same object
                        if name_cat == "suitcase":
                            move_carrier(gscene, obj_name, center, rpy)
                        elif name_cat == "clock":
                            move_clock(gscene, obj_name, center, rpy)
                        print("Update existing {} in the scene".format(obj_type))
                        check_update = True
                if not check_update:
                    not_update_list.append(name)

        # treat new object
        for name in not_update_list:
            T = pose_dict[name]
            center, rpy = pose_refine(obj_type, T, obj_height=height)
            new_obj_name = "{}_{:01}".format(obj_type, obj_count)
            if name_cat == "suitcase":
                add_carrier(gscene, new_obj_name, center, rpy)
            elif name_cat == "clock":
                add_clock(gscene, new_obj_name, center, rpy)
            print("Add new {} in the scene".format(obj_type))

    else: # At first, there is no object, add all detected object in the scene
        count_tmp = 0
        for name in pose_dict.keys():
            if "_" in name:
                name_cat = name.split("_")[0]
            else:
                name_cat = name
            if name_cat != "suitcase" and name_cat !="clock":
                pass
            else:
                T = pose_dict[name]
                center, rpy = pose_refine(obj_type, T, obj_height=height)

                new_obj_name = "{}_{:01}".format(obj_type, count_tmp)
                if name_cat == "suitcase":
                    add_carrier(gscene, new_obj_name, center, rpy)
                elif name_cat == "clock":
                    add_clock(gscene, new_obj_name, center, rpy)
                count_tmp += 1
                print("Add new {} in the scene".format(obj_type))