import os
import sys
import numpy as np
sys.path.append(os.path.join(os.path.join(
    os.environ["RNB_PLANNING_DIR"], 'src')))
from pkg.geometry.geometry import *

def add_panda_cam(gscene, tool_link, theta):
    gscene.create_safe(gtype=GEOTYPE.MESH, name="panda_cam_mount_vis", link_name=tool_link, dims=(0.1,0.1,0.1), 
                       center=(0,0,0), rpy=(0,0,theta), display=True, color=(0.8,0.8,0.8,1), collision=False, fixed=True,
                      uri="package://my_mesh/meshes/stl/cam_mount_v3_res.STL")
    gscene.create_safe(gtype=GEOTYPE.BOX, name="panda_cam_mount_col", link_name=tool_link, dims=(0.061,0.061,0.06), 
                       center=(0.0,0.08,-0.015), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_cam_mount_vis")
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="panda_cam_body", link_name=tool_link, dims=(0.061,0.061,0.026), 
                       center=(0.0,0.12,-0.015), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,1), collision=False, fixed=True,
                       parent="panda_cam_mount_vis")
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="panda_cam_body_col", link_name=tool_link, dims=(0.081,0.081,0.046), 
                       center=(0.0,0.0,0.0), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_cam_body")

    viewpoint = gscene.create_safe(gtype=GEOTYPE.SPHERE, name="viewpoint", link_name=tool_link, dims=(0.01, 0.01, 0.01),
                                   center=(0, (0.03115 + 0.014 - 0.0305), 0.013 - 0.012),rpy=(0, 0, np.pi), color=(1, 0, 0, 0.3),
                                   display=True, fixed=True, collision=False, parent="panda_cam_body")
    return viewpoint
    
def add_panda_brush(gscene, tool_link, theta, brush_name, offset=(0,0,0.011), tool_dims=(0.09,0.175,0.05), col_margin=0.01):
    gscene.create_safe(gtype=GEOTYPE.MESH, name="panda_tool_vis", link_name=tool_link, dims=(0.1,0.1,0.1), 
                       center=offset, rpy=(0,0,theta-np.pi/2), display=True, color=(0.8,0.8,0.8,1), collision=False, fixed=True,
                       uri="package://my_mesh/meshes/stl/WipingTool_res.STL")
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="panda_tool_mount_col", link_name=tool_link, dims=(0.08,0.08,0.03), 
                       center=(0.0,0.0,0.0), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_tool_vis")
    gscene.create_safe(gtype=GEOTYPE.BOX, name="panda_tool_root_col", link_name=tool_link, dims=(0.05,0.05,0.04), 
                       center=(0,0.0,0.02), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_tool_vis")
    gscene.create_safe(gtype=GEOTYPE.BOX, name="panda_tool_rod_col", link_name=tool_link, dims=(0.05,0.05,0.25), 
                       center=(0.088,0.0,0.11), rpy=(0,np.pi/4,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_tool_vis")
    gscene.create_safe(gtype=GEOTYPE.BOX, name="panda_tool_rod_col", link_name=tool_link, dims=(0.05,0.05,0.25), 
                       center=(0.088,0.0,0.11), rpy=(0,np.pi/4,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent="panda_tool_vis")
    brush_face = gscene.create_safe(gtype=GEOTYPE.BOX, name=brush_name, link_name=tool_link, dims=tool_dims, 
                       center=(0.22,0.0,0.192), rpy=(0,-np.pi/2,0), display=True, color=(0.8,0.8,0.0,0.9), collision=False, fixed=True,
                       parent="panda_tool_vis")
    gscene.create_safe(gtype=GEOTYPE.BOX, name=brush_name+"_col", link_name=tool_link, dims=np.add(tool_dims, (col_margin,col_margin,0)),
                       center=(0, 0, col_margin), rpy=(0,0,0), display=True, color=(0.8,0.8,0.8,0.2), collision=True, fixed=True,
                       parent=brush_name)
    return brush_face

import numpy as np

# add object
def add_carrier(gscene, name, carrier_center, carrier_rpy):
    obj_vis = gscene.create_safe(GEOTYPE.MESH, name, link_name="base_link",
                                 dims=(0.1, 0.1, 0.1), center=carrier_center, rpy=carrier_rpy,
                                 color=(0.8, 0.8, 0.8, 1), display=True, fixed=False, collision=False,
                                 uri="package://my_mesh/meshes/stl/carrier_centered_m_scale.STL", scale=(1., 1., 1.))

    obj_col = gscene.create_safe(GEOTYPE.BOX, "{}_col".format(name), link_name="base_link",
                                 dims=(0.4+0.02, 0.29+0.02, 0.635), center=(0,0,0), rpy=(0,0,0),
                                 color=(0, 0, 0, 0.1), display=True, fixed=False, collision=True,
                                 parent="{}".format(name))

    return obj_vis


def add_clock(gscene, name, clock_center, clock_rpy):
    obj_vis = gscene.create_safe(GEOTYPE.MESH, name, link_name="base_link",
                                 dims=(0.1, 0.1, 0.1), center=clock_center, rpy=clock_rpy,
                                 color=(0.8, 0.8, 0.8, 1), display=True, fixed=False, collision=False,
                                 uri="package://my_mesh/meshes/stl/tableclock_centered_m_scale.STL", scale=(1., 1., 1.))

    obj_col = gscene.create_safe(GEOTYPE.BOX, "{}_col".format(name), link_name="base_link",
                                 dims=(0.138+0.01, 0.05+0.01, 0.078), center=(0,0,0), rpy=(0,0,0),
                                 color=(0, 0, 0, 0.1), display=True, fixed=False, collision=True,
                                 parent="{}".format(name))

    return obj_vis


def add_table(gscene, name, table_center, table_rpy):
    vis_name = name + "_vis"
    obj_vis = gscene.create_safe(GEOTYPE.MESH, vis_name, link_name="base_link",
                                 dims=(0.1, 0.1, 0.1), center=table_center, rpy=table_rpy,
                                 color=(0.8, 0.8, 0.8, 1), display=True, fixed=True, collision=False,
                                 uri="package://my_mesh/meshes/stl/table_floor_centered_m_scale.STL", scale=(1., 1., 1.))

    obj_body = gscene.create_safe(GEOTYPE.BOX, name, link_name="base_link",
                                 dims=(1.6+0.14, 0.8+0.14, 0.02), center=(0,0,0.715), rpy=(0,0,0),
                                 color=(0, 0, 0, 0.1), display=True, fixed=False, collision=True,
                                 parent=vis_name)

    obj_back_col = gscene.create_safe(GEOTYPE.BOX, "{}_back_col".format(name), link_name="base_link",
                                 dims=(1.6+0.14, 0.14, 0.705+0.1), center=(0,0.4-0.05,0.705/2), rpy=(0,0,0),
                                 color=(0, 0, 0, 0.1), display=True, fixed=False, collision=True,
                                 parent=vis_name)

    obj_left_leg_col = gscene.create_safe(GEOTYPE.BOX, "{}_left_leg_col".format(name), link_name="base_link",
                                 dims=(0.1, 0.8+0.1, 0.705+0.1), center=(-0.8+0.05,0,0.705/2), rpy=(0,0,0),
                                 color=(0, 0, 0, 0.1), display=True, fixed=False, collision=True,
                                 parent=vis_name)

    obj_right_leg_col = gscene.create_safe(GEOTYPE.BOX, "{}_right_leg_col".format(name), link_name="base_link",
                                          dims=(0.1, 0.8+0.1, 0.705+0.1), center=(0.8-0.05, 0, 0.705/2), rpy=(0, 0, 0),
                                          color=(0, 0, 0, 0.1), display=True, fixed=False, collision=True,
                                          parent=vis_name)

    return obj_vis, obj_body


def add_chair(gscene, name, chair_center, chair_rpy):
    obj_vis = gscene.create_safe(GEOTYPE.MESH, name, link_name="base_link",
                                 dims=(0.1, 0.1, 0.1), center=chair_center, rpy=chair_rpy,
                                 color=(0.8, 0.8, 0.8, 1), display=True, fixed=False, collision=False,
                                 uri="package://my_mesh/meshes/stl/chair_floor_centered_m_scale.STL", scale=(1., 1., 1.))

    obj_col = gscene.create_safe(GEOTYPE.BOX, "{}_col".format(name), link_name="base_link",
                                 dims=(0.37+0.02, 0.37+0.02, 0.455), center=(0,0,0.455/2), rpy=(0,0,0),
                                 color=(0, 0, 0, 0.1), display=True, fixed=False, collision=True,
                                 parent=name)

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


def move_chair(gscene, name, chair_center, chair_rpy):
    obj_vis = gscene.NAME_DICT[name]
    obj_vis.set_offset_tf(center=chair_center, orientation_mat=Rot_rpy(chair_rpy))
    gscene.update_markers_all()


def pose_refine(obj_type, T, obj_height=0.725):
    # rotation constraint
    T_new = align_z(T)
    rpy = Rot2rpy(T_new[:3, :3])

    # height constraint
    center = T_new[:3, 3]
    if obj_type == "dining table" or obj_type == "chair":
        center[2] = 0
    elif obj_type == "suitcase" or obj_type == "clock":
        center[2] = obj_height

    if obj_type == "clock":
        rpy = Rot2rpy(np.matmul(T_new[:3, :3], Rot_axis(1, -np.pi / 2)))

    return center, rpy

##
# @brief add & update for suitcase, clock
# @param gscene geometry scene
# @param crob combined robot class
# @param obj_type object name (e.g suitcase, clock)
# @param pose_dict detection result dictionary from detector
# @param separate_dict distance to distinguish whether object already detected or not
# @param height height of floor fitting
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
            if name_cat != "suitcase" and name_cat !="clock" and name_cat != "chair":
                pass
            else:
                T = pose_dict[name]
                center, rpy = pose_refine(obj_type, T, obj_height=height)

                check_update = False # True if same on scene
                for i in range(obj_count):
                    obj_name = "{}_{:01}".format(obj_type, i)
                    T_ = gscene.NAME_DICT[obj_name].get_tf(crob.home_pose)
                    center_, rpy_ = pose_refine(obj_type, T_, obj_height=height)

                    if np.linalg.norm(center-center_) < separate_dist: # consider same object
                        if name_cat == "chair":
                            move_chair(gscene, obj_name, center, rpy)
                        elif name_cat == "suitcase":
                            move_carrier(gscene, obj_name, center, rpy)
                        elif name_cat == "clock":
                            move_clock(gscene, obj_name, center, rpy)
                        print("Update existing {} in the scene".format(obj_type))
                        check_update = True
                        break
                if not check_update:
                    not_update_list.append(name)

        # treat new object
        for name in not_update_list:
            T = pose_dict[name]
            center, rpy = pose_refine(obj_type, T, obj_height=height)
            new_obj_name = "{}_{:01}".format(obj_type, obj_count)
            if name_cat == "chair":
                add_chair(gscene, new_obj_name, center, rpy)
            elif name_cat == "suitcase":
                add_carrier(gscene, new_obj_name, center, rpy)
            elif name_cat == "clock":
                add_clock(gscene, new_obj_name, center, rpy)
            print("Add new {} in the scene".format(obj_type))
            obj_count += 1

    else: # At first, there is no object, add all detected object in the scene
        count_tmp = 0
        for name in pose_dict.keys():
            if "_" in name:
                name_cat = name.split("_")[0]
            else:
                name_cat = name
            if name_cat != "suitcase" and name_cat !="clock" and name_cat != "chair":
                pass
            else:
                T = pose_dict[name]
                center, rpy = pose_refine(obj_type, T, obj_height=height)

                new_obj_name = "{}_{:01}".format(obj_type, count_tmp)
                if name_cat == "chair":
                    add_chair(gscene, new_obj_name, center, rpy)
                elif name_cat == "suitcase":
                    add_carrier(gscene, new_obj_name, center, rpy)
                elif name_cat == "clock":
                    add_clock(gscene, new_obj_name, center, rpy)
                count_tmp += 1
                print("Add new {} in the scene".format(obj_type))
