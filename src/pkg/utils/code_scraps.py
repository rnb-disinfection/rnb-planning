from ..geometry.geometry import *
from ..utils.utils import list2dict
from ..planning.constraint.constraint_subject import *
from ..planning.constraint.constraint_actor import PlacePlane


##
# @brief add indy_gripper_asm2 mesh and collision boundary for the gripper
# @param gscene     rnb-planning.src.pkg.geometry.geometry.GeometryScene
# @param robot_name full indexed name of the robot
def add_indy_gripper_asm2(gscene, robot_name):
    gscene.create_safe(GEOTYPE.MESH, "{}_gripper_vis".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.1,0.1,0.1), center=(0,0,0), rpy=(0,0,np.pi/2),
                       color=(0.1,0.1,0.1,1), display=True, fixed=True, collision=False,
                       uri="package://my_mesh/meshes/stl/indy_gripper_asm2_res.STL", scale=(1,1,1))

    gscene.create_safe(GEOTYPE.BOX, "{}_gripper".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.06,0.08,0.06), center=(0,0,0.04), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger1".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.03,0.03,0.095), center=(0.006,0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger2".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.03,0.03,0.095), center=(-0.006,0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger3".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.03,0.03,0.095), center=(0.006,-0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger4".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.03,0.03,0.095), center=(-0.006,-0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)


##
# @brief add add_sweep_tool to indy
# @param gscene     rnb-planning.src.pkg.geometry.geometry.GeometryScene
# @param robot_name full indexed name of the robot
def add_indy_sweep_tool(gscene, robot_name, face_name="brush_face"):
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="{}_fts".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.02), dims=(0.07, 0.07, 0.04), rpy=(0, 0, 0), color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="{}_fts_col".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.02), dims=(0.11, 0.11, 0.04), rpy=(0, 0, 0), color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="{}_pole".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.055), dims=(0.03, 0.03, 0.030), rpy=(0, 0, 0), color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="{}_pole_col".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.055), dims=(0.07, 0.07, 0.030), rpy=(0, 0, 0), color=(0.0, 0.8, 0.0, 0.2),
                       collision=True, fixed=True)

    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_brushbase".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.0775), dims=(0.06, 0.14, 0.015), rpy=(0, 0, 0), color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name=face_name, link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.09), dims=(0.05, 0.13, 0.02), rpy=(0, 0, 0), color=(1.0, 1.0, 0.94, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_col".format(face_name), link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.08), dims=(0.08, 0.15, 0.03), rpy=(0, 0, 0), color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)


def finish_L_shape(gscene, gtem_dict):
    if "l_shape" in gtem_dict:
        l_center = gtem_dict["l_shape"]
        gscene.create_safe(gtype=GEOTYPE.BOX, name="l_shape_x", link_name=l_center.link_name,
                           center=(l_center.dims[0], 0.0, 0.0), dims=l_center.dims, rpy=(0, 0, 0), color=l_center.color,
                           collision=l_center.collision, fixed=l_center.fixed, parent="l_shape")
        gscene.create_safe(gtype=GEOTYPE.BOX, name="l_shape_z", link_name=l_center.link_name,
                           center=(0.0, 0.0, l_center.dims[2]), dims=l_center.dims, rpy=(0, 0, 0), color=l_center.color,
                           collision=l_center.collision, fixed=l_center.fixed, parent="l_shape")


##
# @brief remove place points except for the current one
def use_current_place_point_only(pscene, current_state):
    for oname, aname, bname, bgname in current_state.binding_state:
        obj = pscene.subject_dict[oname]
        if isinstance(obj, AbstractObject):
            pp_list = [ap for ap in obj.action_points_dict.values() if isinstance(ap, PlacePoint)]
            for pp in pp_list:
                if pp.name != aname:
                    del obj.action_points_dict[pp.name]
    pscene.update_subjects()


##
# @brief remove attached binders on objects except for the current one
def use_current_sub_binders_only(pscene, current_state):
    active_binders  = [binding[2] for binding in current_state.binding_state if binding[2] is not None]

    for obj in pscene.subject_dict.values():
        for bname, binder in pscene.actor_dict.items():
            if binder.geometry.name in obj.geometry.get_family() and bname not in active_binders:
                pscene.remove_binder(bname)

##
# @brief set action points for l_shape object
def set_l_shape_object(pscene):
    if "l_shape" in pscene.subject_dict:
        l_sub = pscene.subject_dict["l_shape"]
        l_sub.action_points_dict = {}
        l_sub.add_place_points(l_sub.geometry)
        for gname in l_sub.geometry.children:
            child = pscene.gscene.NAME_DICT[gname]
            if child.collision and child.display:
                l_sub.add_grip_points(child, GRASP_DEPTH_MIN=0.015)
                l_sub.register_binders(pscene, PlacePlane, geometry=child)
        l_sub.set_conflict_dict()

import requests
from bs4 import BeautifulSoup
from time import sleep

def switch_command(ip_addr, on_off, UI_PORT=9990):
    uri = "http://{ip_addr}:{UI_PORT}/param_setting?control_force0={on_off}".format(ip_addr=ip_addr, UI_PORT=UI_PORT, on_off=int(on_off))
    print(uri)
    requests.get(uri)

class ModeSwitcher:
    def __init__(self, pscene):
        self.pscene = pscene
        self.crob = pscene.combined_robot
        self.switch_delay = 0.5

    def switch_in(self, snode_pre, snode_new):
        indy = self.crob.robot_dict['indy0']
        switch_state = False
        for n1, n2 in zip(snode_pre.state.node, snode_new.state.node):
            if n1 == 1 and n2 == 2:
                switch_state = True
                break
        if switch_state:
            sleep(self.switch_delay)
            with indy:
                indy.stop_tracking()
            sleep(self.switch_delay)
            switch_command(indy.server_ip, True)
            sleep(self.switch_delay)
        return switch_state

    def switch_out(self, switch_state, snode_new):
        indy = self.crob.robot_dict['indy0']
        if switch_state:
            sleep(self.switch_delay)
            with indy:
                indy.stop_tracking()
            sleep(self.switch_delay)
            switch_command(indy.server_ip, False)
            sleep(self.switch_delay)
            indy.joint_move_make_sure(snode_new.traj[-1][self.crob.idx_dict["indy0"]])


### resized image plot
# ratio = 1.0/3
# color_image_tmp = cv2.resize(color_image, dsize=None, fx=ratio, fy=ratio)
# cam_tmp = kn_config[0].copy()
# cam_tmp[0,0] *= ratio
# cam_tmp[1,1] *= ratio
# cam_tmp[1,2] *= ratio
# cam_tmp[1,2] *= ratio
# corner_dict_tmp = {k: v*ratio for k, v in corner_dict.items()}
#
# plt.figure(figsize=(25,15))
# color_image_out = draw_objects(color_image_tmp, aruco_map, {}, corner_dict_tmp, cam_tmp, kn_config[1], axis_len=0.1)#objectPose_dict, corner_dict
# plt.imshow(color_image_out[:,:,[2,1,0]])


### plot xy error bar
# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.collections import PatchCollection
# from matplotlib.patches import Rectangle
#
# # Number of data points
# n=5
#
# # Dummy data
# x=np.arange(0,n,1)
# y=np.random.rand(n)*5.
#
# # Dummy errors (above and below)
# xerr=np.random.rand(2,n)
# yerr=np.random.rand(2,n)
#
# # Create figure and axes
# fig,ax = plt.subplots(1)
#
# # Plot data points
# ax.errorbar(x,y,xerr=xerr,yerr=yerr,fmt='None',ecolor='k')
#
# # Function to plot error boxes
# def makeErrorBoxes(xdata,ydata,xerror,yerror,fc='r',ec='None',alpha=0.5):
#
#     # Create list for all the error patches
#     errorboxes = []
#
#     # Loop over data points; create box from errors at each point
#     for xc,yc,xe,ye in zip(xdata,ydata,xerror.T,yerror.T):
#         rect = Rectangle((xc-xe[0],yc-ye[0]),xe.sum(),ye.sum())
#         errorboxes.append(rect)
#
#     # Create patch collection with specified colour/alpha
#     pc = PatchCollection(errorboxes,facecolor=fc,alpha=alpha,edgecolor=ec)
#
#     # Add collection to axes
#     ax.add_collection(pc)
#
# # Call function to create error boxes
# makeErrorBoxes(x,y,xerr,yerr)
#
# # Add some space around the data points on the axes
# ax.margins(0.1)
#
# plt.show()



### compare single/stereo camera measurement error
#
# h_st_vec = []
# h_kn_vec = []
# for _ in range(30):
#     time.sleep(1)
#     kn_config, rs_config, T_c12 = calibrate_stereo(aruco_map, dictionary)
#
#     xyz_rpy_robots, xyz_rvec_cams, env_gen_dict, objectPose_dict, corner_dict, color_image  = \
#         detect_environment(
#             aruco_map, dictionary, robot_tuples=ROBOTS_ON_SCENE,
#             env_dict={'floor': CallHolder(GeoBox, ["center", "orientation"], BLH=(1.52,0.72,0.01)),
#                       'wall':CallHolder(GeoBox, ["center", "orientation"], BLH=(3,3,0.01))},
#             camT_dict={"cam0":np.identity(4), "cam1": T_c12},
#             ref_name='floor')
#
#     h_st_vec.append(np.matmul(SE3_inv(objectPose_dict["floor"]), objectPose_dict["box1"])[2,3])
#     h_st_vec.append(np.matmul(SE3_inv(objectPose_dict["floor"]), objectPose_dict["box2"])[2,3])
#     h_st_vec.append(np.matmul(SE3_inv(objectPose_dict["floor"]), objectPose_dict["box3"])[2,3])
#
#     color_image = get_kn_image()
#     objectPose_dict_kn, corner_dict_kn = get_object_pose_dict(color_image, aruco_map, dictionary, *kn_config)
#
#     h_kn_vec.append(np.matmul(SE3_inv(objectPose_dict_kn["floor"]), objectPose_dict_kn["box1"])[2,3])
#     h_kn_vec.append(np.matmul(SE3_inv(objectPose_dict_kn["floor"]), objectPose_dict_kn["box2"])[2,3])
#     h_kn_vec.append(np.matmul(SE3_inv(objectPose_dict_kn["floor"]), objectPose_dict_kn["box3"])[2,3])
#     print("="*100)
#     print("h_st_vec: {}/{}".format(round(np.mean(h_st_vec)*1000, 2), round(np.std(h_st_vec)*1000, 2)))
#     print("h_kn_vec: {}/{}".format(round(np.mean(h_kn_vec)*1000, 2), round(np.std(h_kn_vec)*1000, 2)))
#
# # [30, 100, 170]
# # ====================================================================================================
# # h_st_vec: 172.15/0.83 (173.56/171.1)
# # h_kn_vec: 166.7/1.42 (168.63/164.57)
# # ====================================================================================================
# # h_st_vec: 101.73/0.43 (102.57/100.92)
# # h_kn_vec: 96.82/1.76 (99.0/94.07)
# # ====================================================================================================
# # h_st_vec: 30.93/0.91 (32.37/29.64)
# # h_kn_vec: 26.81/2.57 (29.57/22.88)
