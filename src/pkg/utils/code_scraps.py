from ..geometry.geometry import *
from ..utils.utils import list2dict


##
# @brief add indy_gripper_asm2 mesh and collision boundary for the gripper
# @param gscene     rnb-planning.src.pkg.geometry.geometry.GeometryScene
# @param robot_name full indexed name of the robot
# @param link_name  full name of the link that the gripper will be attached
def add_indy_gripper_asm2(gscene, robot_name, link_name):
    gscene.create_safe(GEOTYPE.MESH, "{}_gripper_vis".format(robot_name), link_name=link_name,
                       dims=(0.1,0.1,0.1), center=(0,0,0), rpy=(0,0,np.pi/2),
                       color=(0.1,0.1,0.1,1), display=True, fixed=True, collision=False,
                       uri="package://my_mesh/meshes/stl/indy_gripper_asm2_res.STL", scale=(1,1,1))

    gscene.create_safe(GEOTYPE.BOX, "{}_gripper".format(robot_name), link_name=link_name,
                       dims=(0.06,0.08,0.06), center=(0,0,0.04), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger1".format(robot_name), link_name=link_name,
                       dims=(0.03,0.03,0.095), center=(0.006,0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger2".format(robot_name), link_name=link_name,
                       dims=(0.03,0.03,0.095), center=(-0.006,0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger3".format(robot_name), link_name=link_name,
                       dims=(0.03,0.03,0.095), center=(0.006,-0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger4".format(robot_name), link_name=link_name,
                       dims=(0.03,0.03,0.095), center=(-0.006,-0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

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
