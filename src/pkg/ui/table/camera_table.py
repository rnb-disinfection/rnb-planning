# from .table_interface import *
# from ...geometry.builder.scene_builder import *
#
# class CameraTable(TableInterface):
#     HEADS = [IDENTIFY_COL, 'SType', 'Position', 'Direction', 'CameraMatrix', 'Distortion']
#     HILIGHT_KEY = 'camera'
#     CUSTOM_BUTTONS = ["Calibrate"]
#
#     def get_items(self):
#         detector = self.detector
#         T_01 = SE3_inv(cam.ref_tuple[1])
#         T_02 = np.matmul(T_01, cam.T_c12)
#         return [[cam.CAM0_NAME, "Kinect", round_it_str(T_01[:3,3], 4), round_it_str(Rot2rpy(T_01[:3,:3]), 4), round_it_str(cam.kn_config[0].flatten(), 2), round_it_str(cam.kn_config[1].flatten(), 4)],
#                 [cam.CAM1_NAME, "Realsense", round_it_str(T_02[:3,3], 4), round_it_str(Rot2rpy(T_02[:3,:3]), 4), round_it_str(cam.rs_config[0].flatten(), 2), round_it_str(cam.rs_config[1].flatten(), 4)]]
#
#     def get_items_dict(self):
#         return {item[0]: item for item in self.get_items()}
#
#     def serialize(self, gtem):
#         return gtem
#
#     def highlight_item(self, gtem, color=None):
#         pass
#
#     def add_item(self, value):
#         raise(RuntimeError("Cannot add or delete camera"))
#
#     def delete_item(self, active_row):
#         raise(RuntimeError("Cannot add or delete camera"))
#
#     def update_item(self, atem, active_col, value):
#         cam = self.graph.cam
#         res, msg = True, ""
#         if active_col == IDENTIFY_COL:
#             res, msg = False, "cannot change camera name"
#         elif active_col == 'SType':
#             res, msg = False, "cannot change camera type"
#         elif active_col == 'Position':
#             if atem[0] == cam.CAM0_NAME:
#                 T_01 = SE3_inv(cam.ref_tuple[1])
#                 T_02 = np.matmul(T_01, cam.T_c12)
#                 T_01[:3,3] = str_num_it(value)
#                 T_10 = SE3_inv(T_01)
#                 cam.T_c12 = np.matmul(T_10, T_02)
#                 cam.ref_tuple = (cam.ref_tuple[0], T_10)
#             elif atem[0] == cam.CAM1_NAME:
#                 T_01 = SE3_inv(cam.ref_tuple[1])
#                 T_10 = SE3_inv(T_01)
#                 T_02 = np.matmul(T_01, cam.T_c12)
#                 T_02[:3,3] = str_num_it(value)
#                 cam.T_c12 = np.matmul(T_10, T_02)
#             cam.update_cam_coords()
#             self.graph.set_cam_robot_collision()
#             self.graph.gscene.set_rviz()
#         elif active_col == 'Direction':
#             if atem[0] == cam.CAM0_NAME:
#                 T_01 = SE3_inv(cam.ref_tuple[1])
#                 T_02 = np.matmul(T_01, cam.T_c12)
#                 T_01[:3,:3] = Rot_rpy(str_num_it(value))
#                 T_10 = SE3_inv(T_01)
#                 cam.T_c12 = np.matmul(T_10, T_02)
#                 cam.ref_tuple = (cam.ref_tuple[0], T_10)
#             elif atem[0] == cam.CAM1_NAME:
#                 T_01 = SE3_inv(cam.ref_tuple[1])
#                 T_10 = SE3_inv(T_01)
#                 T_02 = np.matmul(T_01, cam.T_c12)
#                 T_02[:3,:3] = Rot_rpy(str_num_it(value))
#                 cam.T_c12 = np.matmul(T_10, T_02)
#             cam.update_cam_coords()
#             self.graph.set_cam_robot_collision()
#             self.graph.gscene.set_rviz()
#         elif active_col == 'CameraMatrix':
#             if atem[0] == cam.CAM0_NAME:
#                 cam.kn_config = (np.array(str_num_it(value)).reshape(3,3), cam.kn_config[1])
#             elif atem[0] == cam.CAM1_NAME:
#                 cam.rs_config = (np.array(str_num_it(value)).reshape(3,3), cam.rs_config[1])
#         elif active_col == 'Distortion':
#             if atem[0] == cam.CAM0_NAME:
#                 cam.kn_config = (cam.kn_config[0], np.array(str_num_it(value)))
#             elif atem[0] == cam.CAM1_NAME:
#                 cam.rs_config = (cam.rs_config[0], np.array(str_num_it(value)))
#         return res, msg
#
#     def button(self, button, *args, **kwargs):
#         print("button clicked")
#         if button == TAB_BUTTON.CUSTOM:
#             if args[0]:
#                 print("============== start calibration =================")
#                 self.graph.cam.calibrate(self.graph.gscene)
#                 self.graph.set_cam_robot_collision()
#                 self.graph.gscene.set_rviz()
#                 print("============== finish calibration =================")
#             else:
#                 print("Unknown button")
#         else:
#             TableInterface.button(self, button, *args, **kwargs)
#         print("button action done")