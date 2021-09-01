import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import cv2
import os
import sys
import shutil
import json
import SharedArray as sa
import time

from perception_config import *
from pyransac3d import *


####################### JSON NUMPYENCODER #######################
class NumpyEncoder(json.JSONEncoder):
    """ Special json encoder for numpy types """
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


geometry_list = [Cuboid(), Sphere(), Cylinder()]

####################### Util functions #######################
def make_dataset_folder(path_folder):
    if not os.path.exists(path_folder):
        os.makedirs(path_folder)
        os.makedirs(path_folder + '/depth')
        os.makedirs(path_folder + '/color')
    else:
        user_input = raw_input("%s not empty. Overwrite? (y/n) : " % path_folder)
        if user_input.lower() == 'y':
            shutil.rmtree(path_folder)
            os.makedirs(path_folder)
            os.makedirs(path_folder + '/depth')
            os.makedirs(path_folder + '/color')
            return True
        else:
            return False


def save_intrinsic_as_json(filename, frame, d_scale):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    with open(filename, 'w') as outfile:
        obj = json.dump(
            {
                'color_format':
                    "RGB8",
                'depth_format':
                    "Z16",
                'depth_scale':
                    1/d_scale,
                'device_name':
                    "Intel RealSense L515",
                'fps':
                    30.0,
                'width':
                    intrinsics.width,
                'height':
                    intrinsics.height,
                'intrinsic_matrix': [
                    intrinsics.fx, 0.0, 0.0, 0.0, intrinsics.fy, 0.0, intrinsics.ppx,
                    intrinsics.ppy, 1.0
                ],
                'serial_number':
                    "f0271852"
            },
            outfile,
            indent=8)


def read_intrinsic_from_json(filename):
    with open(filename, 'r') as f:
        json_data = json.load(f)
    cam_width = json_data['width']
    cam_height = json_data['height']
    cam_fx = json_data['intrinsic_matrix'][0]
    cam_fy = json_data['intrinsic_matrix'][4]
    cam_ppx = json_data['intrinsic_matrix'][6]
    cam_ppy = json_data['intrinsic_matrix'][7]
    depth_scale = json_data['depth_scale']
    return cam_width, cam_height, cam_fx, cam_fy, cam_ppx, cam_ppy, depth_scale


def load_camera_trajectory(traj_log):
    cam_poses = []
    with open(traj_log) as f:
        content = f.readlines()

        # Load .log file.
        for i in range(0, len(content), 5):
            # format %d (src) %d (tgt) %f (fitness)
            data = list(map(float, content[i].strip().split(' ')))
            ids = (int(data[0]), int(data[1]))
            fitness = data[2]

            # format %f x 16
            T_gt = np.array(
                list(map(float, (''.join(
                    content[i + 1:i + 5])).strip().split()))).reshape((4, 4))

            cam_poses.append(T_gt)
    return cam_poses



####################### Core functions #######################
class Camera_parameter:
    def __init__(self, intrins_path, trajectory_log):
        cam_width_, cam_height_, cam_fx_, cam_fy_, cam_ppx_, cam_ppy_, depth_scale_ = read_intrinsic_from_json(
            intrins_path)
        cam_traj_ = load_camera_trajectory(trajectory_log)
        self.cam_width = cam_width_
        self.cam_height = cam_height_
        self.cam_fx = cam_fx_
        self.cam_fy = cam_fy_
        self.cam_ppx = cam_ppx_
        self.cam_ppy = cam_ppy_
        self.depth_scale = depth_scale_
        self.cam_traj = cam_traj_

    def get_intrinsic(self):
        intrinsic = np.array([[self.cam_fx, 0.0, self.cam_ppx],
                              [0.0, self.cam_fy, self.cam_ppy],
                              [0.0, 0.0, 1.0]])
        return intrinsic

    def get_extrinsic(self):
        return self.cam_traj



def streaming(path_folder):
    # Make save directory
    if (make_dataset_folder(path_folder)):
        # Create a pipeline
        pipeline = rs.pipeline()
        # Create a config and configure the pipeline to stream
        config = rs.config()

        #     # Get device product line for setting a supporting resolution
        #     pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        #     pipeline_profile = config.resolve(pipeline_wrapper)
        #     device = pipeline_profile.get_device()
        #     device_product_line = str(device.get_info(rs.camera_info.product_line))

        #     if device_product_line == 'L500':
        #         config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        #         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #     else:
        #         config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        #         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start streaming
        profile = pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        # Set Preset option
        depth_sensor.set_option(rs.option.visual_preset, 3)
        depth_sensor.set_option(rs.option.laser_power, 88)
        depth_sensor.set_option(rs.option.noise_filtering, 4)
        depth_sensor.set_option(rs.option.receiver_gain, 17)
        depth_sensor.set_option(rs.option.post_processing_sharpening, 2.0)
        depth_sensor.set_option(rs.option.pre_processing_sharpening, 0.5)

        #  Not display the baground more than clipping_distance
        clipping_distance_in_meters = 2.0  # 2 meter
        clipping_distance = clipping_distance_in_meters / depth_scale

        # Align depth to color
        align_to = rs.stream.color
        align = rs.align(align_to)

        # Streaming loop
        frame_count = 0
        poses = []
        first_pose = np.identity(4)
        try:
            while True:
                # Get frameset of color and depth
                frames = pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # Get aligned frames
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                # # Get intrinsic parameters
                # intrins = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
                # cameraMatrix = np.array([[intrins.fx, 0, intrins.ppx],
                #                          [0, intrins.fy, intrins.ppy],
                #                          [0, 0, 1]])
                # distCoeffs = np.array(intrins.coeffs)

                # Validate that both frames are valid
                if not aligned_depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(aligned_depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Temporary save camera pose
                #             amap = get_aruco_map()
                #             objectPose_dict, corner_dict = amap.get_object_pose_dict(color_image, cameraMatrix, distCoeffs)
                if (frame_count == 0):
                    #                 poses.append(np.identity(4))
                    #                 first_pose = objectPose_dict.values()[0]
                    save_intrinsic_as_json(path_folder + "/intrinsic.json", color_frame, depth_scale)
                #             else :
                #                 pose = np.matmul(first_pose, np.linalg.inv(objectPose_dict.values()[0]))
                #                 poses.append(pose)

                # Save color, depth image
                #             cv2.imwrite("_%s/%d.png" % \
                #                         (path_folder + "/depth/depth", frame_count), depth_image)
                #             cv2.imwrite("_%s/%d.jpg" % \
                #                         (path_folder + "/color/color", frame_count), color_image)
                cv2.imwrite(path_folder + "/depth/depth_{}.png".format(frame_count), depth_image)
                cv2.imwrite(path_folder + "/color/color_{}.jpg".format(frame_count), color_image)
                #             print("Saved color + depth image %d" % frame_count)
                frame_count += 1

                # Remove background - Set pixels further than clipping_distance to grey
                grey_color = 153
                # depth image is 1 channel, color is 3 channels
                depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
                bg_removed = np.where((depth_image_3d > clipping_distance) | \
                                      (depth_image_3d <= 0), grey_color, color_image)

                # Render images
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
                images = np.hstack((bg_removed, depth_colormap))
                cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Recorder Realsense', images)
                key = cv2.waitKey(1)

                # if 'esc' button pressed, escape loop and exit streaming
                if key == 27:
                    cv2.destroyAllWindows()
                    break
        finally:
            print("Stop the streaming and Save RGBD images\n")
            pipeline.stop()

    else:
        pass



def getPCDAll(img_num, depth_seg_path, pcd_path, Camera_parameter, detected_class):
    cam_width = Camera_parameter.cam_width
    cam_height = Camera_parameter.cam_height
    cam_fx = Camera_parameter.cam_fx
    cam_fy = Camera_parameter.cam_fy
    cam_ppx = Camera_parameter.cam_ppx
    cam_ppy = Camera_parameter.cam_ppy
    d_scale = Camera_parameter.depth_scale
    cam_traj = Camera_parameter.cam_traj

    obj_pcd_num = 0
    pcd = o3d.io.read_point_cloud(pcd_path)
    for i in range(len(detected_class)):
        temp_path = depth_seg_path + '/' + valid_class_dict[detected_class[i]]
        for j in range(len(os.listdir(temp_path))):
            # Consider only segmentation result number is over 80% of total image number
            if (len(os.listdir(temp_path + '/{}'.format(j + 1))) > int(img_num * 0.8)):
                pcd_overlap = o3d.geometry.PointCloud()
                for k in range(img_num // 3):
                    l = k * 4
                    if (os.path.exists(temp_path + '/{}'.format(j + 1) + '/depth_mask_{}.png'.format(l))):
                        img_s = o3d.io.read_image(temp_path + '/{}'.format(j + 1) + '/depth_mask_{}.png'.format(l))
                        pcd_s = o3d.geometry.PointCloud.create_from_depth_image(img_s,
                                                    o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                            cam_height,cam_fx, cam_fy,
                                                                            cam_ppx, cam_ppy), np.linalg.inv(cam_traj[l]),
                                                                            depth_scale= d_scale)

                        d_trunc = np.linalg.norm(pcd_s.get_center()) * 1.2
                        pcd_s = o3d.geometry.PointCloud.create_from_depth_image(img_s,
                                                    o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                            cam_height, cam_fx, cam_fy,
                                                                            cam_ppx, cam_ppy), np.linalg.inv(cam_traj[l]),
                                                                            depth_scale = d_scale, depth_trunc = d_trunc)
                        source = pcd_s
                        dists = source.compute_point_cloud_distance(pcd)

                        dists = np.array(dists)
                        thres = 0.003
                        indices = np.where(dists < thres)[0]
                        # pcd_overlap = pcd.select_by_index(indices)

                        p_overlap = []
                        for k in range(len(indices)):
                            p_overlap.append(source.points[indices[k]])

                        pcd_temp = o3d.geometry.PointCloud()
                        pcd_temp.points = o3d.utility.Vector3dVector(p_overlap)
                        pcd_temp.uniform_down_sample(every_k_points=9)
                        pcd_overlap = pcd_overlap + pcd_temp
                        pcd_overlap.uniform_down_sample(every_k_points = 15)
                    else:
                        pass

                #     o3d.visualization.draw_geometries([pcd_overlap])
                o3d.io.write_point_cloud(WORKING_DIR + "/object_{}.pcd".format(obj_pcd_num), pcd_overlap)
                obj_pcd_num += 1
    return obj_pcd_num


def getGPDAll(obj_pcd_num):
    grasp_list_all = []
    for i in range(obj_pcd_num):
        grasp_poses = detect_from_server(i)
        time.sleep(0.05)
        grasp_list_all.append(grasp_poses)

    return grasp_list_all



def append_geometry_list(result, geometry_type_list, geometry_inform_list):
    if (len(result) == 5):
        geometry_type_list.append('Cylinder')
        geometry_inform_list.append([result[0], result[1], result[2], result[3]])
    elif (len(result) == 4):
        geometry_type_list.append('Cuboid')
        geometry_inform_list.append([result[0], result[1], result[2]])
    elif (len(result) == 3):
        geometry_type_list.append('Sphere')
        geometry_inform_list.append([result[0], result[1]])



def extract_outliers(pcd_points, inliers):
    temp = []
    for i in range(len(pcd_points)):
        idx = i
        if not idx in inliers:
            temp.append(pcd_points[idx])

    p_outliers = np.zeros((len(temp), 3))
    for i in range(len(temp)):
        p_outliers[i] = temp[i]

    return p_outliers



def save_obj_geo_json(geometry_type_list, geometry_inform_list, num):
    json_data = {}
    for i in range(len(geometry_type_list)):
        json_data[i + 1] = geometry_inform_list[i]
        print(geometry_type_list[i])
        print(geometry_inform_list[i])

    with open(WORKING_DIR + '/obj_geometry_{}.json'.format(num), 'w') as outfile:
        obj = json.dump(json_data, outfile, cls=NumpyEncoder)


def feature_matching(pcd_points):
    total = len(pcd_points)

    # Cuboid case
    # Pyransac3D cuboid fitting only return 3 plane eq, So apply fitting twice
    eq11, inliers11 = geometry_list[0].fit(pcd_points, thresh=0.02, maxIteration=1300)
    temp_outlier = extract_outliers(pcd_points, inliers11)
    eq12, inliers12 = geometry_list[0].fit(temp_outlier, thresh=0.02, maxIteration=1300)
    p_outliers1 = extract_outliers(temp_outlier, inliers12)

    # Calculate first fitting inliers
    p1_temp = []
    for i in range(len(pcd_points)):
        idx = i
        if not idx in inliers11:
            p1_temp.append(pcd_points[idx])
    p1 = np.zeros((len(p1_temp), 3))
    for i in range(len(p1_temp)):
        p1[i] = p1_temp[i]

    # Calculate second fitting inliers
    p2_temp = []
    for i in range(len(temp_outlier)):
        idx = i
        if not idx in inliers12:
            p2_temp.append(pcd_points[idx])
    p2 = np.zeros((len(p2_temp), 3))
    for i in range(len(p2_temp)):
        p2[i] = p2_temp[i]

    center11 = np.array([np.mean(p1[:,0]), np.mean(p1[:,1]), np.mean(p1[:,2])])
    center12 = np.array([np.mean(p2[:,0]), np.mean(p2[:,1]), np.mean(p2[:,2])])
    center1 = (center11 + center12) / 2

    # Sphere case
    center2, radius2, inliers2 = geometry_list[1].fit(pcd_points, thresh=0.02, maxIteration=2000)
    p_outliers2 = extract_outliers(pcd_points, inliers2)

    # Cylinder case
    center3, axis3, radius3, height3, inliers3 = geometry_list[2].fit(pcd_points, thresh=0.02, maxIteration=4000)
    p_outliers3 = extract_outliers(pcd_points, inliers3)

    # Choose max inlier fitting
    inlier_num = [len(inliers11) + len(inliers12), len(inliers2), len(inliers3)]
    max_index = inlier_num.index(max(inlier_num))

    if max_index == 0:
        return [center1, eq11, eq12, p_outliers1]
    elif max_index == 1:
        return [center2, radius2, p_outliers2]
    elif max_index == 2:
        return [center3, axis3, radius3, height3, p_outliers3]



def geometry_matching(pcd_points, num):
    total = len(pcd_points)
    geometry_type_list = []
    geometry_inform_list = []

    result = feature_matching(pcd_points)
    outliers = result[len(result) - 1]
    outliers_ratio = len(outliers) / total

    append_geometry_list(result, geometry_type_list, geometry_inform_list)

    while not outliers_ratio < 0.1:
        result = feature_matching(outliers)
        outliers = result[len(result) - 1]
        outliers_ratio = len(outliers) / total

        append_geometry_list(result, geometry_type_list, geometry_inform_list)

    save_obj_geo_json(geometry_type_list, geometry_inform_list, num)


def get_Cuboid(temp):
    # Cuboid has center, 3 plane eq1, 3 plane eq2
    # Object Position
    center = temp[0]

    # Object Dimension
    eq1 = temp[1]
    eq2 = temp[2]

    a1 = (eq1[0][0] + eq2[0][0]) / 2
    b1 = (eq1[0][1] + eq2[0][1]) / 2
    c1 = (eq1[0][2] + eq2[0][2]) / 2
    normal1 = np.array([a1, b1, c1])
    normal1 = normal1 / np.linalg.norm(normal1)
    w = np.abs((eq1[0][3] - eq2[0][3]) / np.sqrt(a1 * a1 + b1 * b1 + c1 * c1))

    a2 = (eq1[1][0] + eq2[1][0]) / 2
    b2 = (eq1[1][1] + eq2[1][1]) / 2
    c2 = (eq1[1][2] + eq2[1][2]) / 2
    normal2 = np.array([a2, b2, c2])
    normal2 = normal2 / np.linalg.norm(normal2)
    h = np.abs((eq1[1][3] - eq2[1][3]) / np.sqrt(a2 * a2 + b2 * b2 + c2 * c2))

    a3 = (eq1[2][0] + eq2[2][0]) / 2
    b3 = (eq1[2][1] + eq2[2][1]) / 2
    c3 = (eq1[2][2] + eq2[2][2]) / 2
    normal3 = np.array([a3, b3, c3])
    normal3 = normal3 / np.linalg.norm(normal3)
    d = np.abs((eq1[2][3] - eq2[2][3]) / np.sqrt(a3 * a3 + b3 * b3 + c3 * c3))

    dims = np.round((w, h, d), 3)

    # Object Orientation
    axis_vec = normal1 + normal2 + normal3
    axis_vec = axis_vec / np.linalg.norm(axis_vec)
    vec = np.array([1, 1, 1])
    vec = vec / np.linalg.norm(vec)
    R_co = get_rotationMatrix_from_vectors(vec, axis_vec)

    # 6DoF
    T_co = np.identity(4)
    T_co[:3, :3] = np.round(R_co, 3)
    T_co[:3, 3] = np.round(center, 3)

    return T_co, dims, "Cuboid"



def get_Sphere(temp):
    # Sphere has center, radius
    # Object Position & Dimension
    center = temp[0]
    dims = (temp[1], temp[1], temp[1])

    # 6DoF (Do not need to consider orientation due to sphere geometry)
    T_co = np.identity(4)
    T_co[:3, 3] = np.round(center, 3)
    return T_co, dims, "Sphere"



def get_Cylinder(temp):
    # Cylinder has center, axis, radius, height
    # Object Position & Dimension
    center = temp[0]
    dims = (temp[2], temp[2], temp[3])

    # Object Orientation
    axis_vec = temp[1]
    vec = np.array([0, 0, 1])
    R_co = get_rotationMatrix_from_vectors(vec, axis_vec)

    # 6DoF
    T_co = np.identity(4)
    T_co[:3, :3] = np.round(R_co, 3)
    T_co[:3, 3] = np.round(center, 3)
    return T_co, dims, "Cylinder"




def check_geo_type(temp):
    if (len(temp) == 2):
        return "Sphere"
    elif (len(temp) == 3):
        return "Cuboid"
    elif (len(temp) == 4):
        return "Cylinder"



def convertGeometry(json_data, geo_num):
    geo_type = check_geo_type(json_data[str(geo_num)])
    if (geo_type == "Cuboid"):
        return get_Cuboid(json_data[str(geo_num)])
    elif (geo_type == "Sphere"):
        return get_Sphere(json_data[str(geo_num)])
    elif (geo_type == "Cylinder"):
        return get_Cylinder(json_data[str(geo_num)])



####################### SharedArray Server functions #######################
PCD_NUM_URI = "shm://pcd_num"
GPD_LIST_URI = "shm://gpd_list"
REQ_URI = "shm://request"
RESP_URI = "shm://response"

# pcd_num_p = None
# return_gpd_list_p = None
# request_p = None
# resp_p = None

def attacth_to_server():
    global pcd_num_p, return_gpd_list_p, request_p, resp_p
    try:
        pcd_num_p = sa.attach(PCD_NUM_URI)
        return_gpd_list_p = sa.attach(GPD_LIST_URI)
        request_p = sa.attach(REQ_URI)
        resp_p = sa.attach(RESP_URI)
    except Exception as e:
        print(e)


def detect_from_server(obj_num):
    if pcd_num_p is not None:
        pcd_num_p[:] = obj_num
        request_p[:] = 1
        while not resp_p[0]:
            time.sleep(0.01)
        resp_p[:] = 0
        return np.copy(return_gpd_list_p)
    else:
        print("Detect server not attached - call attach_to_server")