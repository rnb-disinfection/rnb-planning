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


# geometry_list = [Cuboid(), Sphere(), Cylinder()]

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



####################### Util functions for streaming #######################
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
        #         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        #         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #     else:
        #         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        #         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start streaming
        profile = pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        # Set Preset option
        depth_sensor.set_option(rs.option.visual_preset, 3)
        depth_sensor.set_option(rs.option.laser_power, 89)
        depth_sensor.set_option(rs.option.noise_filtering, 4)
        depth_sensor.set_option(rs.option.receiver_gain, 17)
        depth_sensor.set_option(rs.option.post_processing_sharpening, 2.0)
        depth_sensor.set_option(rs.option.pre_processing_sharpening, 0.7)

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



####################### Util functions for get point cloud #######################
def getPCDAll(img_num, depth_seg_path, pcd_path, Camera_parameter):
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
    for i in range(len(os.listdir(depth_seg_path))):

        # Consider only segmentation result number is over 70% of total image number
        if (len(os.listdir(depth_seg_path + '/{}'.format(i + 1))) > int(img_num * 0.7)):
            pcd_overlap = o3d.geometry.PointCloud()
            for j in range(img_num // 4):
                k = j * 4
                if (os.path.exists(depth_seg_path + '/{}'.format(i + 1) + '/depth_mask_{}.png'.format(k))):
                    img_s = o3d.io.read_image(depth_seg_path + '/{}'.format(i + 1) + '/depth_mask_{}.png'.format(k))
                    pcd_s = o3d.geometry.PointCloud.create_from_depth_image(img_s,
                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                        cam_height,cam_fx, cam_fy,
                                                                        cam_ppx, cam_ppy), np.linalg.inv(cam_traj[k]),
                                                                        depth_scale= d_scale)

                    d_trunc = np.linalg.norm(pcd_s.get_center()) * 1.1
                    pcd_s = o3d.geometry.PointCloud.create_from_depth_image(img_s,
                                                o3d.camera.PinholeCameraIntrinsic(cam_width,
                                                                        cam_height, cam_fx, cam_fy,
                                                                        cam_ppx, cam_ppy), np.linalg.inv(cam_traj[k]),
                                                                        depth_scale = d_scale, depth_trunc = d_trunc)
                    source = pcd_s
                    dists = source.compute_point_cloud_distance(pcd)

                    dists = np.array(dists)
                    thres = 0.003
                    indices = np.where(dists < thres)[0]
                    # pcd_overlap = pcd.select_by_index(indices)

                    p_overlap = []
                    for l in range(len(indices)):
                        p_overlap.append(source.points[indices[l]])

                    pcd_temp = o3d.geometry.PointCloud()
                    pcd_temp.points = o3d.utility.Vector3dVector(p_overlap)
                    # size = int(len(np.asarray(pcd_temp.points)) // 1000)
                    pcd_temp.uniform_down_sample(every_k_points=13)
                    pcd_overlap = pcd_overlap + pcd_temp
                    # size = int(len(np.asarray(pcd_overlap.points)) // 1000)
                    pcd_overlap.uniform_down_sample(every_k_points=19)
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



####################### Util functions for add geometry #######################
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
    tmp = []
    for i in range(len(pcd_points)):
        idx = i
        if idx not in inliers:
            tmp.append(pcd_points[idx])

    p_outliers = np.zeros((len(tmp), 3))
    for i in range(len(tmp)):
        p_outliers[i] = tmp[i]

    return p_outliers



def get_inliers(pcd_points, inliers):
    p_inliers = []
    for i in range(len(pcd_points)):
        idx = i
        if idx in inliers:
            p_inliers.append(pcd_points[idx])

    return p_inliers



def save_obj_geo_json(geometry_type_list, geometry_inform_list, num):
    json_data = {}
    for i in range(len(geometry_type_list)):
        json_data[i + 1] = geometry_inform_list[i]
        print(geometry_type_list[i])
        print(geometry_inform_list[i])

    with open(WORKING_DIR + '/obj_geometry_{}.json'.format(num), 'w') as outfile:
        obj = json.dump(json_data, outfile, cls=NumpyEncoder)



def post_processing_cuboid(pcd_points, p_inliers, eq1, eq2):
    x_sum = 0.0
    y_sum = 0.0
    z_sum = 0.0
    for i in range(len(p_inliers)):
        x_sum += p_inliers[i][0]
        y_sum += p_inliers[i][1]
        z_sum += p_inliers[i][2]

    center = np.array([x_sum / len(p_inliers), y_sum / len(p_inliers), z_sum / len(p_inliers)])

    # normal1
    dot1 = abs(np.dot(eq1[0][:3], eq2[0][:3]))
    dot2 = abs(np.dot(eq1[0][:3], eq2[1][:3]))
    dot3 = abs(np.dot(eq1[0][:3], eq2[2][:3]))
    dot_list = [dot1, dot2, dot3]
    idx = dot_list.index(max(dot_list))
    normal1 = np.array([0, 0, 0])
    length1 = 0
    if idx == 0:
        normal1, length1 = calculate_normal_dims(eq1[0], eq2[0])
    elif idx == 1:
        normal1, length1 = calculate_normal_dims(eq1[0], eq2[1])
    elif idx == 2:
        normal1, length1 = calculate_normal_dims(eq1[0], eq2[2])

    # normal2
    dot1 = abs(np.dot(eq1[1][:3], eq2[0][:3]))
    dot2 = abs(np.dot(eq1[1][:3], eq2[1][:3]))
    dot3 = abs(np.dot(eq1[1][:3], eq2[2][:3]))
    dot_list = [dot1, dot2, dot3]
    idx = dot_list.index(max(dot_list))
    normal2 = np.array([0, 0, 0])
    length2 = 0
    if idx == 0:
        normal2, length2 = calculate_normal_dims(eq1[1], eq2[0])
    elif idx == 1:
        normal2, length2 = calculate_normal_dims(eq1[1], eq2[1])
    elif idx == 2:
        normal2, length2 = calculate_normal_dims(eq1[1], eq2[2])

    # normal3
    dot1 = abs(np.dot(eq1[2][:3], eq2[0][:3]))
    dot2 = abs(np.dot(eq1[2][:3], eq2[1][:3]))
    dot3 = abs(np.dot(eq1[2][:3], eq2[2][:3]))
    dot_list = [dot1, dot2, dot3]
    idx = dot_list.index(max(dot_list))
    normal3 = np.array([0, 0, 0])
    length3 = 0
    if idx == 0:
        normal3, length3 = calculate_normal_dims(eq1[2], eq2[0])
    elif idx == 1:
        normal3, length3 = calculate_normal_dims(eq1[2], eq2[1])
    elif idx == 2:
        normal3, length3 = calculate_normal_dims(eq1[2], eq2[2])

    a = length1 / 2.0
    b = length2 / 2.0
    c = length3 / 2.0

    idx_inliers = []
    thres = np.sqrt(a*a + b*b + c*c) * 1.2
    for i in range(len(pcd_points)):
        if np.linalg.norm(pcd_points[i,:] - center) < thres:
            idx_inliers.append(i)

    p_inliers_result = get_inliers(pcd_points, idx_inliers)
    p_outliers_result = extract_outliers(pcd_points, idx_inliers)

    x_sum = 0.0
    y_sum = 0.0
    z_sum = 0.0
    center_result = np.zeros((1,3))
    for i in range(len(p_inliers_result)):
        x_sum += p_inliers_result[i][0]
        y_sum += p_inliers_result[i][1]
        z_sum += p_inliers_result[i][2]

    if len(p_inliers_result) != 0.0:
        center_result = np.array([x_sum / len(p_inliers_result), y_sum / len(p_inliers_result), z_sum / len(p_inliers_result)])

    return center_result, p_inliers_result, p_outliers_result




def cuboid_fitting(pcd_points, thres, thres_ratioB=0.2, maxIterB=3500):
    cuboid_obj = Cuboid()

    # Cuboid case
    # Pyransac3D cuboid fitting only return 3 plane eq, So apply fitting twice
    eq11, inliers11 = cuboid_obj.fit(pcd_points, thresh=thres * thres_ratioB, maxIteration=maxIterB)
    temp_outlier = extract_outliers(pcd_points, inliers11)
    eq12, inliers12 = cuboid_obj.fit(temp_outlier, thresh=thres * thres_ratioB, maxIteration=maxIterB)
    p_inliers11 = get_inliers(pcd_points, inliers11)
    p_inliers12 = get_inliers(temp_outlier, inliers12)
    p_inliers1 = p_inliers11 + p_inliers12
    p_outliers1 = extract_outliers(temp_outlier, inliers12)


    # Post-processing center, inliers
    center1, p_inliers1, p_outliers1 = post_processing_cuboid(pcd_points, p_inliers1, eq11, eq12)

    return [center1, eq11, eq12, p_outliers1], p_inliers1



def sphere_fitting(pcd_points, thres, thres_ratioS=0.15, maxIterS=3000):
    sphere_obj = Sphere()

    # Sphere case
    center2, radius2, inliers2 = sphere_obj.fit(pcd_points, thresh=thres * thres_ratioS, maxIteration=maxIterS)
    p_inliers2 = get_inliers(pcd_points, inliers2)
    p_outliers2 = extract_outliers(pcd_points, inliers2)

    return [center2, radius2, p_outliers2], p_inliers2



def cylinder_fitting(pcd_points, thres, thres_ratioC=0.25, maxIterC=8000):
    cylinder_obj = Cylinder()

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_points)
    pcd.estimate_normals()
    pcd.normalize_normals()
    point_normals = np.asarray(pcd.normals)

    # Cylinder case
    center3, axis3, radius3, height3, inliers3 = cylinder_obj.fit(pcd_points, point_normals, thresh=thres * thres_ratioC,
                                                                                            maxIteration=maxIterC)
    p_inliers3 = get_inliers(pcd_points, inliers3)
    p_outliers3 = extract_outliers(pcd_points, inliers3)

    return [center3, axis3, radius3, height3, p_outliers3], p_inliers3



def feature_matching(pcd_points, voxel_size=0.05, thres_ratioB=0.2, thres_ratioS=0.15, thres_ratioC=0.25,
                                            maxIterB=3500, maxIterS=3000, maxIterC=8000):
    total = len(pcd_points)
    thres = voxel_size

    # First, find primitive shape(harsh threshold)
    resultB, inliersB = cuboid_fitting(pcd_points, thres, thres_ratioB=thres_ratioB, maxIterB=maxIterB)
    resultS, inliersS = sphere_fitting(pcd_points, thres, thres_ratioS=thres_ratioS, maxIterS=maxIterS)
    resultC, inliersC = cylinder_fitting(pcd_points, thres, thres_ratioC=thres_ratioC, maxIterC=maxIterC)

    print("Total number :", total)
    print("Cuboid inlier : ", int((len(inliersB)/2)))
    print("Sphere inlier : ", len(inliersS))
    print("Cylinder inlier : ", len(inliersC))

    # Determien privitivs shape. Choose max inlier fitting
    inlier_num = [int((len(inliersB)/2)), len(inliersS), len(inliersC)]
    max_index = inlier_num.index(max(inlier_num))

    # Get inliers(relaxed threshold)
    # This case is cuboid fitting
    if max_index == 0:
        return cuboid_fitting(pcd_points, thres, thres_ratioB=thres_ratioB * 2.4, maxIterB=maxIterB)
    # This case is sphere fitting
    elif max_index == 1:
        return sphere_fitting(pcd_points, thres, thres_ratioS=thres_ratioS * 1.9, maxIterS=maxIterS)
    # This case is cylinder fitting
    elif max_index == 2:
        return cylinder_fitting(pcd_points, thres, thres_ratioC=thres_ratioC * 4.1, maxIterC=maxIterC)



def geometry_matching(pcd_points, num, voxel_size=0.05, outliers_ratio_max=0.2, thres_ratioB=0.2, thres_ratioS=0.15, thres_ratioC=0.25,
                                                        maxIterB=3500, maxIterS=3000, maxIterC=8000):
    total = len(pcd_points)
    geometry_type_list = []
    geometry_inform_list = []
    save_inlier_points = []

    result, inliers = feature_matching(pcd_points, voxel_size=voxel_size, thres_ratioB=thres_ratioB,
                                       thres_ratioS=thres_ratioS, thres_ratioC=thres_ratioC,
                                       maxIterB=maxIterB, maxIterS=maxIterS, maxIterC=maxIterC)
    outliers = result[len(result) - 1]
    outliers_ratio = float(len(outliers)) / float(total)
    save_inlier_points = save_inlier_points + inliers

    pcd_ = o3d.geometry.PointCloud()
    result_ = o3d.geometry.PointCloud()
    pcd_.points = o3d.utility.Vector3dVector(pcd_points)
    result_.points = o3d.utility.Vector3dVector(inliers)
    pcd_.paint_uniform_color((0,0,0))
    result_.paint_uniform_color((1,0,0))
    o3d.visualization.draw_geometries([pcd_, result_])

    append_geometry_list(result, geometry_type_list, geometry_inform_list)
    print("Outlier ratio : ", outliers_ratio)

    while not outliers_ratio < outliers_ratio_max:
        result, inliers = feature_matching(outliers, voxel_size=voxel_size, thres_ratioB=thres_ratioB,
                                           thres_ratioS=thres_ratioS, thres_ratioC=thres_ratioC,
                                           maxIterB=maxIterB, maxIterS=maxIterS, maxIterC=maxIterC)
        outliers = result[len(result) - 1]
        outliers_ratio = float(len(outliers)) / float(total)
        save_inlier_points = save_inlier_points + inliers

        pcd_ = o3d.geometry.PointCloud()
        result_ = o3d.geometry.PointCloud()
        pcd_.points = o3d.utility.Vector3dVector(outliers)
        result_.points = o3d.utility.Vector3dVector(inliers)
        pcd_.paint_uniform_color((0, 0, 0))
        result_.paint_uniform_color((1, 0, 0))
        o3d.visualization.draw_geometries([pcd_, result_])

        append_geometry_list(result, geometry_type_list, geometry_inform_list)
        print("Outlier ratio : ", outliers_ratio)

    save_obj_geo_json(geometry_type_list, geometry_inform_list, num)

    return save_inlier_points



def calculate_normal_dims(eq1, eq2):
    if (np.dot(eq1[:3], eq2[:3]) < 0):
        a = (eq1[0] - eq2[0]) / 2
        b = (eq1[1] - eq2[1]) / 2
        c = (eq1[2] - eq2[2]) / 2
        normal = np.array([a, b, c])
        length = abs((eq1[3] + eq2[3]) / np.linalg.norm(normal))
        normal = normal / np.linalg.norm(normal)
    else:
        a = (eq1[0] + eq2[0]) / 2
        b = (eq1[1] + eq2[1]) / 2
        c = (eq1[2] + eq2[2]) / 2
        normal = np.array([a, b, c])
        length = abs((eq1[3] - eq2[3]) / np.linalg.norm(normal))
        normal = normal / np.linalg.norm(normal)

    return normal, length



def get_Cuboid(temp):
    # Cuboid has center, 3 plane eq1, 3 plane eq2
    # Object Position
    center = temp[0]

    # Object Dimension
    eq1 = temp[1]
    eq2 = temp[2]

    # normal1
    dot1 = abs(np.dot(eq1[0][:3], eq2[0][:3]))
    dot2 = abs(np.dot(eq1[0][:3], eq2[1][:3]))
    dot3 = abs(np.dot(eq1[0][:3], eq2[2][:3]))
    dot_list = [dot1, dot2, dot3]
    idx = dot_list.index(max(dot_list))
    normal1 = np.array([0,0,0])
    length1 = 0
    if idx == 0:
        normal1, length1 = calculate_normal_dims(eq1[0], eq2[0])
    elif idx == 1:
        normal1, length1 = calculate_normal_dims(eq1[0], eq2[1])
    elif idx == 2:
        normal1, length1 = calculate_normal_dims(eq1[0], eq2[2])

    # normal2
    dot1 = abs(np.dot(eq1[1][:3], eq2[0][:3]))
    dot2 = abs(np.dot(eq1[1][:3], eq2[1][:3]))
    dot3 = abs(np.dot(eq1[1][:3], eq2[2][:3]))
    dot_list = [dot1, dot2, dot3]
    idx = dot_list.index(max(dot_list))
    normal2 = np.array([0, 0, 0])
    length2 = 0
    if idx == 0:
        normal2, length2 = calculate_normal_dims(eq1[1], eq2[0])
    elif idx == 1:
        normal2, length2 = calculate_normal_dims(eq1[1], eq2[1])
    elif idx == 2:
        normal2, length2 = calculate_normal_dims(eq1[1], eq2[2])

    # normal3
    dot1 = abs(np.dot(eq1[2][:3], eq2[0][:3]))
    dot2 = abs(np.dot(eq1[2][:3], eq2[1][:3]))
    dot3 = abs(np.dot(eq1[2][:3], eq2[2][:3]))
    dot_list = [dot1, dot2, dot3]
    idx = dot_list.index(max(dot_list))
    normal3 = np.array([0, 0, 0])
    length3 = 0
    if idx == 0:
        normal3, length3 = calculate_normal_dims(eq1[2], eq2[0])
    elif idx == 1:
        normal3, length3 = calculate_normal_dims(eq1[2], eq2[1])
    elif idx == 2:
        normal3, length3 = calculate_normal_dims(eq1[2], eq2[2])

    dims = np.round((length1, length2, length3), 4)
    normals = [normal1, normal2, normal3]
    R_co = np.identity(3)
    R_co[:3, 0] = normal1
    R_co[:3, 1] = normal2
    R_co[:3, 2] = normal3
    # print("R_co")
    # print(R_co)
    # print(np.linalg.det(R_co))

    # 6DoF
    T_co = np.identity(4)
    T_co[:3, :3] = np.round(R_co, 4)
    T_co[:3, 3] = np.round(center, 4)

    return T_co, dims, "Cuboid"



def get_Sphere(temp):
    # Sphere has center, radius
    # Object Position & Dimension
    center = temp[0]
    dims = (temp[1], temp[1], temp[1])

    # 6DoF (Do not need to consider orientation due to sphere geometry)
    T_co = np.identity(4)
    T_co[:3, 3] = np.round(center, 4)
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
    T_co[:3, :3] = np.round(R_co, 4)
    T_co[:3, 3] = np.round(center, 4)
    return T_co, dims, "Cylinder"



def check_geo_type(temp):
    if (len(temp) == 2):
        return "Sphere"
    elif (len(temp) == 3):
        return "Cuboid"
    elif (len(temp) == 4):
        return "Cylinder"



def add_geometry_body(gscene, geotype, T_bo, dims_, num):
    if (geotype == "Cuboid"):
        body = gscene.create_safe(gtype=GEOTYPE.BOX, name="obj_{}".format(num), link_name="base_link",
                                  dims=dims_, center=T_bo[:3, 3], rpy=Rot2rpy(T_bo[:3, :3]),
                                  color=(0.1,0.9,0.1,0.9), display=True, collision=True, fixed=False)

    elif (geotype == "Sphere"):
        body = gscene.create_safe(gtype=GEOTYPE.SPHERE, name="obj_{}".format(num), link_name="base_link",
                                  dims=dims_, center=T_bo[:3, 3], rpy=Rot2rpy(T_bo[:3, :3]),
                                  color=(0.1,0.1,0.9,0.9), display=True, collision=True, fixed=False)

    elif (geotype == "Cylinder"):
        body = gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="obj_{}".format(num), link_name="base_link",
                                  dims=dims_, center=T_bo[:3, 3], rpy=Rot2rpy(T_bo[:3, :3]),
                                  color=(0.9,0.1,0.1,0.9), display=True, collision=True, fixed=False)
    return body



def add_geometry_sub(gscene, geotype_sub, T_oo_sub, dims_sub, num, sub_num):
    if (geotype_sub == "Cuboid"):
        gscene.create_safe(gtype=GEOTYPE.BOX, name="obj_{}".format(num) + "_sub_{}".format(sub_num),
            link_name="base_link", dims=dims_sub, center=T_oo_sub[:3,3], rpy=Rot2rpy(T_oo_sub[:3,:3]),
            color=(0.1,0.9,0.1,0.9), display=True, collision=True, fixed=False,  parent="obj_{}".format(num))

    elif (geotype_sub == "Sphere"):
        gscene.create_safe(gtype=GEOTYPE.SPHERE, name="obj_{}".format(num) + "_sub_{}".format(sub_num),
            link_name="base_link", dims=dims_sub, center=T_oo_sub[:3,3], rpy=Rot2rpy(T_oo_sub[:3,:3]),
            color=(0.1,0.1,0.9,0.9), display=True, collision=True, fixed=False,  parent="obj_{}".format(num))

    elif (geotype_sub == "Cylinder"):
        gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="obj_{}".format(num) + "_sub_{}".format(sub_num),
            link_name="base_link", dims=dims_sub, center=T_oo_sub[:3,3], rpy=Rot2rpy(T_oo_sub[:3,:3]),
            color=(0.9,0.1,0.1,0.9), display=True, collision=True, fixed=False,  parent="obj_{}".format(num))



def add_geometry(gscene, json_data, T_bc, component_num, num, grasp_list):
    if component_num == 1:
        T_co, dims, geotype = convertGeometry(json_data, component_num)
        for i in range(len(grasp_list[num])):
            grasp_list[num - 1] = np.matmul(np.linalg.inv(T_co), grasp_list[num - 1])
        T_bo = np.matmul(T_bc, T_co)
        body = add_geometry_body(gscene, geotype, T_bo, dims, num)
    else:
        T_co, dims, geotype = convertGeometry(json_data, 1)
        for i in range(len(grasp_list[num])):
            grasp_list[num - 1] = np.matmul(np.linalg.inv(T_co), grasp_list[num - 1])
        T_bo = np.matmul(T_bc, T_co)
        body = add_geometry_body(gscene, geotype, T_bo, dims, num)

        for i in range(component_num - 1):
            T_co_sub, dims_sub, geotype_sub = convertGeometry(json_data, i + 2)
            T_bo_sub = np.matmul(T_bc, T_co_sub)
            T_oo_sub = np.matmul(np.linalg.inv(T_bo), T_bo_sub)
            add_geometry_sub(gscene, geotype_sub, T_oo_sub, dims_sub, num, i + 1)

    return body



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
