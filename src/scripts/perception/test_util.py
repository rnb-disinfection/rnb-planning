import pyrealsense2 as rs
import numpy as np
import cv2
import os
import sys
import open3d as o3d
import shutil

sys.path.append(os.path.join(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src')))
from pkg.global_config import RNB_PLANNING_DIR
from pkg.utils.utils import *
from pkg.utils.rotation_utils import *

def make_dataset_folder(path_folder):
    if not os.path.exists(path_folder):
        os.makedirs(path_folder)
        os.makedirs(path_folder + '/depth')
        os.makedirs(path_folder + '/color')
        os.makedirs(path_folder + '/depth_segmented')
        os.makedirs(path_folder + '/color_segmented')

    else:
        user_input = raw_input("%s not empty. Overwrite? (y/n) : " % path_folder)
        if user_input.lower() == 'y':
            shutil.rmtree(path_folder)
            os.makedirs(path_folder)
            os.makedirs(path_folder + '/depth')
            os.makedirs(path_folder + '/color')
            os.makedirs(path_folder + '/depth_segmented')
            os.makedirs(path_folder + '/color_segmented')
            return True
        else:
            return False


def cam_streaming(path_folder):
    # Make save directory
    if (make_dataset_folder(path_folder)):
        # Create a pipeline
        pipeline = rs.pipeline()
        # Create a config and configure the pipeline to stream
        config = rs.config()

        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)

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

        # #  Not display the baground more than clipping_distance
        # clipping_distance_in_meters = 3.0  # 2 meter
        # clipping_distance = clipping_distance_in_meters / depth_scale

        # Align depth to color
        align_to = rs.stream.color
        align = rs.align(align_to)

        # Streaming loop
        frame_count = 0
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

                # # Remove background - Set pixels further than clipping_distance to grey
                # grey_color = 153
                # # depth image is 1 channel, color is 3 channels
                # depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
                # bg_removed = np.where((depth_image_3d > clipping_distance) | \
                #                       (depth_image_3d <= 0), grey_color, color_image)
                #
                # # Render images
                # depth_colormap = cv2.applyColorMap(
                #     cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
                # images = np.hstack((bg_removed, depth_colormap))

                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = depth_colormap.shape
                color_colormap_dim = color_image.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                if depth_colormap_dim != color_colormap_dim:
                    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                    images = np.hstack((resized_color_image, depth_colormap))
                else:
                    images = np.hstack((color_image, depth_colormap))
                cv2.namedWindow('Recorder Realsense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Recorder Realsense', images)
                key = cv2.waitKey(1)

                if True:
                    # Save color, depth image
                    cv2.imwrite(path_folder + "/depth/depth_{}.png".format(frame_count), depth_image)
                    cv2.imwrite(path_folder + "/color/color_{}.jpg".format(frame_count), color_image)
                    print("Saved color + depth image %d" % frame_count)
                    frame_count += 1

                # if 'esc' button pressed, escape loop and exit streaming
                if key == 27:
                    cv2.destroyAllWindows()
                    break
        finally:
            print("Stop the streaming and Save RGBD images\n")
            pipeline.stop()

    else:
        pass


from math import pi, cos, sin


class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, v):
        if not isinstance(v, Vector):
            return NotImplemented
        return Vector(self.x + v.x, self.y + v.y)

    def __sub__(self, v):
        if not isinstance(v, Vector):
            return NotImplemented
        return Vector(self.x - v.x, self.y - v.y)

    def cross(self, v):
        if not isinstance(v, Vector):
            return NotImplemented
        return self.x*v.y - self.y*v.x


class Line:
    # ax + by + c = 0
    def __init__(self, v1, v2):
        self.a = v2.y - v1.y
        self.b = v1.x - v2.x
        self.c = v2.cross(v1)

    def __call__(self, p):
        return self.a*p.x + self.b*p.y + self.c

    def intersection(self, other):
        # See e.g.     https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Using_homogeneous_coordinates
        if not isinstance(other, Line):
            return NotImplemented
        w = self.a*other.b - self.b*other.a
        return Vector(
            (self.b*other.c - self.c*other.b)/w,
            (self.c*other.a - self.a*other.c)/w
        )


def rectangle_vertices(cx, cy, w, h, r):
    angle = pi*r/180
    dx = w/2
    dy = h/2
    dxcos = dx*cos(angle)
    dxsin = dx*sin(angle)
    dycos = dy*cos(angle)
    dysin = dy*sin(angle)
    return (
        Vector(cx, cy) + Vector(-dxcos - -dysin, -dxsin + -dycos),
        Vector(cx, cy) + Vector( dxcos - -dysin,  dxsin + -dycos),
        Vector(cx, cy) + Vector( dxcos -  dysin,  dxsin +  dycos),
        Vector(cx, cy) + Vector(-dxcos -  dysin, -dxsin +  dycos)
    )

def intersection_area(r1, r2):
    # r1 and r2 are in (center, width, height, rotation) representation
    # First convert these into a sequence of vertices

    rect1 = rectangle_vertices(*r1)
    rect2 = rectangle_vertices(*r2)

    # Use the vertices of the first rectangle as
    # starting vertices of the intersection polygon.
    intersection = rect1

    # Loop over the edges of the second rectangle
    for p, q in zip(rect2, rect2[1:] + rect2[:1]):
        if len(intersection) <= 2:
            break # No intersection

        line = Line(p, q)

        # Any point p with line(p) <= 0 is on the "inside" (or on the boundary),
        # any point p with line(p) > 0 is on the "outside".

        # Loop over the edges of the intersection polygon,
        # and determine which part is inside and which is outside.
        new_intersection = []
        line_values = [line(t) for t in intersection]
        for s, t, s_value, t_value in zip(
            intersection, intersection[1:] + intersection[:1],
            line_values, line_values[1:] + line_values[:1]):
            if s_value <= 0:
                new_intersection.append(s)
            if s_value * t_value < 0:
                # Points are on opposite sides.
                # Add the intersection of the lines to new_intersection.
                intersection_point = line.intersection(Line(s, t))
                new_intersection.append(intersection_point)

        intersection = new_intersection

    # Calculate area
    if len(intersection) <= 2:
        return 0

    return 0.5 * sum(p.x*q.y - p.y*q.x for p, q in
                     zip(intersection, intersection[1:] + intersection[:1]))


def get_iou_3d(aabox, aabox_gt, points, points_gt, T_off):
    points_center = aabox.get_center()
    points_gt_center = aabox_gt.get_center()

    points_dim = (np.max(points[:, 0]) - np.min(points[:, 0]),
                  np.max(points[:, 1]) - np.min(points[:, 1]),
                  np.max(points[:, 2]) - np.min(points[:, 2]))

    points_gt_dim = (np.max(points_gt[:, 0]) - np.min(points_gt[:, 0]),
                     np.max(points_gt[:, 1]) - np.min(points_gt[:, 1]),
                     np.max(points_gt[:, 2]) - np.min(points_gt[:, 2]))

    z_max = points_center[2] + points_dim[2] / 2
    z_min = points_center[2] - points_dim[2] / 2

    z_max_ = points_gt_center[2] + points_gt_dim[2] / 2
    z_min_ = points_gt_center[2] - points_gt_dim[2] / 2

    h_overlap = min(z_max, z_max_) - max(z_min, z_min_)

    angle = np.rad2deg(Rot2axis(T_off[:3,:3],3))
    polygon1 = (points_center[0], points_center[1], points_dim[0], points_dim[1], angle)
    polygon2 = (points_gt_center[0], points_gt_center[1], points_gt_dim[0], points_gt_dim[1], 0)
    intersection = intersection_area(polygon1, polygon2)

    iou_3d = intersection * h_overlap / (aabox.volume() + aabox_gt.volume() - intersection * h_overlap)
    print("IOU_3d:", iou_3d)
    return iou_3d



def preprocess(aabox, aabox_gt, points, points_gt):
    points_center = aabox.get_center()
    points_gt_center = aabox_gt.get_center()

    points_dim = (np.max(points[:, 0]) - np.min(points[:, 0]),
                  np.max(points[:, 1]) - np.min(points[:, 1]),
                  np.max(points[:, 2]) - np.min(points[:, 2]))

    points_gt_dim = (np.max(points_gt[:, 0]) - np.min(points_gt[:, 0]),
                     np.max(points_gt[:, 1]) - np.min(points_gt[:, 1]),
                     np.max(points_gt[:, 2]) - np.min(points_gt[:, 2]))
    x_max = points_center[0] + points_dim[0] / 2
    x_min = points_center[0] - points_dim[0] / 2
    y_max = points_center[1] + points_dim[1] / 2
    y_min = points_center[1] - points_dim[1] / 2
    z_max = points_center[2] + points_dim[2] / 2
    z_min = points_center[2] - points_dim[2] / 2
    points_2d = (x_min, y_min, x_max, y_max)

    x_max_ = points_gt_center[0] + points_gt_dim[0] / 2
    x_min_ = points_gt_center[0] - points_gt_dim[0] / 2
    y_max_ = points_gt_center[1] + points_gt_dim[1] / 2
    y_min_ = points_gt_center[1] - points_gt_dim[1] / 2
    z_max_ = points_gt_center[2] + points_gt_dim[2] / 2
    z_min_ = points_gt_center[2] - points_gt_dim[2] / 2
    points_gt_2d = (x_min_, y_min_, x_max_, y_max_)

    h_overlap = min(z_max, z_max_) - max(z_min, z_min_)

    return points_2d, points_gt_2d, h_overlap

def box2d_iou(box1, box2):
    ''' Compute 2D bounding box IoU.
    Input:
        box1: tuple of (xmin,ymin,xmax,ymax)
        box2: tuple of (xmin,ymin,xmax,ymax)
    Output:
        iou: 2D IoU scalar
    '''
    return get_iou({'x1': box1[0], 'y1': box1[1], 'x2': box1[2], 'y2': box1[3]}, \
                   {'x1': box2[0], 'y1': box2[1], 'x2': box2[2], 'y2': box2[3]})

def get_iou(bb1, bb2):
    # determine the coordinates of the intersection rectangle
    x_left = max(bb1['x1'], bb2['x1'])
    y_top = max(bb1['y1'], bb2['y1'])
    x_right = min(bb1['x2'], bb2['x2'])
    y_bottom = min(bb1['y2'], bb2['y2'])

    if x_right < x_left or y_bottom < y_top:
        return 0.0

    # The intersection of two axis-aligned bounding boxes is always an
    # axis-aligned bounding box
    intersection_area = (x_right - x_left) * (y_bottom - y_top)

    # compute the area of both AABBs
    bb1_area = (bb1['x2'] - bb1['x1']) * (bb1['y2'] - bb1['y1'])
    bb2_area = (bb2['x2'] - bb2['x1']) * (bb2['y2'] - bb2['y1'])

    # compute the intersection over union by taking the intersection
    # area and dividing it by the sum of prediction + ground-truth
    # areas - the interesection area
    iou = intersection_area / float(bb1_area + bb2_area - intersection_area)
    assert iou >= 0.0
    assert iou <= 1.0
    return iou, intersection_area

def get_3d_iou(aabox, aabox_gt, points, points_gt):
    points_2d, points_gt_2d, h_overlap = preprocess(aabox, aabox_gt, points, points_gt)
    iou_2d, overlap_2d = box2d_iou(points_2d, points_gt_2d)
    iou_3d = overlap_2d * h_overlap / (aabox.volume() + aabox_gt.volume() - overlap_2d * h_overlap)
    print("IOU_3d:", iou_3d)
    return np.round(iou_3d, 5)