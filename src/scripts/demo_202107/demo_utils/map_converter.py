import os
import sys
sys.path.append(os.path.join(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src')))

import cv2
from pkg.utils.rotation_utils import *
from pkg.geometry.geotype import GEOTYPE

# NODE_NAME = "listen_tester"
# rospy.init_node(NODE_NAME, anonymous=True)


def convert_map(map_data):
    map_size = (map_data.info.height, map_data.info.width)
    map_im = (np.reshape(map_data.data, map_size) + 1).astype(np.uint8)
    return map_im, map_data.info.resolution


def convert_map_cm(map_data):
    map_im, resolution = convert_map(map_data)
    res_ratio = resolution / 0.01
    map_im_cm = cv2.resize(map_im, dsize=None, fx=res_ratio, fy=res_ratio)
    return map_im_cm


def line2im(line, shape, thickness=1):
    lineim = np.zeros(shape, dtype=np.uint8)
    cv2.line(lineim, (line[0], line[1]), (line[2], line[3]), 255, thickness)
    return lineim


def add_line_to_gscene(gscene, name, pt1, pt2, thickness, height, link_name="base_link"):
    center = tuple(np.mean([pt1, pt2], axis=0)) + (height / 2,)
    dp = np.subtract(pt2, pt1)
    dims = (np.linalg.norm(dp), thickness, height)
    rpy = Rot2rpy(Rot_axis(3, np.arctan2(dp[1], dp[0])))
    gscene.create_safe(GEOTYPE.BOX, name, link_name=link_name,
                       dims=dims, center=center, rpy=rpy,
                       color=(1, 0, 0, 0.3), display=True,
                       collision=True, fixed=True)