from __future__ import print_function
import os
import sys

import rospy

sys.path.append(os.path.join(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src')))
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src/scripts/demo_202107'))
from pkg.utils.ros_utils import *
from demo_utils.map_converter import *
from demo_utils.kiro_udp_send import start_mobile_udp_thread, get_xyzw_cur
import time
import cv2
from enum import Enum
from pkg.global_config import RNB_PLANNING_DIR
from pkg.utils.ros_utils import *
from pkg.utils.rotation_utils import *
from pkg.utils.utils import *
from pkg.utils.shared_function import shared_fun, CallType, ArgProb, ResProb, \
    set_serving, is_serving, serve_forever
import cv2
import subprocess

TIMEOUT_GET_MAP = 3
class GetMapResult(Enum):
    SUCCESS = 0
    TIMEOUT = 1

DEMO_UTIL_DIR = os.path.join(RNB_PLANNING_DIR, "src/scripts/demo_202107/demo_utils")

GMAP_SIZE = (384, 384)

class KiroMobileMap:
    def __init__(self, master_ip, cur_ip, connection_state=True):
        self.master_ip, self.cur_ip = master_ip, cur_ip
        self.connection_state = connection_state
        self.gtem_list = []
        if is_serving():
            if connection_state:
                rospy.init_node("map_receiver")
                # GET MAP
                self.map_listener = Listener(topic_name="/map", topic_type=OccupancyGrid)
                self.cost_listener = Listener(topic_name="/move_base/global_costmap/costmap",
                                              topic_type=OccupancyGrid)
                self.lcost_listener = Listener(topic_name="/move_base/local_costmap/costmap",
                                               topic_type=OccupancyGrid)
        else:
            output = subprocess.Popen(['sh',
                                       os.path.join(DEMO_UTIL_DIR, 'get_ros_map.sh'),
                                       self.master_ip, self.cur_ip,
                                       str(connection_state).lower()],
                                     cwd=DEMO_UTIL_DIR)
        self.get_maps = \
            shared_fun(CallType.SYNC, "KiroMobileMap",
                       ResProb(0, (1000000,), dict),
                       ResProb(1, (1000000,), dict),
                       ResProb(2, (1000000,), dict))\
                (self.get_maps)

    def get_maps(self):
        if self.connection_state:
            lcost_dict = extract_attr_dict(self.lcost_listener.get_data())
            cost_dict = extract_attr_dict(self.cost_listener.last_dat)
            map_dict = extract_attr_dict(self.map_listener.last_dat)
        else:
            map_data = load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/map_data.pkl"))
            cost_data = load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/cost_data.pkl"))
            lcost_data = load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/lcost_data.pkl"))
            lcost_dict = extract_attr_dict(lcost_data)
            cost_dict = extract_attr_dict(cost_data)
            map_dict = extract_attr_dict(map_data)
        return lcost_dict, cost_dict, map_dict

    def set_maps(self, lcost_dict, cost_dict, map_dict, T_bm, canny_ksize=10):
        self.lcost_im, self.lresolution = convert_map(lcost_dict)
        self.map_im, self.resolution = convert_map(map_dict)
        self.cost_im, self.resolution = convert_map(cost_dict)
        
        ret, self.cost_bin = cv2.threshold(self.cost_im, 100, 255, cv2.THRESH_BINARY)
        self.cost_closed = cv2.morphologyEx(self.cost_bin, cv2.MORPH_CLOSE,
                                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                                      (canny_ksize,canny_ksize)))
        self.cost_canny = cv2.Canny(self.cost_closed, 50, 150)

        ret, self.lcost_bin = cv2.threshold(self.lcost_im, 100, 255, cv2.THRESH_BINARY)
        self.lcost_closed = cv2.morphologyEx(self.lcost_bin, cv2.MORPH_CLOSE,
                                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                                      (canny_ksize,canny_ksize)))
        self.lcost_canny = cv2.Canny(self.lcost_closed, 50, 150)
        self.T_bm = T_bm

    @classmethod
    def convert_im2scene(cls, img_bin, resolution, origin_on_base, T_bm, height):
        if origin_on_base:
            T_ib = np.identity(4)               # mobile in origin - assume origin=base
            T_im = np.matmul(T_ib, T_bm)
        else:
            T_im = SE3(Rot_axis(3, np.pi), (0,)*3) # mobile in origin - assume origin=mobile
            T_ib = np.matmul(T_im, SE3_inv(T_bm))    # origin in base

        T_bi = SE3_inv(T_ib)

        im_o = np.divide(img_bin.shape, 2)
        points_px=list(reversed(np.subtract(np.where(img_bin), 
                                            im_o[:, np.newaxis])))
        pt_rpy = Rot2rpy(T_bi[:3,:3])
        pt_list = []
        for i_p, (x, y) in list(enumerate(zip(*points_px))):
            pt = np.multiply((x,y), resolution)
            pt_b = np.matmul(T_bi[:2,:2], pt) + T_bi[:2,3]
            pt_b = tuple(pt_b) + (height,)
            pt_list.append((pt_b, pt_rpy))
        return pt_list

    def add_to_scene(self, gscene, pt_list, gtype, radius, color=(1, 0, 0, 0.3), display=True, collision=True):
        for i_p, (pt_b, pt_rpy) in enumerate(pt_list):
            gtem = gscene.create_safe(gtype, "pt_{}".format(i_p),
                                      link_name="base_link",
                                      dims=(radius*2,) * 3,
                                      center=pt_b, rpy=pt_rpy,
                                      color=color, display=display,
                                      collision=collision, fixed=True)
            self.gtem_list.append(gtem)
        return self.gtem_list

    def set_collision(self, collision=True):
        for gtem in self.gtem_list:
            gtem.collision = collision

    def clear_from_scene(self, gscene):
        for gtem in self.gtem_list:
            if gtem in gscene:
                gscene.remove(gtem)
        self.gtem_list = []

    def check_position(self, Pb, cut=50):
        Pi = np.matmul(self.T_ib[:2, :2], Pb) + self.T_ib[:2, 3]
        Ppx = (Pi / self.resolution).astype(np.int)
        return self.cost_im[Ppx[1], Ppx[0]] < cut

def get_map_save_data(ip_cur):
    try:
        rospy.init_node("get_map_save_data")
        sock_mobile, server_thread = start_mobile_udp_thread(recv_ip=ip_cur)
        # GET MAP
        map_listener = Listener(topic_name="/map", topic_type=OccupancyGrid)
        cost_listener = Listener(topic_name="/move_base/global_costmap/costmap",
                                 topic_type=OccupancyGrid)
        lcost_listener = Listener(topic_name="/move_base/local_costmap/costmap",
                                 topic_type=OccupancyGrid)

        map_data, cost_data = None, None
        time_start = time.time()
        while map_data is None or cost_data is None:
            time.sleep(0.5)
            map_data = map_listener.last_dat
            cost_data = cost_listener.last_dat
            lcost_data = lcost_listener.last_dat
            if time.time() - time_start > TIMEOUT_GET_MAP:
                break

        if map_data is not None and cost_data is not None:
            cur_xyzw_view = get_xyzw_cur()

            save_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/map_data.pkl"), map_data)
            save_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/cost_data.pkl"), cost_data)
            save_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/lcost_data.pkl"), lcost_data)
            save_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/cur_xyzw_view.pkl"), cur_xyzw_view)
            print(GetMapResult.SUCCESS.name, end="")
        else:
            print(GetMapResult.TIMEOUT.name, end="")
    except Exception as e:
        print(e, end="")
    
import os
import sys
sys.path.append(os.path.join(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src')))

import cv2
from pkg.utils.rotation_utils import *
from pkg.geometry.geotype import GEOTYPE

# NODE_NAME = "listen_tester"
# rospy.init_node(NODE_NAME, anonymous=True)


def convert_map(map_dict):
    map_info = map_dict['info']
    map_size = (map_info['height'], map_info['width'])
    map_im = (np.reshape(map_dict['data'], map_size) + 1).astype(np.uint8)
    return map_im, map_info['resolution']


def convert_map_cm(map_dict):
    map_im, resolution = convert_map(map_dict)
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


def add_px_points(gscene, gtype, points_px, resolution, T_bi, height, radius, sample_ratio):
    rpy = Rot2rpy(T_bi[:3,:3])
    gtem_list = []
    for i_p, pt_px in list(enumerate(zip(*points_px)))[::sample_ratio]:
        pt = np.multiply(pt_px, resolution)
        pt_b = np.matmul(T_bi[:2,:2], pt) + T_bi[:2,3]
        pt_b = tuple(pt_b) +(height/2,)
        gtem = gscene.create_safe(gtype, "pt_{}".format(i_p),
                                  link_name="base_link",
                                  dims=(radius*2,) * 3,
                                  center=pt_b, rpy=rpy,
                                  color=(1, 0, 0, 0.3), display=True,
                                  collision=True, fixed=True)
        gtem_list.append(gtem)
    return gtem_list
    
if __name__ == "__main__":
    set_serving(True)
    kmm = KiroMobileMap(sys.argv[1], sys.argv[2], sys.argv[3]=='true')
    serve_forever([kmm.get_maps], verbose=True)
