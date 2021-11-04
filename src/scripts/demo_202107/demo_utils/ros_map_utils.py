from __future__ import print_function
import os
import sys

sys.path.append(os.path.join(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src')))
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src/scripts/demo_202107'))
from demo_utils.map_converter import *
from pkg.controller.trajectory_client.kiro_udp_send import start_mobile_udp_thread, get_xyzw_cur
from pkg.global_config import RNB_PLANNING_DIR
from pkg.utils.ros_utils import *
from pkg.utils.utils import *
from pkg.utils.shared_function import shared_fun, CallType, ResSpec, \
    set_serving, is_serving, serve_forever
import subprocess
import matplotlib.pyplot as plt
from tf2_msgs.msg import TFMessage

TIMEOUT_GET_MAP = 3
class GetMapResult(Enum):
    SUCCESS = 0
    TIMEOUT = 1

DEMO_UTIL_DIR = os.path.join(RNB_PLANNING_DIR, "src/scripts/demo_202107/demo_utils")

GMAP_SIZE = (384, 384)
SERVER_ID = "KiroMobileMap"

class KiroMobileMap:
    def __init__(self, master_ip, cur_ip, connection_state=True):
        self.master_ip, self.cur_ip = master_ip, cur_ip
        self.connection_state = connection_state
        self.cost_im, self.lcost_im = None, None
        if not is_serving():
            output = subprocess.Popen(['sh',
                                       os.path.join(DEMO_UTIL_DIR, 'get_ros_map.sh'),
                                       self.master_ip, self.cur_ip,
                                       str(connection_state).lower()],
                                     cwd=DEMO_UTIL_DIR)

    @shared_fun(CallType.SYNC, SERVER_ID)
    def init_node(self):
        if self.connection_state:
            rospy.init_node("map_receiver")
            print("map_receiver: node initialized")

    @shared_fun(CallType.SYNC, SERVER_ID,
                ResSpec(0, (1000000,), dict),
                ResSpec(1, (1000000,), dict),
                ResSpec(2, (1000000,), dict),
                ResSpec(3, (1000000,), dict))
    def get_maps(self):
        if self.connection_state:
            # GET MAP
            self.map_listener = Listener(topic_name="/map", topic_type=OccupancyGrid)
            self.cost_listener = Listener(topic_name="/move_base/global_costmap/costmap",
                                          topic_type=OccupancyGrid)
            self.lcost_listener = Listener(topic_name="/move_base/local_costmap/costmap",
                                           topic_type=OccupancyGrid)

            while self.map_listener.last_dat is None \
                    or self.cost_listener.last_dat is None \
                    or self.lcost_listener.last_dat is None:
                time.sleep(0.1)
            map_data = self.map_listener.last_dat
            cost_data = self.cost_listener.last_dat
            lcost_data = self.lcost_listener.last_dat
            save_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/map_data.pkl"), map_data)
            save_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/cost_data.pkl"), cost_data)
            save_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/lcost_data.pkl"), lcost_data)
            for i_tf in range(10):
                self.tf_listener = Listener(topic_name="/tf",
                                               topic_type=TFMessage)
                while self.tf_listener.last_dat is None:
                    time.sleep(0.1)
                tf_data = self.tf_listener.last_dat
                save_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/tf_data_{}.pkl".format(i_tf)), tf_data)

            map_dict = extract_attr_dict(map_data)
            cost_dict = extract_attr_dict(cost_data)
            lcost_dict = extract_attr_dict(lcost_data)
            tf_dict = extract_attr_dict(tf_data)
            self.map_listener.last_dat = self.cost_listener.last_dat = \
                    self.lcost_listener.last_dat = self.tf_listener.last_dat = None
        else:
            map_data = load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/map_data.pkl"))
            cost_data = load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/cost_data.pkl"))
            lcost_data = load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/lcost_data.pkl"))
            tf_data = load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/tf_data.pkl"))
            lcost_dict = extract_attr_dict(lcost_data)
            cost_dict = extract_attr_dict(cost_data)
            map_dict = extract_attr_dict(map_data)
            tf_dict = extract_attr_dict(tf_data)
        return lcost_dict, cost_dict, map_dict, tf_dict

    def set_maps(self, lcost_dict, cost_dict, map_dict, tf_dict, T_bm, canny_ksize=10):
        self.lcost_dict = lcost_dict
        self.map_dict = map_dict
        self.cost_dict = cost_dict
        self.lcost_im, self.lresolution = convert_map(lcost_dict)
        self.map_im, self.resolution = convert_map(map_dict)
        self.cost_im, self.resolution = convert_map(cost_dict)
        self.tf_dict = tf_dict

        ret, self.cost_bin = cv2.threshold(self.cost_im, 100, 255, cv2.THRESH_BINARY)
        self.cost_closed = cv2.morphologyEx(self.cost_bin, cv2.MORPH_CLOSE,
                                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                                      (canny_ksize, canny_ksize)))
        self.cost_canny = cv2.Canny(self.cost_closed, 50, 150)

        ret, self.lcost_bin = cv2.threshold(self.lcost_im, 100, 255, cv2.THRESH_BINARY)
        self.lcost_closed = cv2.morphologyEx(self.lcost_bin, cv2.MORPH_CLOSE,
                                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                                       (canny_ksize, canny_ksize)))
        self.lcost_canny = cv2.Canny(self.lcost_closed, 50, 150)
        self.T_bm = T_bm

        origin = self.cost_dict['info']['origin']
        self.T_bi = T_xyzquat(([origin['position'][k] for k in "xyz"],
                                [origin['orientation'][k] for k in "xyzw"]))

    def convert_im2scene(self, img_bin, resolution, T_bi, img_cost=None):
        points_idc = np.where(img_bin)
        points_px = list(reversed(points_idc))
        if img_cost is not None:
            costs = img_cost[points_idc]
        else:
            costs = None

        pt_list = []
        for i_p, (x, y) in list(enumerate(zip(*points_px))):
            pt_i = tuple(np.multiply((x, y), resolution)) + (0,)
            pt_b = np.matmul(T_bi[:3, :3], pt_i) + T_bi[:3, 3]
            pt_list.append(pt_b)
        return pt_list, costs


    @classmethod
    def add_to_scene(cls, name, gscene, pt_list, resolution, costs=None, colormap=None, color=(1,0,0,1)):
        if colormap is None:
            colormap = plt.get_cmap("YlOrRd")

        if costs is not None:
            colors = colormap(np.divide(costs.astype(float), np.max(costs)))
        else:
            colors = [color] * len(pt_list)
        mesh = gscene.create_safe(
            gtype=GEOTYPE.MESH, name=name, link_name="base_link",
            dims=(0.01,) * 3, center=(0, 0, 0), rpy=(0, 0, 0),
            color=(1, 0, 0, 1), display=True,
            collision=False, fixed=True, vertices=pt_list, scale=(resolution, resolution, 1),
            colors=colors)
        return mesh

    @classmethod
    def add_pixel_poles(cls, name, gscene, pt_list, resolution, height=2.0, color=(1, 0, 0, 0.1)):
        pt_list = np.add(pt_list, [0, 0, height / 2])
        poles = []
        for i_p, pt in enumerate(pt_list):
            poles.append(
                gscene.create_safe(GEOTYPE.BOX, "{}_{}".format(name, i_p),
                               link_name="base_link",
                               dims=(resolution, resolution, height),
                               center=pt, rpy=(0, 0, 0),
                               color=color, fixed=True, collision=True,
                               )
            )
        return poles

    @classmethod
    def remove_poles_by_box(cls, gscene, box, pt_list, Q, inside=True):
        T_bx = box.get_tf(Q)
        T_xb = SE3_inv(T_bx)
        box_dims = box.dims
        pt_list_remain = []
        for pt in pt_list:
            P_xp = np.matmul(T_xb[:3, :3], pt) + T_xb[:3, 3]
            if inside:
                res = all(np.abs(P_xp[:2]) < np.divide(box_dims[:2], 2))
            else:
                res = any(np.abs(P_xp[:2]) > np.divide(box_dims[:2], 2))
            if not res:
                pt_list_remain.append(pt)
        return pt_list_remain

    @classmethod
    def get_box_costs(cls, box, Q, T_bi, cost_im, resolution, scale=1.0):
        if len(Q) < len(box.gscene.joint_names):
            Q = np.pad(Q, (0, len(box.gscene.joint_names) - len(Q)), 'constant')
        T_bm = box.get_tf(Q)

        T_im = np.matmul(SE3_inv(T_bi), T_bm)
        verts_m = box.get_vertice_radius()[0][::2] * scale
        verts_i = np.transpose(np.matmul(T_im[:3, :3], verts_m.transpose()) + T_im[:3, 3:4])
        points_px = np.round((verts_i[:, :2] / resolution)[:, [1, 0]]).astype(int)

        cost_vals = []
        for pt in points_px:
            cost_vals.append(cost_im[pt[0], pt[1]])
        return cost_vals

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
    serve_forever(SERVER_ID, [kmm.get_maps, kmm.init_node], verbose=True)
