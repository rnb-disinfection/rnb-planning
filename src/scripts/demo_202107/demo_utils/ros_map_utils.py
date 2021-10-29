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
from demo_utils.map_converter import *
import cv2
import subprocess

TIMEOUT_GET_MAP = 3
class GetMapResult(Enum):
    SUCCESS = 0
    TIMEOUT = 1

DEMO_UTIL_DIR = os.path.join(RNB_PLANNING_DIR, "src/scripts/demo_202107/demo_utils")
class KiroMobileMap:
    def __init__(self, master_ip, cur_ip):
        self.master_ip, self.cur_ip = master_ip, cur_ip

    def update_map(self):
        output = subprocess.check_output(['sh', os.path.join(DEMO_UTIL_DIR, 'get_ros_map.sh'), self.master_ip, self.cur_ip],
                                 cwd=DEMO_UTIL_DIR)
        if output.endswith(GetMapResult.SUCCESS.name):
            print("UPDATE MAP SUCCEEDED")
        else:
            TextColors.RED.println("==== UPDATE MAP FAILED ====")
            print(output)
        self.load_map()

    def load_map(self, canny_ksize=10):
        self.map_data = load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/map_data.pkl"))
        self.cost_data = load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/cost_data.pkl"))
        self.xyzw_view = load_pickle(os.path.join(os.environ["RNB_PLANNING_DIR"],"data/cur_xyzw_view.pkl"))
        self.map_im, self.resolution = convert_map(self.map_data)
        self.cost_im, self.resolution = convert_map(self.cost_data)
        ret, self.cost_bin = cv2.threshold(self.cost_im, 100, 255, cv2.THRESH_BINARY)
        self.cost_closed = cv2.morphologyEx(self.cost_bin, cv2.MORPH_CLOSE,
                                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                                      (canny_ksize,canny_ksize)))
        self.cost_canny = cv2.Canny(self.cost_closed, 50, 150)
        self.origin = ((lambda x: (-x.x, -x.y, x.z)) (self.cost_data.info.origin.position),
                       (lambda x: (x.x, x.y, x.z, x.w)) (self.cost_data.info.origin.orientation))

    def convert_im2scene(self, img_bin, gscene, mobile_base, height):
        T_io = T_xyzquat(self.origin)

        cur_xyzw = self.xyzw_view
        Q_ref = np.array([0]*len(gscene.joint_names), dtype=np.float)
        Q_CUR = np.copy(Q_ref)
        Q_CUR[:2] = cur_xyzw[:2]
        Q_CUR[2] = Rot2axis(Rotation.from_quat((0,0,cur_xyzw[2], cur_xyzw[3])).as_dcm(), 3)
        gscene.show_pose(Q_CUR)
        T_om = T_xyzquat(((tuple(cur_xyzw[:2])+(0,)), ((0,0)+tuple(cur_xyzw[2:]))))
        T_bm = gscene.get_tf(mobile_base, Q_CUR)
        T_bo = np.matmul(T_bm, SE3_inv(T_om))

        T_im = matmul_series(T_io,
                             SE3(Rot_axis(1, np.pi), (0,0,0)),
                             T_om)

        self.T_bi = matmul_series(T_bm, SE3_inv(T_im))
        self.T_ib = SE3_inv(self.T_bi)


        points_px=list(reversed(np.where(img_bin)))
        self.pt_rpy = Rot2rpy(self.T_bi[:3,:3])
        self.pt_list = []
        for i_p, pt_px in list(enumerate(zip(*points_px))):
            pt = np.multiply(pt_px, self.resolution)
            pt_b = np.matmul(self.T_bi[:2,:2], pt) + self.T_bi[:2,3]
            pt_b = tuple(pt_b) + (height,)
            self.pt_list.append(pt_b)
        return self.pt_list

    def add_to_scene(self, gscene, pt_list, gtype, radius, color=(1, 0, 0, 0.3), display=True, collision=True):
        self.gtem_list = []
        for i_p, pt_b in enumerate(pt_list):
            gtem = gscene.create_safe(gtype, "pt_{}".format(i_p),
                                      link_name="base_link",
                                      dims=(radius*2,) * 3,
                                      center=pt_b, rpy=self.pt_rpy,
                                      color=color, display=display,
                                      collision=collision, fixed=True)
            self.gtem_list.append(gtem)
        return self.gtem_list

    def set_collision(self, collision=True):
        for gtem in self.gtem_list:
            gtem.collision = collision

    def clear_from_scene(self, gscene):
        for gtem in self.gtem_list:
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
    
    
    
if __name__ == "__main__":
    get_map_save_data(sys.argv[1])