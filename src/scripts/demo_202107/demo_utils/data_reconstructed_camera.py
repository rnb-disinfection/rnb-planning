import os
import sys
sys.path.append(os.path.join(os.path.join(
    os.environ["RNB_PLANNING_DIR"], 'src')))

from pkg.global_config import *
from pkg.detector.camera.camera_interface import CameraInterface
from pkg.detector.multiICP.multiICP import *
from pkg.utils.utils import *

SCENE_DATA_PATH = os.path.join(RNB_PLANNING_DIR, "data", "scene_data")
try_mkdir(SCENE_DATA_PATH)


class DataRecontructedCamera(CameraInterface):

    def __init__(self, crob, viewpoint, datetime_load=None):
        self.crob, self.viewpoint = crob, viewpoint
        if datetime_load is None:
            datetime_list = os.listdir(SCENE_DATA_PATH)
            datetime_load = sorted(datetime_list)[-1]
        self.datetime_load = datetime_load

    ##
    # @brief   Load images and config, make cam_pose-image Map, load
    def initialize(self):
        data_path = os.path.join(SCENE_DATA_PATH, self.datetime_load)
        config_dat = load_pickle(data_path + "/config.pkl")
        self.cameraMatrix, self.distCoeffs, self.depth_scale = \
            config_dat['cameraMatrix'], config_dat['distCoeffs'], config_dat['depth_scale']

        cam_pose_list = []
        color_depth_list = []
        for fname in sorted(os.listdir(data_path)):
            if fname.endswith("pkl") and fname != "config.pkl":
                scene_dat = load_pickle(os.path.join(data_path, fname))
                color, depth, cam_pose = \
                    scene_dat['color'], scene_dat['depth'], scene_dat['cam_pose']
                cam_pose_list.append(cam_pose)
                color_depth_list.append((color, depth))
        self.cam_pose_list = np.stack(cam_pose_list)
        self.color_depth_list = color_depth_list

    def get_nearest_data(self, T):
        idx = np.argmin(
            np.linalg.norm(
                matmul_md(
                    self.cam_pose_list, np.linalg.inv(T)) - np.identity(4),
                axis=(1, 2)
            )
        )
        return self.color_depth_list[idx] + (self.cam_pose_list[idx],)

    ##
    # @brief   function prototype to get camera configuration
    # @return  cameraMatrix 3x3 camera matrix in pixel units,
    # @return  distCoeffs distortion coefficients, 5~14 float array
    # @return  depth_scale scaling constant to be multiplied to depthmap(int) to get m scale values
    def get_config(self):
        return self.cameraMatrix, self.distCoeffs, self.depth_scale

    def disconnect(self):
        pass

    ##
    # @brief   Get image from nearest data point
    def get_image(self):
        T = self.viewpoint.get_tf(self.crob.get_real_robot_pose())
        return self.get_nearest_data(T)[0]

    ##
    # @brief   Get depthmap from nearest data point and transform model point
    def get_depthmap(self):
        T = self.viewpoint.get_tf(self.crob.get_real_robot_pose())
        color_dat, depth_dat, Tdat = self.get_nearest_data(T)
        depthmap = self.transform_depthmap(color_dat, depth_dat, Tdat, T)
        return depthmap

    ##
    # @brief   Get image and depthmap from nearest data point and transform model point
    def get_image_depthmap(self):
        T = self.viewpoint.get_tf(self.crob.get_real_robot_pose())
        color_dat, depth_dat, Tdat = self.get_nearest_data(T)
        depthmap = self.transform_depthmap(color_dat, depth_dat, Tdat, T)
        return color_dat, depthmap

    def transform_depthmap(self, color_dat, depth_dat, Tc_dat, Tc_to, depth_trunc=10.0):
        pcd = cdp2pcd(
            ColorDepthMap(color_dat, depth_dat,
                          cammat2intrins(self.cameraMatrix, tuple(reversed(depth_dat.shape[:2]))),
                          self.depth_scale), depth_trunc=depth_trunc)

        Tbi, Tbn = Tc_dat, Tc_to
        Tni = np.matmul(np.linalg.inv(Tbn), Tbi)
        points_n = np.matmul(pcd.points, Tni[:3, :3].transpose()) + Tni[:3, 3]

        ptz = np.matmul(self.cameraMatrix, np.transpose(points_n)).transpose()
        pt_list = np.round(ptz[:, :2] / ptz[:, 2:3]).astype(int)

        depthmap = np.ones_like(depth_dat, dtype=float) * 1e3
        for pt, z in zip(pt_list, ptz[:, 2]):
            depthmap[pt[1], pt[0]] = min(depthmap[pt[1], pt[0]], z)
        depthmap[np.where(depthmap == 1e3)] = 0
        depthmap = (depthmap / self.depth_scale).astype(depth_dat.dtype)
        return depthmap

    ##
    # @brief   Save Camera Configuration and reset scene index
    def ready_saving(self, cameraMatrix, distCoeffs, depth_scale):
        self.data_path = os.path.join(SCENE_DATA_PATH, get_now())
        try_mkdir(self.data_path)
        save_pickle(self.data_path + "/config.pkl",
                    {"cameraMatrix": cameraMatrix,
                     "distCoeffs": distCoeffs,
                     "depth_scale": depth_scale})
        self.idx = 0

    ##
    # @brief   Save colormap and depthmap with camera pose.
    def save_scene(self, color, depth, cam_pose):
        self.idx += 1
        save_pickle(self.data_path + "/{:08}.pkl".format(self.idx),
                    {"color": color,
                     "depth": depth,
                     "cam_pose": cam_pose})
