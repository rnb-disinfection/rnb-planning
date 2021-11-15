import os
import sys

from ...utils.utils_python3 import SE3_inv
from ...utils.shared_function import shared_fun, CallType, ArgSpec, ResSpec, serve_forever, SHARED_FUNC_ALL
from ...global_config import RNB_PLANNING_DIR
RNB_TRACK_DIR = os.path.join(os.environ['RNB_TRACK_DIR'])
sys.path.append(RNB_TRACK_DIR)

__SERVER_ID = "SharedTrackers"


class SharedTrackers:
    def __init__(self):
        if __name__ != "__main__":
            output = subprocess.Popen(['python3',
                                       os.path.join(RNB_PLANNING_DIR, 'src/pkg/detector/vive/shared_vive.py')])

    @shared_fun(CallType.SYNC, __SERVER_ID)
    def init_vive(self):
        self.vive = triad_openvr.triad_openvr()
        self.vive.print_discovered_objects()
        self.tracker_names = self.vive.object_names["Tracker"]
        self.ref_name = None
        self.tf_base_ref = None

    @shared_fun(CallType.SYNC, __SERVER_ID,
                ArgSpec("ref_name", (32,), str),
                ArgSpec("tf_base_ref", (4,4), float))
    def set_reference(self, ref_name, tf_base_ref):
        if ref_name not in self.tracker_names:
            raise(RuntimeError("non-registered reference tracker name"))
        self.ref_name = ref_name
        self.tf_base_ref = np.copy(tf_base_ref)

    @shared_fun(CallType.SYNC, __SERVER_ID,
                ResSpec(0, (10000,), dict))
    def get_all_pose(self):
        pose_dict = {}
        if self.ref_name is not None:
            if self.ref_name not in self.tracker_names:
                raise(RuntimeError("non-registered reference tracker name"))
            T_vref = self.vive.devices[self.ref_name].get_pose_matrix()
            T_bv = np.matmul(self.tf_base_ref, SE3_inv(T_vref))
        else:
            T_bv = np.identity(4)

        for tname in self.tracker_names:
            pose = self.vive.devices[tname].get_pose_matrix()
            pose_dict[tname] = np.matmul(T_bv, pose)
        return pose_dict

    @shared_fun(CallType.SYNC, __SERVER_ID,
                ArgSpec("tname", (32,), str),
                ResSpec(0, (4,4), float))
    def get_pose(self, tname):
        if self.ref_name is not None:
            if self.ref_name not in self.tracker_names:
                raise(RuntimeError("non-registered reference tracker name"))
            T_vref = self.vive.devices[self.ref_name].get_pose_matrix()
            T_bv = np.matmul(self.tf_base_ref, SE3_inv(T_vref))
        else:
            T_bv = np.identity(4)
        if tname not in self.vive.devices:
            return None
        else:
            return np.matmul(T_bv, self.vive.devices[tname].get_pose_matrix())


if __name__ == "__main__":
    vvt = SharedTrackers()
    serve_forever(__SERVER_ID, [vvt.get_all_pose, vvt.get_all_pose], verbose=True)
