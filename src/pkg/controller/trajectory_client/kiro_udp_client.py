import os
import sys

sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src/scripts/milestone_202110'))


from pkg.controller.trajectory_client.trajectory_client  import TrajectoryClient
from pkg.utils.utils import *
from pkg.utils.rotation_utils import *
# from .trajectory_client import TrajectoryClient
# from ...utils.utils import *
# from ...utils.rotation_utils import *
from kiro_udp_send import start_mobile_udp_thread, get_reach_state_edgeup, send_pose_udp, get_xyzw_cur


class KiroUDPClient(TrajectoryClient):
    DURATION_SHORT_MOTION_REF = 5
    SHORT_MOTION_RANGE = 0.04
    def __init__(self, server_ip, ip_cur, dummy=False):
        TrajectoryClient.__init__(self, server_ip, traj_freq=10)
        self.server_ip, self.dummy = server_ip, dummy
        self.teleport = True
        if not dummy:
            self.sock_mobile, self.server_thread = start_mobile_udp_thread(recv_ip=ip_cur)
            time.sleep(1)
        self.xyzw_last = [0, 0, 0, 1]
        self.validifier = None
        self.tool_angle = 0
        self.sure_count_default = 0
        self.allowance = 2e-2

    def xyzw2joints(self, xyzw):
        Q_CUR = np.array([0] * 6, dtype=np.float)
        Q_CUR[:2] = xyzw[:2]
        Q_CUR[2] = Rot2axis(Rotation.from_quat((0, 0, xyzw[2], xyzw[3])).as_dcm(), 3)
        return Q_CUR

    def joints2xyzw(self, Q):
        xyzw = tuple(Q[:2]) + tuple(Rotation.from_dcm(Rot_axis(3, Q[2])).as_quat()[2:])
        return xyzw

    def get_qcount(self):
        # print("kmb qcount: {}".format(not get_reach_state_edgeup()))
        return not get_reach_state_edgeup()

    def get_qcur(self):
        if self.dummy:
            cur_xyzw = self.xyzw_last
        else:
            cur_xyzw = get_xyzw_cur()
        return self.xyzw2joints(cur_xyzw)

    def send_qval(self, qval):
        raise (RuntimeError("send_qval is not supported with KiroUDPClient"))

    def terminate_loop(self):
        raise (RuntimeError("terminate_loop is not supported with KiroUDPClient"))

    ##
    # @brief    Send target pose to the server and store the queue count.
    # @param online If this flag is set True, it will wait the queue on the server to sync the motion.
    def push_Q(self, Q, online=False):
        raise (RuntimeError("push_Q is not supported with KiroUDPClient"))

    ##
    # @brief publish qtar trajectory will be generated on the mobile robot
    def move_joint_s_curve(self, qtar, *args, **kwargs):
        self.joint_move_make_sure(qtar)

    ##
    # @brief    send s-surve trajectory on-line to move joint to target position.
    #           To test on-line trajectory following for adaptive motion
    # @param qtar         target joint configuration
    # @param N_div          the number of divided steps (default=100)
    # @param start_tracking to reset trajectory and start tracking
    # @param auto_stop      auto-stop trajectory-following after finishing the motion
    def move_joint_s_curve_online(self, qtar, q0=None, N_div=100, auto_stop=True):
        raise (RuntimeError("move_joint_s_curve_online is not supported with KiroUDPClient"))

    ##
    # @param trajectory radian
    # @return interpolated trajecotry, expected motion time
    def move_joint_wp(self, trajectory, *args, **kwargs):
        if self.teleport:
            traj_wps = [trajectory[-1]]
        else:
            trajectory = np.concatenate([[self.get_qcur()], trajectory])
            traj_wps = simplify_traj(trajectory, step_fractions=[0, 1])

        #         self.joint_waypoint_clean()
        for Q in traj_wps:
            self.joint_move_make_sure(Q)
        return traj_wps, float(len(traj_wps)) / self.traj_freq

    ##
    # @brief move joint with waypoints, one-by-one
    # @param trajectory numpy array (trajectory length, joint num)
    def move_joint_traj(self, trajectory, auto_stop=True, wait_motion=True):
        if not wait_motion:
            TextColors.RED.println("KiroUDPClient always wait for motion")
        self.move_joint_wp(trajectory)

    ##
    # @brief Make sure the joints move to Q using the indy DCP joint_move_to function.
    # @param Q radian
    def joint_move_make_sure(self, Q, sure_count=None, Qorigin=None, *args, **kwargs):
        if sure_count is None:
            sure_count = self.sure_count_default
        Q = np.copy(Q)
        Qcur = self.get_qcur()
        diff = np.subtract(Q[:3], Qcur[:3])
        diff[2] = Rot2axis(Rot_axis(3, diff[2]), 3)
        diff_nm_p = np.linalg.norm(diff[:2])
        diff_nm = np.linalg.norm(diff)

        NEAR_MOTION_RANGE = 0.4

        if Qorigin is None:
            Qorigin = np.copy(Q)
        if diff_nm_p > NEAR_MOTION_RANGE and self.validifier and not self.validifier(Q):
            Qapp = np.copy(Q)
            for _ in range(100):
                r, th = np.random.uniform([0, -np.pi], [NEAR_MOTION_RANGE, np.pi])
                x, y = np.matmul(Rot_axis(3, th)[:2, :2], [r, 0])
                Qapp[:2] = Q[:2] + [x,y]
                ret = self.validifier(Qapp)
                if ret:
                    break
            if ret:
                print("[INFO] Approach through: {} -> {}".format(Qapp[:3], Q[:3]))
                self.joint_move_make_sure(Qapp, sure_count=0)
            else:
                TextColors.RED.println("[WARN] No available approach position. Try anyway")

        if self.dummy:
            self.xyzw_last = self.joints2xyzw(Q)
        else:
            if diff_nm <= self.allowance:
                return

            send_pose_udp(self.sock_mobile, self.joints2xyzw(Q),
                          tool_angle=self.tool_angle, send_ip=self.server_ip)
            print("Distance={} ({})".format(diff_nm, np.round(diff, 3)))
            if diff_nm < self.SHORT_MOTION_RANGE:
                timeout_short = (diff_nm / self.SHORT_MOTION_RANGE) * self.DURATION_SHORT_MOTION_REF
                TextColors.YELLOW.println("[WARN] TOO SMALL VARIANCE, REDUCE TIMEOUT to {:.3}".format(timeout_short))
                self.wait_queue_empty(timeout_short)
            else:
                self.wait_queue_empty(60)
            time.sleep(1)
            Qcur = self.get_qcur()
            print("End up at={} ({:.3} / {:.3})".format(np.round(Qcur[:3], 3),
                                                  np.linalg.norm(Qcur[:3]-Q[:3]),
                                                  np.linalg.norm(Qcur[:3]-Qorigin[:3])))
        if sure_count>0:
            print("sure_count={}".format(sure_count))
            Qadj = np.copy(Q)
            diff_origin = Qorigin[:2] - Qcur[:2]
            diff_nm_origin = np.linalg.norm(diff_origin)
            diff_cur = Q[:2] - Qcur[:2]
            diff_nm_cur = np.linalg.norm(diff_cur)
            if diff_nm_origin > self.allowance: # if distance from original goal > alpha
                Qadj[:2] = Qadj[:2] + diff_cur/diff_nm_cur*self.allowance # add alpha distance to current difference
                time.sleep(0.5)
                self.joint_move_make_sure(Qadj, sure_count=sure_count-1, Qorigin=Qorigin)

    ##
    # @brief Surely move joints to Q using the indy DCP joint_move_to function.
    # @param Q radian
    def grasp(self, grasp):
        return

    ##
    # @brief reset robot and trajectory client
    def reset(self):
        self.qcount = self.get_qcount()

    def start_tracking(self):
        return {}

    ##
    # @brief override stop_tracking in IndyDCPClient. reset the robot and trajectory client, and stop tracking.
    # @remark   reset_robot is added here because it resets the internal robot pose reference.
    #           If reset_robot is not called, it will immediately move to the original reference pose.
    def stop_tracking(self):
        return {}

if __name__ == "__main__":
    kmb = KiroUDPClient('192.168.0.102', '192.168.0.8')
    while True:
        print(get_xyzw_cur())
        time.sleep(0.5)

