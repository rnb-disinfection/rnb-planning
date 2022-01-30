import os
import sys
from .trajectory_client  import TrajectoryClient
from ...utils.utils import *
from ...utils.rotation_utils import *


class PostechMobileClient(TrajectoryClient):

    def __init__(self, server_ip):
        TrajectoryClient.__init__(self, server_ip, traj_freq=10)

    def get_qcur(self):
        return np.array(list(send_recv({"pos": 1}, self.server_ip, self.server_port)["pos"])+[0,0,0])

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
    def joint_move_make_sure(self, Q, sure_count=None, Qfinal=None, check_valid=1, *args, **kwargs):
        send_recv()
        if self.gscene is not None:
            if sure_count==0:
                arrow_name = "mob_tar_{}".format(check_valid)
                color = (1,1,0,0.5)
            else:
                arrow_name = "mob_tar_fin"
                color = (1,0,0,0.5)
            self.gscene.add_highlight_axis(arrow_name, arrow_name, link_name="base_link",
                                      center=list(Q[:2])+[0],
                                      orientation_mat=Rot_axis(3, Q[2]),
                                      color=color, axis="x",
                                      dims=(0.6, 0.4, 0.01))
            time.sleep(0.1)
        try:
            Q = np.copy(Q)
            Qcur = self.get_qcur()
            diff = np.subtract(Q[:3], Qcur[:3])
            diff[2] = Rot2axis(Rot_axis(3, diff[2]), 3)
            diff_nm_p = np.linalg.norm(diff[:2])
            diff_nm = np.linalg.norm(diff)

            if Qfinal is None:
                Qfinal = np.copy(Q)

            if sure_count is None: # default call
                sure_count = self.sure_count_default # need to adjust finely

            if check_valid:
                if diff_nm > self.NEAR_MOTION_RANGE and not self.check_valid(Qcur):
                    cur_val = self.coster(Qcur)
                    min_Q, min_val = self.get_best_near_point(Qcur)
                    TextColors.BLUE.println("[INFO] Depart via: {} ({}) <- {} ({})".format(
                        np.round(min_Q[:3], 2), min_val, np.round(Qcur[:3], 2), cur_val))
                    self.joint_move_make_sure(min_Q, sure_count=0, check_valid=check_valid-1)

                if diff_nm > self.NEAR_MOTION_RANGE and not self.check_valid(Q):
                    cur_val = self.coster(Q)
                    min_Q, min_val = self.get_best_near_point(Q)
                    TextColors.BLUE.println("[INFO] Approach through: {} ({}) -> {} ({})".format(
                        np.round(min_Q[:3], 2), min_val, np.round(Q[:3], 2), cur_val))
                    self.joint_move_make_sure(min_Q, sure_count=0, check_valid=check_valid-1)

            if KIRO_UDP_OFFLINE_DEBUG:
                self.xyzw_last = self.joints2xyzw(Q)
                time.sleep(0.5)
            else:
                if diff_nm <= self.allowance:
                    return

                send_pose_udp(self.sock_mobile, self.joints2xyzw(Q), send_ip=self.server_ip)
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
                                                  np.linalg.norm(Qcur[:3]-Qfinal[:3])))
            if sure_count>0:
                print("sure_count={}".format(sure_count))
                Qadj = np.copy(Q)
                diff_origin = Qfinal[:2] - Qcur[:2]
                diff_nm_origin = np.linalg.norm(diff_origin)
                diff_cur = Q[:2] - Qcur[:2]
                diff_nm_cur = np.linalg.norm(diff_cur)
                if diff_nm_origin > self.allowance: # if distance from original goal > alpha
                    Qadj[:2] = Qadj[:2] + diff_cur/diff_nm_cur*self.allowance # add alpha distance to current difference
                    time.sleep(0.5)
                    self.joint_move_make_sure(Qadj, sure_count=sure_count-1, Qfinal=Qfinal, check_valid=0)
        finally:
            if self.gscene is not None:
                self.gscene.clear_highlight(arrow_name)
                time.sleep(0.1)


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

    def pause_tracking(self):
        return {}

    def resume_tracking(self):
        return {}

    ##
    # @brief override stop_tracking in IndyDCPClient. reset the robot and trajectory client, and stop tracking.
    # @remark   reset_robot is added here because it resets the internal robot pose reference.
    #           If reset_robot is not called, it will immediately move to the original reference pose.
    def stop_tracking(self):
        return {}
