import os
import sys
from .trajectory_client  import TrajectoryClient
from ...utils.utils import *
from ...utils.rotation_utils import *

MOBILE_PORT = 1785

class PostechMobileClient(TrajectoryClient):

    def __init__(self, server_ip):
        TrajectoryClient.__init__(self, server_ip, server_port=MOBILE_PORT, traj_freq=10)

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
        pass


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
