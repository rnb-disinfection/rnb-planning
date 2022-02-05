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
        return np.array(list(send_recv({"pos": 1}, self.server_ip, self.server_port)["pos"])
                        +[0,0,0])

    def send_qval(self, qval):
        return send_recv({'goal': qval[:3]}, self.server_ip, self.server_port)

    def terminate_loop(self):
        return send_recv({'terminate': 1}, self.server_ip, self.server_port)

    ##
    # @brief    send s-surve trajectory on-line to move joint to target position.
    #           To test on-line trajectory following for adaptive motion
    # @param qtar         target joint configuration
    # @param N_div          the number of divided steps (default=100)
    # @param start_tracking to reset trajectory and start tracking
    # @param auto_stop      auto-stop trajectory-following after finishing the motion
    def move_joint_s_curve_online(self, qtar, q0=None, N_div=100, auto_stop=True):
        raise (RuntimeError("move_joint_s_curve_online is not supported with PostechMobileClient"))

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
