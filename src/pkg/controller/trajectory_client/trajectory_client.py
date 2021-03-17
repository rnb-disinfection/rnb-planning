import time
import numpy as np
from ...utils.utils import *
from ...utils.traj_utils import *
import abc


DEFAULT_TRAJ_PORT = 9980
DEFAULT_TRAJ_FREQUENCY = 50

##
# @class TrajectoryClient
# @brief TCP/IP client for trajectory Interface
# @remark use move_joint_s_curve to send off-line trajectory
#         use move_joint_s_curve_online to test online trajectory following.
class TrajectoryClient(object):

    def __init__(self, server_ip, server_port=DEFAULT_TRAJ_PORT, traj_freq=DEFAULT_TRAJ_FREQUENCY):
        self.server_ip, self.server_port, self.traj_freq = \
            server_ip, server_port, traj_freq
        ##
        # @brief periodic timer with the given traj_freq
        self.periodic = PeriodicTimer(1.0/traj_freq)
        ##
        # @brief periodic timer 4 times faster than the given traj_freq, to give margins
        self.periodic_x4 = PeriodicTimer(1.0/traj_freq/4)

    def reset(self):
        self.qcount = self.get_qcount()
        reset_dict = {'reset': 1, 'period_s': 1.0 / self.traj_freq, }
        return send_recv(reset_dict, self.server_ip, self.server_port)

    def get_qcount(self):
        return send_recv({'qcount': 0}, self.server_ip, self.server_port)['qcount']

    def get_qcur(self):
        self.qcur = send_recv({'getq': 0}, self.server_ip, self.server_port)['qval']
        return self.qcur

    def send_qval(self, qval):
        return send_recv({'qval': qval}, self.server_ip, self.server_port)

    def start_tracking(self):
        return send_recv({'follow': 1}, self.robot_ip, self.robot_port)

    def stop_tracking(self):
        return send_recv({'stop': 1}, self.robot_ip, self.robot_port)

    def terminate_loop(self):
        return send_recv({'terminate': 1}, self.robot_ip, self.robot_port)

    def move_possible_joints_x4(self, Q):
        if self.qcount >= 3:
            self.periodic_x4.wait()
        if self.qcount > 3:
            self.qcount = self.get_qcount()
            sent = False
        else:
            self.qcount = self.send_qval(Q)['qcount']
            sent = True
        return sent

    def move_joint_interpolated(self, qtar, q0=None, N_div=100, N_stop=None, start=False, linear=False, end=False):
        if N_stop is None or N_stop > N_div or N_stop<0:
            if start or linear:
                N_stop = N_div
            else:
                N_stop = N_div + 1

        qcur = np.array(self.get_qcur()) if q0 is None else q0
        DQ = qtar - qcur
        if not (linear or end):
            self.reset(q0)
        i_step = 0
        while i_step < N_stop:
            if start:
                Q = qcur + DQ * (np.sin(np.pi * (float(i_step) / N_div *0.5 - 0.5)) + 1)
            elif linear:
                Q = qcur + DQ * (float(i_step) / N_div)
            elif end:
                Q = qcur + DQ * (np.sin(np.pi * (float(i_step) / N_div *0.5 )))
            else:
                Q = qcur + DQ * (np.sin(np.pi * (float(i_step) / N_div - 0.5)) + 1) / 2
            i_step += self.move_possible_joints_x4(Q)

    ##
    # @param Q radian
    @abc.abstractmethod
    def joint_move_make_sure(self, Q):
        pass

    ##
    # @param trajectory radian
    # @param vel_lims radian/s, scalar or vector
    # @param acc_lims radian/s2, scalar or vector
    def move_joint_wp(self, trajectory, vel_lims, acc_lims):
        traj_tot = calc_safe_cubic_traj(1.0/self.traj_freq, trajectory, vel_lim=vel_lims, acc_lim=acc_lims)
        for Q in traj_tot:
            self.move_possible_joints_x4(Q)

    @abc.abstractmethod
    def start_online_tracking(self, Q0):
        pass

    @abc.abstractmethod
    def finish_online_tracking(self):
        pass


class MultiTracker:
    def __init__(self, robots, idx_list, Q0, on_rviz=False):
        assert len(robots) == len(idx_list), \
            "number of robots and indice for robot joints should be same"
        assert len(Q0) == sum([len(idx) for idx in idx_list]), \
            "total number joints does not match with inital Q"
        self.num_robots = len(robots)
        self.robots, self.idx_list, self.Q0, self.on_rviz = robots, idx_list, Q0, on_rviz

    def __enter__(self):
        if not self.on_rviz:
            for rbt, idx in zip(self.robots, self.idx_list):
                rbt.start_online_tracking(self.Q0)
            self.sent_list = [False] * self.num_robots
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not self.on_rviz:
            for rbt in self.robots:
                rbt.finish_online_tracking()

    def move_possible_joints_x4(self, Q):
        gtimer = GlobalTimer.instance()
        for i_rbt in range(self.num_robots):
            rbt, idx, sent = self.robots[i_rbt], self.idx_list[i_rbt], self.sent_list[i_rbt]
            if not sent:
                gtimer.tic("send-robot-{}".format(i_rbt))
                self.sent_list[i_rbt] = rbt.move_possible_joints_x4(Q[idx])
                gtimer.toc("send-robot-{}".format(i_rbt), stack=True)
            else:
                gtimer.tic("count-robot-{}".format(i_rbt))
                rbt.get_qcount()
                gtimer.toc("count-robot-{}".format(i_rbt), stack=True)
        sent_all = all(self.sent_list)
        if sent_all:
            self.sent_list = [False] * self.num_robots
        return sent_all