import time
import numpy as np
import rospy
from ...utils.utils import *
import abc

DEFAULT_PORT_REPEATER = 1189
DEFAULT_TRAJ_FREQUENCY = 50

class Repeater(object):

    def __init__(self, repeater_ip, disable_getq,
                 repeater_port=DEFAULT_PORT_REPEATER, traj_freq=DEFAULT_TRAJ_FREQUENCY):
        self.repeater_ip, self.disable_getq, self.repeater_port, self.traj_freq = \
            repeater_ip, disable_getq, repeater_port, traj_freq
        self.rate_x1 = rospy.Rate(traj_freq)
        self.rate_x4 = rospy.Rate(traj_freq*2)
        self.rate_x4 = rospy.Rate(traj_freq*4)

    def reset(self, q0=None, update_params=False, upload_params=False):
        if upload_params:
            self.update_params(download=False)
        if update_params:
            self.update_params(download=True)

        self.qcount = self.get_qcount()
        reset_dict = {'reset': 1, 'period_s': 1.0 / self.traj_freq, }
        if self.disable_getq:
            if q0 is None:
                raise(RuntimeError("need q0 value when getq function is disabled"))
            reset_dict.update({'qcur': q0})
        else:
            self.get_qcur()
        return send_recv(reset_dict, self.repeater_ip, self.repeater_port)

    def update_params(self, download=True):
        if download:
            self.alpha_lpf = self.k_gain = self.d_gain = self.v_sat = self.a_sat = -1
        self.set_alpha_lpf(self.alpha_lpf)
        self.set_k_gain(self.k_gain)
        self.set_d_gain(self.d_gain)
        self.set_v_sat(self.v_sat)
        self.set_a_sat(self.a_sat)

    def set_alpha_lpf(self, alpha_lpf):
        self.alpha_lpf = send_recv({'alpha_lpf': alpha_lpf}, self.repeater_ip, self.repeater_port)['alpha_lpf']
        return self.alpha_lpf

    def set_k_gain(self, k_gain):
        self.k_gain = send_recv({'k_gain': k_gain}, self.repeater_ip, self.repeater_port)['k_gain']
        return self.k_gain

    def set_d_gain(self, d_gain):
        self.d_gain = send_recv({'d_gain': d_gain}, self.repeater_ip, self.repeater_port)['d_gain']
        return self.d_gain

    def set_v_sat(self, v_sat):
        self.v_sat = send_recv({'v_sat': v_sat}, self.repeater_ip, self.repeater_port)['v_sat']
        return self.v_sat

    def set_a_sat(self, a_sat):
        self.a_sat = send_recv({'a_sat': a_sat}, self.repeater_ip, self.repeater_port)['a_sat']
        return self.a_sat

    def get_qcount(self):
        return send_recv({'qcount': 0}, self.repeater_ip, self.repeater_port)['qcount']

    def get_qcur(self):
        if self.disable_getq:
            raise(RuntimeError("getq function is disabled"))
        self.qcur = send_recv({'getq': 0}, self.repeater_ip, self.repeater_port)['qval']
        return self.qcur

    def send_qval(self, qval):
        return send_recv({'qval': qval}, self.repeater_ip, self.repeater_port)

    def stop_tracking(self):
        return send_recv({'stop': 1}, self.repeater_ip, self.repeater_port)

    def terminate_loop(self):
        return send_recv({'terminate': 1}, self.repeater_ip, self.repeater_port)

    def move_possible_joints_x4(self, Q):
        if self.qcount >= 3:
            self.rate_x4.sleep()
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
    # @param vel_limits radian
    # @param acc_limits radian
    def move_joint_wp(self, trajectory, vel_limits, acc_limits):
        Q_prev = trajectory[0]
        len_traj = len(trajectory)
        start = True
        for i in range(len_traj):
            Q_cur = trajectory[i]
            diff_abs = np.abs(Q_cur - Q_prev)
            max_diff = np.max(diff_abs, axis=0)
            if max_diff <= 1e-3:
                continue
            T_vmax = np.max(max_diff / vel_limits)
            T_amax = np.sqrt(np.max(2 * max_diff / acc_limits))
            T = np.maximum(T_vmax, T_amax)
            end = (i == len_traj - 1)
            self.move_joint_interpolated(Q_cur, Q_prev,
                                         N_div=np.ceil(T * float(self.traj_freq * 4)),
                                         start=start, linear=not (start or end), end=end)
            start = False
            Q_prev = Q_cur

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