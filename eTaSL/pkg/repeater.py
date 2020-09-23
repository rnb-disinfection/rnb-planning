import time
import numpy as np
import rospy
from pkg.utils import *

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

    def move_joint_possible_x4(self, Q):
        if self.qcount >= 3:
            self.rate_x4.sleep()
        if self.qcount > 3:
            self.qcount = self.get_qcount()
            sent = False
        else:
            self.qcount = self.send_qval(Q)['qcount']
            sent = True
        return sent

    def move_joint_interpolated(self, qtar, q0=None, N_div=100, N_stop=None):
        if N_stop is None or N_stop > N_div or N_stop<0:
            N_stop = N_div + 1

        qcur = np.array(self.get_qcur()) if q0 is None else q0
        DQ = qtar - qcur

        self.reset(q0)
        i_step = 0
        while i_step < N_stop:
            Q = qcur + DQ * (np.sin(np.pi * (float(i_step) / N_div - 0.5)) + 1) / 2
            i_step += self.move_joint_possible_x4(Q)