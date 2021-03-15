from .repeater import *
from .indy_utils.indydcp_client import IndyDCPClient
from .indy_utils.indy_program_maker import JsonProgramComponent
from functools import wraps


DEFAULT_TRAJ_PORT = 9980
DEFAULT_TRAJ_FREQUENCY = 50

INDY_DOF = 6
INDY_CONTROL_FREQ = 4000

def connect_indy(func):
    @wraps(func)
    def __wrapper(indy, *args, **kwargs):
        if "connect" in kwargs:
            connect = kwargs["connect"]
            del kwargs["connect"]
        else:
            connect = False
        if connect: indy.connect()
        res = func(indy, *args, **kwargs)
        if connect: indy.disconnect()
        return res

    return __wrapper

class indytraj_client(IndyDCPClient, Repeater):

    def __init__(self, server_ip,
                 traj_port=DEFAULT_TRAJ_PORT,
                 traj_freq=DEFAULT_TRAJ_FREQUENCY, *args, **kwargs):
        kwargs_indy, kwargs_otic = divide_kwargs(kwargs, IndyDCPClient.__init__, Repeater.__init__)
        IndyDCPClient.__init__(self, *args, server_ip=server_ip, **kwargs_indy)
        Repeater.__init__(self, repeater_ip=self.server_ip, disable_getq=True, **kwargs_otic)
        self.indy_grasp_DO = 0
        self.traj_port = traj_port
        self.traj_freq = traj_freq
        self.period_s = 1.0/traj_freq

    def reset(self):
        self.qcount = self.get_qcount()
        reset_dict = {'reset': True, 'period_s': self.period_s}
        return send_recv(reset_dict, self.server_ip, self.traj_port)

    def get_qcount(self):
        return send_recv({'qcount': 0}, self.server_ip, self.traj_port)['qcount']

    def get_qcur(self):
        self.qcur = np.array(send_recv({'getq': 0}, self.server_ip, self.traj_port)['qval'])
        return self.qcur

    def send_qval(self, qval):
        return send_recv({'qval': qval}, self.server_ip, self.traj_port)

    def start_tracking(self):
        return send_recv({'follow': 1}, self.server_ip, self.traj_port)

    def stop_tracking(self):
        return send_recv({'stop': 1}, self.server_ip, self.traj_port)

    def terminate_loop(self):
        return send_recv({'terminate': 1}, self.server_ip, self.traj_port)

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
            self.reset()
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
    # @param trajectory radian
    # @param vel_lims radian/s, scalar or vector
    # @param acc_lims radian/s2, scalar or vector
    def move_joint_wp(self, trajectory, vel_lims, acc_lims, wait_finish=True, auto_stop=False):
        traj_tot = calc_safe_cubic_traj(1.0/self.traj_freq, trajectory, vel_lim=vel_lims, acc_lim=acc_lims)
        self.reset()
        self.start_tracking()
        for Q in traj_tot:
            self.move_possible_joints_x4(Q)
        if auto_stop:
            self.stop_tracking()

    def connect_and(self, func, *args, **kwargs):
        with self:
            return func(*args, **kwargs)

    @connect_indy
    def wait_motion(self, period=1e-1):
        while not self.get_robot_status()['movedone']:
            time.sleep(period)

    ##
    # @param Q radian
    def joint_move_make_sure(self, Q, N_repeat=2):
        with self:
            for _ in range(N_repeat):
                self.joint_move_to(np.rad2deg(Q))
                self.wait_motion()

    @connect_indy
    def reset_robot(self):
        IndyDCPClient.reset_robot(self)

    def start_online_tracking(self, q0, connect=True):
        if connect:
            self.connect()
        self.move_ext_traj_txt(traj_type=1, traj_freq=INDY_CONTROL_FREQ, dat_size=INDY_DOF,
                               traj_data=q0.tolist() + [0] * 2 * INDY_DOF)
        ret = self.reset(q0, update_params=True)
        if connect:
            self.disconnect()
        return ret

    def finish_online_tracking(self, connect=True):
        self.stop_tracking()
        self.terminate_loop()
        if connect:
            self.connect()
        self.stop_motion()
        if connect:
            self.disconnect()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    @connect_indy
    def wait_di(self, idx, period=1e-1):
        while True:
            time.sleep(period)
            if self.get_di()[idx]:
                break

    @connect_indy
    def grasp(self, grasp, depth=0.009):
        gstate = self.get_endtool_do(0)
        if gstate != grasp:
            self.set_endtool_do(0, grasp)
            time.sleep(0.5)

## do vacuum mode
    # @connect_indy
    # def grasp(self, grasp, depth=0.009):
    #     gstate = self.get_do()[self.indy_grasp_DO]
    #     if gstate != grasp:
    #         self.set_do(self.indy_grasp_DO, int(grasp))
    #         if grasp:
    #             uvw_cur = self.get_task_pos()[3:]
    #             Rbe_cur = Rot_zyx(*np.deg2rad(uvw_cur)[[2, 1, 0]])
    #             time.sleep(0.1)
    #             self.task_move_by(np.matmul(Rbe_cur, [0, 0, depth]).tolist() + [0] * 3)
    #             time.sleep(0.1)
    #             self.wait_motion()
    #             self.task_move_by(np.matmul(Rbe_cur, [0, 0, -depth]).tolist() + [0] * 3)
    #             time.sleep(0.1)
    #             self.wait_motion()

