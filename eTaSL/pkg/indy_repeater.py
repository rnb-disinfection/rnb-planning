import matplotlib.pyplot as plt
import time
import numpy as np
from .rotation_utils import *
from .repeater import *
from indy_utils.indydcp_client import IndyDCPClient
from functools import wraps

INDY_DOF = 6
INDY_CONTROL_FREQ = 4000
DEFAULT_INDY_IP = '192.168.0.63'

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

    def __init__(self, server_ip=DEFAULT_INDY_IP, *args, **kwargs):
        kwargs_indy, kwargs_otic = divide_kwargs(kwargs, IndyDCPClient.__init__, Repeater.__init__)
        IndyDCPClient.__init__(self, *args, server_ip=server_ip, **kwargs_indy)
        Repeater.__init__(self, repeater_ip=self.server_ip, disable_getq=True, **kwargs_otic)
        self.indy_grasp_DO = 0

    def connect_and(self, func, *args, **kwargs):
        with self:
            return func(*args, **kwargs)

    @connect_indy
    def wait_motion(self, period=1e-1):
        while not self.get_robot_status()['movedone']:
            time.sleep(period)

    @connect_indy
    def joint_move_make_sure(self, Q, N_repeat=1):
        for _ in range(N_repeat):
            self.joint_move_to(Q)
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
        gstate = self.get_do()[self.indy_grasp_DO]
        if gstate != grasp:
            self.set_do(self.indy_grasp_DO, int(grasp))
            if grasp:
                uvw_cur = self.get_task_pos()[3:]
                Rbe_cur = Rot_zyx(*np.deg2rad(uvw_cur)[[2, 1, 0]])
                time.sleep(0.1)
                self.task_move_by(np.matmul(Rbe_cur, [0, 0, depth]).tolist() + [0] * 3)
                time.sleep(0.1)
                self.wait_motion()
                self.task_move_by(np.matmul(Rbe_cur, [0, 0, -depth]).tolist() + [0] * 3)
                time.sleep(0.1)
                self.wait_motion()

