import matplotlib.pyplot as plt
import time
import numpy as np
from .utils import *
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

    def init_online_tracking(self, q0, connect=True):
        if connect:
            self.connect()
        self.move_ext_traj_txt(traj_type=1, traj_freq=INDY_CONTROL_FREQ, dat_size=INDY_DOF,
                               traj_data=q0.tolist() + [0] * 2 * INDY_DOF)
        ret = self.reset(q0, update_params=True)
        if connect:
            self.disconnect()
        return ret

    def stop_online_tracking(self, connect=True):
        self.stop_tracking()
        self.terminate_loop()
        if connect:
            self.connect()
        self.stop_motion()
        if connect:
            self.disconnect()