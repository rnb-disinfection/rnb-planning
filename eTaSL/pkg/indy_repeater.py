import matplotlib.pyplot as plt
import time
import numpy as np
from pkg.utils import *
from indy_utils import indydcp_client as client
from functools import wraps

INDY_DOF = 6
PORT_REPEATER = 1189
INDY_CONTROL_FREQ = 4000
DEFAULT_TRAJ_RATE_INDY = 50


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


@connect_indy
def indy_wait_motion(indy, period=1e-1):
    while not indy.get_robot_status()['movedone']:
        time.sleep(period)


@connect_indy
def indy_joint_move_make_sure(indy, Q, N_makesure=5):
    for _ in range(N_makesure):
        indy.joint_move_to(Q)
        indy_wait_motion(indy)

def indy_init_online_tracking(indy, q0, traj_freq=DEFAULT_TRAJ_RATE_INDY, connect=True):
    if connect:
        indy.connect()
    indy.move_ext_traj_txt(traj_type=1, traj_freq=INDY_CONTROL_FREQ, dat_size=INDY_DOF,
                           traj_data=q0.tolist() + [0] * 2 * INDY_DOF)
    ret = send_recv({'reset': 1, 'period_s': 1.0 / traj_freq, 'qcur': q0},
              indy.server_ip, PORT_REPEATER)
    if connect:
        indy.disconnect()
    return ret


def indy_send_track_q(indy, Q):
    return send_recv({'qval': Q}, indy.server_ip, PORT_REPEATER)


def indy_get_qcount(indy):
    return send_recv({'qcount': 1}, indy.server_ip, PORT_REPEATER)['qcount']


def indy_stop_online_tracking(indy, connect=True):
    send_recv({'stop': 1}, indy.server_ip, PORT_REPEATER)
    send_recv({'terminate': 1}, indy.server_ip, PORT_REPEATER)
    if connect:
        indy.connect()
    indy.stop_motion()
    if connect:
        indy.disconnect()