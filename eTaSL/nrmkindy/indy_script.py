# -*- coding: utf-8 -*- 

# script interfaces (fuction) using indy controller class

from nrmkindy import indy_ctrl
# TODO below 3 controller will be changed using shared memory
from nrmkindy import indy_remote_ctrl
from nrmkindy import indy_robot_io
from nrmkindy import indy_data_retriever
# TODO ------------------
from indy_data import *
from indy_data_io import *
from indy_props import *
from indy_model import *

from nrmkindy import indy_util

_ctrl = None
_io = None
_remote = None
_retr = None


def config_script(model_name):
    global _ctrl, _io, _remote, _retr
    _ctrl = indy_ctrl.IndyController(model_name)
    _io = indy_robot_io.IOController()
    _remote = indy_remote_ctrl.IndyRemoteController(model_name)
    _retr = indy_data_retriever.RobotDataRetriever(model_name)


def start_script(host='localhost'):
    global _ctrl, _io, _remote, _retr
    ret = _ctrl.connect(host) and \
          _remote.connect(host) and \
          _retr.connect(host) and \
          _io.connect(host)
    if ret is False:
        end_script()
        raise AssertionError("Cannot connected with robot.")


def end_script():
    global _ctrl, _io, _remote, _retr
    _ctrl.disconnect()
    _io.disconnect()
    _remote.disconnect()
    _retr.disconnect()


# TODO make set sim mode
# def simulation():
#     return _remote.get_sim_mode()

def simulation(on=None):
    if on is None:
        return _remote.get_sim_mode()
    else:
        _remote.set_sim_mode(on)
        _retr.set_sim_mode(on)


def use_radians():
    _ctrl.set_use_radians(True)
    _remote.set_use_radians(True)
    _retr.set_use_radians(True)
    _io.set_use_radians(True)


def use_degrees():
    _ctrl.set_use_radians(False)
    _remote.set_use_radians(False)
    _retr.set_use_radians(False)
    _io.set_use_radians(False)


def movej(q=None, **kwargs):
    if q is None and len(kwargs) == 0:
        raise ValueError("The parameter is empty!")
    return _ctrl.movej(q, **kwargs)


def amovej(q=None, **kwargs):
    if q is None and len(kwargs) == 0:
        raise ValueError("The parameter is empty!")
    return _ctrl.amovej(q, **kwargs)


def rmovej(q=None, **kwargs):
    if q is None and len(kwargs) == 0:
        raise ValueError("The parameter is empty!")
    return _ctrl.rmovej(q, **kwargs)


def armovej(q=None, **kwargs):
    if q is None and len(kwargs) == 0:
        raise ValueError("The parameter is empty!")
    return _ctrl.armovej(q, **kwargs)


def movel(p=None, **kwargs):
    if p is None and len(kwargs) == 0:
        raise ValueError("The parameter is empty!")
    return _ctrl.movel(p, **kwargs)


def amovel(p=None, **kwargs):
    if p is None and len(kwargs) == 0:
        raise ValueError("The parameter is empty!")
    return _ctrl.amovel(p, **kwargs)


def rmovel(p=None, **kwargs):
    if p is None and len(kwargs) == 0:
        raise ValueError("The parameter is empty!")
    return _ctrl.rmovel(p, **kwargs)


def armovel(p=None, **kwargs):
    if p is None and len(kwargs) == 0:
        raise ValueError("The parameter is empty!")
    return _ctrl.armovel(p, **kwargs)


def mwait(**kwargs):
    if len(kwargs) == 0:
        raise ValueError("The parameter is empty!")
    return _ctrl.mwait(**kwargs)


def mwait_done():
    return _ctrl.mwait_done()


def mstop(cat=None, time=0.0):
    if cat is None and time == 0.0:
        raise ValueError("The parameter is empty!")
    elif cat == 1 or cat == 2:
        if time == 0.0:
            raise ValueError("If stop category 1 or 2, need to input time parameter!")
    return _ctrl.mstop(cat, time)


def home(**kwargs):
    return _ctrl.home(**kwargs)


def ahome(**kwargs):
    return _ctrl.ahome(**kwargs)


def zero(**kwargs):
    return _ctrl.zero(**kwargs)


def props(**kwargs):
    # if parameter is empty, return props (same as all props)
    if len(kwargs) == 0:
        get_prop = (base_mode(), intpr_type(), joint_vel(), joint_time(),
                    task_vel(), task_time(), blending_type(), blending_r(),
                    coord_type(), refframe(), tcp())
        return get_prop
    else:
        return _remote.set_global_property(**kwargs)


def base_mode(type=None):
    if type is None:
        get_prop = _remote.get_global_property()
        return MoveBaseMode(get_prop.motionMode)
    return _remote.set_global_move_base_mode(type)


def intpr_type(type=None):
    if type is None:
        get_prop = _remote.get_global_property()
        return InterpolatorType(get_prop.intrprType)
    return _remote.set_global_intpr(type)


def joint_vel(*args, **kwargs):
    if len(args) == 0 and len(kwargs) == 0:
        get_prop = _remote.get_global_property()
        ref = get_prop.jointMotionVelocity
        if _retr.get_use_radians() is True:
            pass
        else:
            ref.vel = round(indy_util.to_degree(ref.vel), 3)
            ref.acc = round(indy_util.to_degree(ref.acc), 3)
        return ref.vel, ref.acc
    return _remote.set_global_joint_vel(*args, **kwargs)


def joint_time(time=None):
    if time is None:
        get_prop = _remote.get_global_property()
        return get_prop.jointMotionTime
    return _remote.set_global_joint_time(time)


def task_vel(*args, **kwargs):
    if len(args) == 0 and len(kwargs) == 0:
        get_prop = _remote.get_global_property()
        ref = get_prop.taskMotionVelocity
        if _retr.get_use_radians() is True:
            pass
        else:
            ref.rotVel = round(indy_util.to_degree(ref.rotVel), 3)
            ref.rotAcc = round(indy_util.to_degree(ref.rotAcc), 3)
        return ref.dispVel, ref.dispAcc, ref.rotVel, ref.rotAcc
    return _remote.set_global_task_vel(*args, **kwargs)


def task_time(time=None):
    if time is None:
        get_prop = _remote.get_global_property()
        return get_prop.taskMotionTime
    return _remote.set_global_task_time(time)


def blending_type(type=None):
    if type is None:
        get_prop = _remote.get_global_property()
        return BlendingType(get_prop.blendingType)
    return _remote.set_global_blending_type(type)


def blending_r(rad=None):
    if rad is None:
        get_prop = _remote.get_global_property()
        return get_prop.blendingRadius
    return _remote.set_global_blending_rad(rad)


def coord_type(type=None):
    if type is None:
        get_prop = _remote.get_global_property()
        return ReferenceCoordinateType(get_prop.refCoordType)
    return _remote.set_global_ref_coord_type(type)


def refframe(*args):
    # if parameter is refframe, return tcp
    if len(args) == 0:
        # raise ValueError("The parameter is empty!")
        get_prop = _remote.get_global_property()
        ref = get_prop.refFrame
        type = ReferenceFrameType(ref.refFrameType)
        tref = ref.refFrameTRef
        if _retr.get_use_radians() is True:
            pass
        else:
            for i in range(3, 6):
                tref[i] = round(indy_util.to_degree(tref[i]), 3)
        points = ref.refFramePoints
        return type, tuple(tref), tuple(points)

    kwargs = dict()
    type = None
    tref = None
    points = None
    for i in range(len(args)):
        if args[i] == ReferenceFrameType.DIRECT or \
                args[i] == ReferenceFrameType.LINEAR or \
                args[i] == ReferenceFrameType.CIRCULAR:
            type = args[i]
        elif len(args[i]) == 6:
            tref = args[i]
        elif len(args[i]) == 9:
            points = args[i]
    # type 은 무조건 입력해야 함
    if type is None:
        raise ValueError("ReferenceFrameType is empty")
    kwargs['type'] = type
    kwargs['tref'] = tref
    kwargs['points'] = points

    return _remote.set_global_ref_frame(**kwargs)


def tcp(*args):
    arg = None
    # if parameter is empty, return tcp
    if len(args) == 0:
        # raise ValueError("The parameter is empty!")
        get_prop = _remote.get_global_property()
        ref = get_prop.tcp
        return tuple(ref.tcp)
    # tuple or list 변수로 tcp 입력했을 경우
    if len(args) == 1:
        if len(args[0]) != 6:
            raise ValueError("TCP length must be 6")
        arg = args[0]
    else:
        if len(args) != 6:
            raise ValueError("TCP length must be 6")
        arg = args

    return _remote.set_global_tcp(arg)


def q():
    if _retr.get_use_radians() is True:
        return _retr.get_joint_pos()
    else:
        cur_pos = list(_retr.get_joint_pos())
        cur_pos = indy_util.to_degree(cur_pos)
        for i in range(6):
            cur_pos[i] = round(cur_pos[i], 3)
        return tuple(cur_pos)


def qdot():
    if _retr.get_use_radians() is True:
        return _retr.get_joint_vel()
    else:
        cur = list(_retr.get_joint_vel())
        cur = indy_util.to_degree(cur)
        for i in range(6):
            cur[i] = round(cur[i], 3)
        return tuple(cur)


def p():
    if _retr.get_use_radians() is True:
        return _retr.get_task_pos()
    else:
        cur = list(_retr.get_task_pos())
        for i in range(3):
            cur[i] = round(cur[i], 3)
        for i in range(3, 6):
            cur[i] = indy_util.to_degree(cur[i])
            cur[i] = round(cur[i], 3)
        return tuple(cur)


def pdot():
    if _retr.get_use_radians() is True:
        return _retr.get_task_vel()
    else:
        cur = list(_retr.get_task_vel())
        for i in range(3):
            cur[i] = round(cur[i], 3)
        for i in range(3, 6):
            cur[i] = indy_util.to_degree(cur[i])
            cur[i] = round(cur[i], 3)
        return tuple(cur)


def pb():
    if _retr.get_use_radians() is True:
        return _retr.get_task_pos_base()
    else:
        cur = list(_retr.get_task_pos_base())
        for i in range(3):
            cur[i] = round(cur[i], 3)
        for i in range(3, 6):
            cur[i] = indy_util.to_degree(cur[i])
            cur[i] = round(cur[i], 3)
        return tuple(cur)


def pbdot():
    if _retr.get_use_radians() is True:
        return _retr.get_task_vel_base()
    else:
        cur = list(_retr.get_task_vel_base())
        for i in range(3):
            cur[i] = round(cur[i], 3)
        for i in range(3, 6):
            cur[i] = indy_util.to_degree(cur[i])
            cur[i] = round(cur[i], 3)
        return tuple(cur)


def tau():
    if _retr.get_use_radians() is True:
        cur = list(_retr.get_torque())
        for i in range(6):
            cur[i] = indy_util.to_radian(cur[i])
            cur[i] = round(cur[i], 3)
        return tuple(cur)
    else:
        cur = list(_retr.get_torque())
        for i in range(6):
            cur[i] = round(cur[i], 3)
        return tuple(cur)


def ctrl_state():
    # get_basic_robot_control_state
    get_ctrl_state = _retr.get_basic_robot_control_state()
    for key, value in get_ctrl_state.items():
        if _retr.get_use_radians() is True:
            if key == 'torque':
                value = list(value)
                for i in range(6):
                    value[i] = indy_util.to_radian(value[i])
                    value[i] = round(value[i], 3)
            else:
                value = list(value)
                for i in range(6):
                    value[i] = round(value[i], 3)
        else:
            if key == 'jointPos' or key == 'jointVel':
                value = list(value)
                for i in range(6):
                    value[i] = indy_util.to_degree(value[i])
                    value[i] = round(value[i], 3)
            elif key == 'taskPos' or key == 'taskVel' or key == 'taskPosBase' or key == 'taskVelBase':
                value = list(value)
                for i in range(3):
                    value[i] = round(value[i], 3)
                for i in range(3, 6):
                    value[i] = indy_util.to_degree(value[i])
                    value[i] = round(value[i], 3)
            else:
                value = list(value)
                for i in range(6):
                    value[i] = round(value[i], 3)
        get_ctrl_state[key] = tuple(value)
        # print(key, ':', tuple(value))
    return get_ctrl_state



def ctrl_state_all():
    # not implemented yet
    # get_robot_control_state
    pass


def status():
    get_ctrl_status = _retr.get_robot_status()
    # for key, value in get_ctrl_status.items():
    #     print(key, ':', value)
    return get_ctrl_status


def di():
    return _io.get_di()


def do(*args):
    if len(args) == 0:
        return _io.get_do()
    else:
        return _io.set_do(*args)


def ai():
    return _io.get_ai()


def ao(*args):
    if len(args) == 0:
        return _io.get_ao()
    else:
        return _io.set_ao(*args)


def si():
    return _io.get_safe_di()


def so(*args):
    if len(args) == 0:
        return _io.get_safe_do()
    else:
        return _io.get_safe_do(*args)


def ei():
    return _io.get_endtool_di()


def eo(*args):
    if len(args) == 0:
        return _io.get_endtool_do()
    else:
        return _io.set_endtool_do(*args)
