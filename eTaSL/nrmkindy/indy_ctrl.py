# -*- coding: utf-8 -*- 

from nrmkindy import IndyGeneral_pb2 as genr
from nrmkindy import IndyLocalServices_pb2_grpc as local_rpc

from indy_data import *
from indy_data_io import *
from indy_robot_io import *
from indy_abstract_ctrl import *

__metaclass__ = type
class IndyController(AbstractIndyController):
    def __init__(self, model_name):
        super(IndyController, self).__init__(model_name)
        self._move_id_cnt = int(0)

    def connect(self, ip_addr):
        try:
            self._grpc_channel = grpc.insecure_channel(ip_addr + ':51911')
            self._stub = local_rpc.ControlDirectorStub(self._grpc_channel)
            _connected = True
            return True
        except Exception as e:
            print(e)
            return False

    def movej(self, q, **kwargs):
        # q가 tuple or list 체크
        if isinstance(q, tuple) or isinstance(q, list):
            pass
        # q가 JointPos 일 경우. JointPos(1, 2, 3, 4, 5, 6) 객체 형태를 tuple 로 변환
        elif isinstance(q, JointPos):
            q = q.get()
        else:
            raise TypeError("The q is not the correct type")

        if len(q) != self._model.JOINT_DOF:
            raise ValueError("The length of q is different with JOINT_DOF")

        # Check Joint Limits
        for i in range(len(q)):
            if self._use_radians is False:
                if i in range(3):
                    if q[i] > self._model.JOINT_LIMITS_FORWARD_DEG[i] or \
                       q[i] < self._model.JOINT_LIMITS_BACKWARD_DEG[i]:
                        raise ValueError("The joint limit error!")
            else:
                if i in range(3):
                    if q[i] > self._model.JOINT_LIMITS_FORWARD_RAD[i] or \
                       q[i] < self._model.JOINT_LIMITS_BACKWARD_RAD[i]:
                        raise ValueError("The joint limit error!")

        self._move_id_cnt += 1
        prop = self._create_properties(**kwargs)
        pos_q = motion.JointPos(q=q if self._use_radians else indy_util.to_radian(q))
        basic = motion.MotionBasic(pos1=motion.Position(type=motion.PositionType.INDY_POS_TYPE_JOINT,
                                                        jointPos=pos_q),
                                   bAsyncMode=kwargs['async'] if 'async' in kwargs.keys() and isinstance(kwargs['async'], bool) else False,
                                   property=prop
                                   )
        motion_movej = motion.Motion(info=motion.MotionInfo(id=self._move_id_cnt,
                                                            type=motion.MotionType.INDY_MOTION_TYPE_BASIC_MOVE_J),
                                     basic=basic
                                     )

        ret = self._stub.enqueueMotionAcceptable(motion_movej)
        return ret.code == motion.MotionResultCode.MOTION_RESULT_OK

    def amovej(self, q, **kwargs):
        kwargs['async'] = True
        return self.movej(q, **kwargs)

    def rmovej(self, q, **kwargs):
        kwargs['mode'] = MoveBaseMode.RELATIVE
        return self.movej(q, **kwargs)

    def armovej(self, q, **kwargs):
        kwargs['async'] = True
        kwargs['mode'] = MoveBaseMode.RELATIVE
        return self.movej(q, **kwargs)

    def movel(self, p, **kwargs):
        # p가 tuple or list 체크 추가
        if isinstance(p, tuple) or isinstance(p, list):
            pass
        # p가 TaskPos일 경우. TaskPos(1, 2, 3, 4, 5, 6) 객체 형태를 tuple
        elif isinstance(p, TaskPos):
            p = p.get()
        else:
            raise TypeError("The p is not the correct type")

        if len(p) != self._model.JOINT_DOF:
            raise ValueError("The length of p is different with JOINT_DOF")

        self._move_id_cnt += 1
        prop = self._create_properties(**kwargs)

        # u,v,w convert to radian
        for i in range(3, 6):
            p = list(p)
            if self._use_radians:
                continue
            else:
                p[i] = indy_util.to_radian(p[i])
        pos_p = motion.TaskPos(p=p)
        basic = motion.MotionBasic(pos1=motion.Position(type=motion.PositionType.INDY_POS_TYPE_TASK,
                                                        taskPos=pos_p),
                                   bAsyncMode=kwargs["async"] if "async" in kwargs.keys() and isinstance(kwargs["async"], bool) else False,
                                   property=prop)
        motion_movel = motion.Motion(info=motion.MotionInfo(id=self._move_id_cnt,
                                                            type=motion.MotionType.INDY_MOTION_TYPE_BASIC_MOVE_L),
                                     basic=basic)
        ret = self._stub.enqueueMotionAcceptable(motion_movel)
        return ret.code == motion.MotionResultCode.MOTION_RESULT_OK

    def amovel(self, p, **kwargs):
        kwargs["async"] = True;
        return self.movel(p, **kwargs)

    def rmovel(self, p, **kwargs):
        kwargs['mode'] = MoveBaseMode.RELATIVE
        return self.movel(p, **kwargs)

    def armovel(self, p, **kwargs):
        kwargs['async'] = True
        kwargs['mode'] = MoveBaseMode.RELATIVE
        return self.movel(p, **kwargs)

    # def mwait(self, s=None, p=None, t=None, di=None, ai=None, ei=None):
    def mwait(self, **kwargs):
        if kwargs is None:
            raise AttributeError("There is no mwait parameter")

        io_ctrl = IOController()
        wait = motion.MotionWait()
        if "state" in kwargs or "s" in kwargs:
            state_event = kwargs.get("state") or kwargs.get("s")
            if isinstance(state_event, TrajStateEvent):
                pass
            elif isinstance(state_event, int):
                try:
                    state_event = TrajStateEvent(state_event)
                except:
                    raise ValueError("({}) is unknown state".format(state_event))
            else:
                raise TypeError("State Event for mwait() parameter must be TrajStateEvent Enum or int")

            wait = motion.MotionWait(type=motion.MotionWaitType.INDY_MOTION_WAIT_TYPE_TRAJ_EVENT,
                                     trajEvent=motion.TrajectoryEvent(type=motion.TrajectoryEventType.INDY_TRAJ_EVENT_TYPE_STATE,
                                                                      targetProfile=0,
                                                                      trajStateEvent=motion.TrajectoryStateEvent(type=state_event.value)))

        elif "progress" in kwargs or "p" in kwargs:
            progress = kwargs.get("progress") or kwargs.get("p")
            if isinstance(progress, float) or isinstance(progress, int):
                if progress < 0 or progress > 1:
                    raise ValueError("Progress value must be 0 to 1.0 (0~100%)")
            else:
                raise TypeError("Progress Event for mwait() parameter must be float or int")

            wait = motion.MotionWait(type=motion.MotionWaitType.INDY_MOTION_WAIT_TYPE_TRAJ_EVENT,
                                     trajEvent=motion.TrajectoryEvent(type=motion.TrajectoryEventType.INDY_TRAJ_EVENT_TYPE_PROCESS,
                                                                      targetProfile=0,
                                                                      motionProgress=progress))

        elif "time" in kwargs or "t" in kwargs:
            time = kwargs.get("time") or kwargs.get("t")
            if time == 0 or time == 0.0:   # ignore time 0
                return True
            elif isinstance(time, float):
                pass
            elif isinstance(time, int):
                time = float(time)
            else:
                raise TypeError("Timing Event for mwait() parameter must be float or int")

            wait = motion.MotionWait(type=motion.MotionWaitType.INDY_MOTION_WAIT_TYPE_WAIT_TIME,
                                     waitingTime=time)

        elif "di" in kwargs:
            dio_set = indy_util.to_io_set(**kwargs)
            di = io_ctrl._create_dio_set(dio_set)
            wait = motion.MotionWait(type=motion.MotionWaitType.INDY_MOTION_WAIT_TYPE_DIO,
                                     dioSet=di)

        elif "ai" in kwargs:
            aio_set = indy_util.to_io_set(**kwargs)
            ai = io_ctrl._create_aio_set(aio_set)
            wait = motion.MotionWait(type=motion.MotionWaitType.INDY_MOTION_WAIT_TYPE_AIO,
                                     aioSet=ai)

        elif "ei" in kwargs:
            dio_set = indy_util.to_io_set(**kwargs)
            ei = io_ctrl._create_dio_set(dio_set)
            wait = motion.MotionWait(type=motion.MotionWaitType.INDY_MOTION_WAIT_TYPE_ENDTOOLIO,
                                     endtoolIoSet=ei)

        else:
            raise AttributeError("There are only unknown mwait parameters")

        self._move_id_cnt += 1
        motion_wait = motion.Motion(info=motion.MotionInfo(id=self._move_id_cnt,
                                                           type=motion.MotionType.INDY_MOTION_TYPE_SP_WAIT),
                                    wait=wait)
        ret = self._stub.enqueueMotionAcceptable(motion_wait)
        return ret.code == motion.MotionResultCode.MOTION_RESULT_OK

    def mwait_done(self):
        ret = self._stub.waitMotionDone(genr.EmptyMessage())
        return ret.value

    def mstop(self, cat, time=0.0):
        if not isinstance(cat, int):
            raise TypeError("stop category must be int type")
        if cat is not 0 and cat is not 1 and cat is not 2:
            raise ValueError("stop category must have 0, 1 or 2 value")
        if (not isinstance(time, float)) or time < 0.0 or time > 10.0:
            raise ValueError("stop time is 0 to 10")
        motion_stop = motion.MotionStop(stopCat=cat, time=time)
        self._stub.stopMotion(motion_stop)

    def home(self, **kwargs):
        default_motion_property = {
            'mode': MoveBaseMode.ABSOLUTE,
            'tv': (-1, -1, -1, -1),
            'tt': -1,
            'coord': ReferenceCoordinateType._USE_GLOBAL,
            'ref': {'type': ReferenceFrameType._USE_GLOBAL,
                    'tref': (0, 0, 0, 0, 0, 0),
                    'points': (0, 0, 0, 0, 0, 0, 0, 0, 0)},
            'tcp': None,
            'async': False
        }

        tv = motion.TaskMotionVelocity()
        tv.dispVel = default_motion_property['tv'][0]
        tv.dispAcc = default_motion_property['tv'][1]
        tv.rotVel = default_motion_property['tv'][2]
        tv.rotAcc = default_motion_property['tv'][3]

        self._move_id_cnt += 1
        motion_prop = self._create_properties(**kwargs)
        default_prop = motion.MotionProperty(motionMode=default_motion_property['mode'].value,
                                             intrprType=motion_prop.intrprType,
                                             jointMotionVelocity=motion_prop.jointMotionVelocity,
                                             jointMotionTime=motion_prop.jointMotionTime,
                                             taskMotionVelocity=tv,
                                             taskMotionTime=default_motion_property['tt'],
                                             blendingType=motion_prop.blendingType,
                                             blendingRadius=motion_prop.blendingRadius,
                                             refCoordType=default_motion_property['coord'].value,
                                             refFrame=motion.ReferenceFrame(
                                                refFrameType=default_motion_property['ref']['type'].value,
                                                refFrameTRef=default_motion_property['ref']['tref'],
                                                refFramePoints=default_motion_property['ref']['points']),
                                             tcp=motion.ToolCenterPoint(tcp=default_motion_property['tcp'], useGlobal=False) if
                                             default_motion_property['tcp'] is not None
                                             else motion.ToolCenterPoint(useGlobal=True)
                                             )
        basic = motion.MotionBasic(bAsyncMode=kwargs['async'] if 'async' in kwargs.keys() and isinstance(kwargs['async'], bool) else False,
                                   property=default_prop
                                   )
        motion_home = motion.Motion(info=motion.MotionInfo(id=self._move_id_cnt,
                                                           type=motion.MotionType.INDY_MOTION_TYPE_BASIC_MOVE_J),
                                    basic=basic
                                    )
        ret = self._stub.enqueueHome(motion_home)
        return ret.code == motion.MotionResultCode.MOTION_RESULT_OK

    def ahome(self, **kwargs):
        kwargs['async'] = True
        return self.home(**kwargs)

    def zero(self, **kwargs):
        default_motion_property = {
            'mode': MoveBaseMode.ABSOLUTE,
            'tv': (-1, -1, -1, -1),
            'tt': -1,
            'bt': BlendingType._USE_GLOBAL,
            'r': -1,
            'coord': ReferenceCoordinateType._USE_GLOBAL,
            'ref': {'type': ReferenceFrameType._USE_GLOBAL,
                    'tref': (0, 0, 0, 0, 0, 0),
                    'points': (0, 0, 0, 0, 0, 0, 0, 0, 0)},
            'tcp': None,
            'async': False
        }

        tv = motion.TaskMotionVelocity()
        tv.dispVel = default_motion_property['tv'][0]
        tv.dispAcc = default_motion_property['tv'][1]
        tv.rotVel = default_motion_property['tv'][2]
        tv.rotAcc = default_motion_property['tv'][3]

        self._move_id_cnt += 1
        motion_prop = self._create_properties(**kwargs)
        default_prop = motion.MotionProperty(motionMode=default_motion_property['mode'].value,
                                             intrprType=motion_prop.intrprType,
                                             jointMotionVelocity=motion_prop.jointMotionVelocity,
                                             jointMotionTime=motion_prop.jointMotionTime,
                                             taskMotionVelocity=tv,
                                             taskMotionTime=default_motion_property['tt'],
                                             blendingType=default_motion_property['bt'].value,
                                             blendingRadius=default_motion_property['r'],
                                             refCoordType=default_motion_property['coord'].value,
                                             refFrame=motion.ReferenceFrame(
                                                 refFrameType=default_motion_property['ref']['type'].value,
                                                 refFrameTRef=default_motion_property['ref']['tref'],
                                                 refFramePoints=default_motion_property['ref']['points']),
                                             tcp=motion.ToolCenterPoint(tcp=default_motion_property['tcp'],
                                                                        useGlobal=False) if
                                             default_motion_property['tcp'] is not None
                                             else motion.ToolCenterPoint(useGlobal=True)
                                             )
        basic = motion.MotionBasic(property=default_prop)
        motion_zero = motion.Motion(info=motion.MotionInfo(id=self._move_id_cnt,
                                                           type=motion.MotionType.INDY_MOTION_TYPE_BASIC_MOVE_J),
                                    basic=basic
                                    )
        ret = self._stub.enqueueZero(motion_zero)
        return ret.code == motion.MotionResultCode.MOTION_RESULT_OK
