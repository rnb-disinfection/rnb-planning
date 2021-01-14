# -*- coding: utf-8 -*- 

from nrmkindy import indy_model
from nrmkindy import indy_util
import grpc

from nrmkindy import IndyMotion_pb2 as motion

from indy_props import *

__metaclass__ = type

class AbstractController:
    def __init__(self):
        self._use_radians = False
        self._connected = False
        self._grpc_channel = None
        self._stub = None

    def connect(self, ip_addr):
        raise AssertionError("connect() must be implemented")

    def disconnect(self):
        if self.connected() is True:
            self._grpc_channel.close()
            self._grpc_channel = None
            self._stub = None

    def connected(self):
        return self._connected

    def set_use_radians(self, use_radians):
        self._use_radians = use_radians

    def get_use_radians(self):
        return self._use_radians


class AbstractIndyController(AbstractController):
    def __init__(self, model_name):
        super(AbstractIndyController, self).__init__()
        self._use_radians = False

        if model_name == indy_model.NAME_INDY_7:
            self._model = indy_model.Indy7Model()
        elif model_name == indy_model.NAME_INDY_RP2:
            self._model = indy_model.IndyRP2Model()
        else:
            raise AttributeError("Undefined Indy Model.")

    def _check_key(self, props, **kwargs):
        for key in kwargs.keys():
            # 틀린 key가 있는지 -> 선택 (예외발생 or *무시)
            if key not in props.keys():
                raise AttributeError("The {0} key is not founded in properties".format(key))
        return True

    def _check_enum(self, key, value, **kwargs):
        # 사용자가 enum 형을 입력할 경우 pass, int 형을 입력할 경우 enum 형으로 값 변경
        if key == 'mode':
            if isinstance(value, MoveBaseMode):
                pass
            elif isinstance(value, int):
                kwargs['mode'] = MoveBaseMode(value)
        elif key == 'intpr':
            if isinstance(value, InterpolatorType):
                pass
            elif isinstance(value, int):
                kwargs['intpr'] = InterpolatorType(value)
        elif key == 'bt':
            if isinstance(value, BlendingType):
                pass
            elif isinstance(value, int):
                kwargs['bt'] = BlendingType(value)
        elif key == 'coord':
            if isinstance(value, ReferenceCoordinateType):
                pass
            elif isinstance(value, int):
                kwargs['coord'] = ReferenceCoordinateType(value)
        elif key == 'ref':
            if isinstance(kwargs[key]['type'], ReferenceFrameType):
                pass
            elif isinstance(kwargs[key]['type'], int):
                kwargs['ref']['type'] = ReferenceFrameType(kwargs[key]['type'])

        return kwargs

    def _check_range(self, key, value, **kwargs):
        if key == 'jv':
            if not isinstance(value, list) and not isinstance(value, tuple) and not isinstance(value, JointMotionVel):
                raise TypeError("The JointVel is not correct type")
            elif isinstance(value, JointMotionVel):
                value = value.get()

            # Array/Tuple의 경우 갯수 확인
            if len(value) != 2:
                raise ValueError("The length of jv is different '2'")
            # Range값인 경우, 최대값/최소값이 존재함, 범위 벗어났으면 예외발생
            if value[0] > self._model.MAX_JOINT_VEL_RAD or value[0] <= 0:
                raise ValueError("The JointVel is not included within range")
            elif value[1] > self._model.MAX_JOINT_ACC_RAD or value[1] <= 0:
                raise ValueError("The JointAcc is not included within range")
        elif key == 'jt':
            if value > self._model.MAX_MOVE_TIME or value < self._model.MIN_MOVE_TIME:
                raise ValueError("The JointMotionTime is not included within range")
        elif key == 'tv':
            if not isinstance(value, list) and not isinstance(value, tuple) and not isinstance(value, TaskMotionVel):
                raise TypeError("The TaskMotionVel is not correct type")
            elif isinstance(value, TaskMotionVel):
                value = value.get()

            if len(value) != 4:
                raise ValueError("The length of tv is different '4'")
            if value[0] > self._model.MAX_DISP_VEL_METER or value[0] <= 0:
                raise ValueError("The TaskDispVel is not included within range")
            elif value[1] > self._model.MAX_DISP_ACC_METER or value[1] <= 0:
                raise ValueError("The TaskDispAcc is not included within range")
            elif value[2] > self._model.MAX_ROT_VEL_RAD or value[2] <= 0:
                raise ValueError("The TaskRotVel is not included within range")
            elif value[3] > self._model.MAX_ROT_ACC_RAD or value[3] <= 0:
                raise ValueError("The TaskRotAcc is not included within range")
        elif key == 'tt':
            if value > self._model.MAX_MOVE_TIME or value < self._model.MIN_MOVE_TIME:
                raise ValueError("The TaskMotionTime is not included within range")
        elif key == 'r':
            pass
        elif key == 'ref':
            if not isinstance(value, dict) and not isinstance(value, ReferenceFrame):
                raise TypeError("The ReferenceFrame is not correct type")
            elif isinstance(value, ReferenceFrame):
                value = value.get()
                kwargs[key] = value

            # type도 있으면서 다른 값도 있어야 함
            if len(value) < 2 and 'type' in value.keys():
                raise ValueError("The length of ref is less than 2")

            if value['type'] is ReferenceFrameType.DIRECT:
                if not isinstance(value['tref'], list) and not isinstance(value['tref'], tuple):
                    raise TypeError("The ReferenceFrameTref is not correct type")
                else:
                    value['points'] = (0, 0, 0, 0, 0, 0, 0, 0, 0)
            elif value['type'] in [ReferenceFrameType.LINEAR, ReferenceFrameType.CIRCULAR]:
                if not isinstance(value['points'], list) and not isinstance(value['points'], tuple):
                    raise TypeError("The ReferenceFramePoints is not correct type")
                else:
                    value['tref'] = (0, 0, 0, 0, 0, 0)

        elif key == 'tcp':
            if not isinstance(value, list) and not isinstance(value, tuple) and not isinstance(value, ToolCenterPoint):
                raise TypeError("The ToolCenterPoint is not correct type")
            elif isinstance(value, ToolCenterPoint):
                value = value.get()
                kwargs[key] = value

        return kwargs

    def _create_properties(self, **kwargs):
        # 1. 없는 key의 경우 -> USE_GLOBAL 사용하도록 property 생성
        motion_property = {
            'mode': MoveBaseMode._USE_GLOBAL,
            'intpr': InterpolatorType._USE_GLOBAL,
            'jv': (-1, -1),
            'jt': -1,
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

        # 2. 틀린 key가 있는지 -> 선택 (예외발생 or *무시)
        self._check_key(motion_property, **kwargs)

        # degree -> radian 직접 코드
        for key, value in kwargs.items():
            if key == 'jv':
                if isinstance(value, JointMotionVel):
                    value = value.get()
                    kwargs[key] = value

                if self._use_radians:
                    continue
                else:
                    kwargs[key] = indy_util.to_radian(value)
            elif key == 'tv':
                if isinstance(value, TaskMotionVel):
                    value = value.get()
                    kwargs[key] = value
                kwargs[key] = list(kwargs[key])
                for i in range(2, 4):
                    if value[i] == -1:
                        pass
                    elif self._use_radians:
                        continue
                    else:
                        kwargs[key][i] = indy_util.to_radian(value[i])
            elif key is 'ref':
                if isinstance(value, ReferenceFrame):
                    value = value.get()
                    kwargs[key] = value
                # u,v,w convert to radian
                for k in value.keys():
                    if k == 'tref' and kwargs[key]['type'] == ReferenceFrameType.DIRECT:
                        kwargs[key][k] = list(kwargs[key][k])
                        for i in range(3, 6):
                            if self._use_radians:
                                continue
                            else:
                                kwargs[key][k][i] = indy_util.to_radian(value[k][i])
            elif key is 'tcp':
                if isinstance(value, ToolCenterPoint):
                    value = value.get()
                    kwargs[key] = value
                if value is not None:
                    kwargs[key] = list(kwargs[key])
                    for i in range(3, 6):
                        if self._use_radians:
                            continue
                        else:
                            kwargs[key][i] = indy_util.to_radian(value[i])

        # 3. 값이 범위 내에 있는지
        #   3-1. Enum값인 경우. 숫자로 들어왔을 경우, 해당 값이 없으면 예외발생
        #   3-2. Range값인 경우, 최대값/최소값이 존재함, 범위 벗어났으면 예외발생
        #        Array/Tuple의 경우 갯수 확인
        for key, value in kwargs.items():
            if key in ['mode', 'intpr', 'bt', 'coord', 'ref']:
                kwargs.update(self._check_enum(key, value, **kwargs))
                if key is 'ref':
                    kwargs.update(self._check_range(key, value, **kwargs))
            else:
                kwargs.update(self._check_range(key, value, **kwargs))

        motion_property.update(kwargs)

        jv = motion.JointMotionVelocity()
        jv.vel = motion_property['jv'][0]
        jv.acc = motion_property['jv'][1]
        tv = motion.TaskMotionVelocity()
        tv.dispVel = motion_property['tv'][0]
        tv.dispAcc = motion_property['tv'][1]
        tv.rotVel = motion_property['tv'][2]
        tv.rotAcc = motion_property['tv'][3]

        motion_prop = motion.MotionProperty(motionMode=motion_property['mode'].value,
                                            intrprType=motion_property['intpr'].value,
                                            jointMotionVelocity=jv,
                                            jointMotionTime=motion_property['jt'],
                                            taskMotionVelocity=tv,
                                            taskMotionTime=motion_property['tt'],
                                            blendingType=motion_property['bt'].value,
                                            blendingRadius=motion_property['r'],
                                            refCoordType=motion_property['coord'].value,
                                            refFrame=motion.ReferenceFrame(
                                                refFrameType=motion_property['ref']['type'].value,
                                                refFrameTRef=motion_property['ref']['tref'],
                                                refFramePoints=motion_property['ref']['points']),
                                            tcp=motion.ToolCenterPoint(tcp=motion_property['tcp'], useGlobal=False) if
                                            motion_property['tcp'] is not None
                                            else motion.ToolCenterPoint(useGlobal=True)
                                            )

        return motion_prop
