# -*- coding: utf-8 -*- 

from nrmkindy import IndyGeneral_pb2 as genr
from nrmkindy import IndyRemoteServices_pb2_grpc as remote_rpc

from indy_abstract_ctrl import *

__metaclass__ = type

class IndyRemoteController(AbstractIndyController):
    def __init__(self, model_name):
        super(IndyRemoteController, self).__init__(model_name)

    def connect(self, ip_addr):
        try:
            self._grpc_channel = grpc.insecure_channel(ip_addr + ':51913')
            self._stub = remote_rpc.RemoteControlDirectorStub(self._grpc_channel)
            _connected = True
            return True
        except Exception as e:
            print(e)
            return False

    # stop command
    def mstop(self, cat, time=0.0):
        if not isinstance(cat, int):
            raise TypeError("stop category must be int type")
        if cat is 0 or cat is 1 or cat is 2:
            raise ValueError("stop category must have 0, 1 or 2 value")
        if not isinstance(cat, time) or (time<0.0 or time>10.0):
            raise ValueError("stop time is 0 to 10")
        motion_stop = motion.MotionStop(stopCat=cat, time=time)
        self._stub.stop(motion_stop)

    # set simulation mode
    def set_sim_mode(self, mode):
        req = genr.BooleanMessage()
        req.value = mode
        self._stub.setSimulationMode(req)

    # get simulation mode
    def get_sim_mode(self):
        ret = self._stub.getSimulationMode(genr.EmptyMessage())
        return ret

    # set direct teaching mode
    def change_direct_teaching_mode(self, target):
        req = genr.BooleanMessage()
        req.value = target
        self._stub.changeDirectTeachingMode(req)

    # move home position
    def move_home(self):
        ret = self._stub.moveHome(genr.EmptyMessage())
        return ret.value

    # move zero position
    def move_zero(self):
        ret = self._stub.moveZero(genr.EmptyMessage())
        return ret.value

    def get_global_property(self):
        ret = self._stub.getGlobalProperty(genr.EmptyMessage())
        return ret

    def set_global_property(self, **kwargs):
        glob_prop = self._create_properties(**kwargs)
        self._stub.setGlobalProperty(glob_prop)

    def set_global_move_base_mode(self, type):
        if isinstance(type, MoveBaseMode):
            req = type
        elif isinstance(type, int):
            req = MoveBaseMode(type)
        else:
            raise TypeError("The type is not MoveBaseModeType or int")

        glob_mode = prop.MotionProperty(motionMode=req.value)
        self._stub.setGlobalMoveBaseMode(glob_mode)

    def set_global_intpr(self, type):
        if isinstance(type, InterpolatorType):
            req = type
        elif isinstance(type, int):
            req = InterpolatorType(type)
        else:
            raise TypeError("The type is not InterpolatorType or int")

        glob_intpr = prop.MotionProperty(intrprType=req.value)
        self._stub.setGlobalInterpolatorType(glob_intpr)

    # FIXME args is None 또는 kwargs is None 은 == () 과 == {} 로 전부 바꿔야 함
    def set_global_joint_vel(self, *args, **kwargs):
        # if args is None and kwargs is None:
        #     raise ValueError("The parameter is empty")
        if len(args) != 0:
            if len(args) == 1:
                if isinstance(args[0], JointMotionVel) or \
                        isinstance(args[0], list) or isinstance(args[0], tuple):
                    kwargs['jv'] = args[0]
                else:
                    raise ValueError("The parameter is only 1 list or tuple or JointMotionVel!")
            elif len(args) == 2:
                kwargs['jv'] = (args[0], args[1])
        elif len(kwargs) != 0:
            jvel = dict()
            jvel['jv'] = (kwargs['vel'], kwargs['acc'])
            kwargs = jvel
        glob_jv = self._create_properties(**kwargs)
        self._stub.setGlobalJointMotionVelocity(glob_jv)

    def set_global_joint_time(self, time):
        glob_jt = prop.MotionProperty(jointMotionTime=time)
        self._stub.setGlobalJointMotionTime(glob_jt)

    def set_global_task_vel(self, *args, **kwargs):
        if len(args) != 0:
            if len(args) == 1:
                if isinstance(args[0], TaskMotionVel) or \
                        isinstance(args[0], list) or isinstance(args[0], tuple):
                    kwargs['tv'] = args[0]
                else:
                    raise ValueError("The parameter is only 1 list or tuple or JointMotionVel!")
            elif len(args) == 4:
                kwargs['tv'] = (args[0], args[1], args[2], args[3])
            else:
                raise ValueError("The parameter need to input dispVel, dispAcc, rotVel, rotAcc")
        elif len(kwargs) != 0:
            tvel = dict()
            tvel['tv'] = (kwargs['dispVel'], kwargs['dispAcc'], kwargs['rotVel'], kwargs['rotAcc'])
            kwargs = tvel

        glob_tv = self._create_properties(**kwargs)
        self._stub.setGlobalTaskMotionVelocity(glob_tv)

    def set_global_task_time(self, time):
        glob_tt = prop.MotionProperty(taskMotionTime=time)
        self._stub.setGlobalTaskMotionTime(glob_tt)

    def set_global_blending_type(self, type):
        if isinstance(type, BlendingType):
            req = type
        elif isinstance(type, int):
            req = BlendingType(type)
        else:
            raise TypeError("The type is not the correct type")

        glob_bt = prop.MotionProperty(blendingType=req.value)
        self._stub.setGlobalBlendingType(glob_bt)

    def set_global_blending_rad(self, rad):
        glob_rad = prop.MotionProperty(blendingRadius=rad)
        self._stub.setGlobalBlendingRadius(glob_rad)

    def set_global_ref_coord_type(self, type):
        if isinstance(type, ReferenceCoordinateType):
            req = type
        elif isinstance(type, int):
            req = ReferenceCoordinateType(type)
        else:
            raise TypeError("The type is not the correct type")

        glob_coord = prop.MotionProperty(refCoordType=req.value)
        self._stub.setGlobalReferenceCoordinateType(glob_coord)

    def set_global_ref_frame(self, **kwargs):
        if isinstance(kwargs['type'], ReferenceFrameType):
            ref_type = kwargs['type']
        elif isinstance(kwargs['type'], int):
            ref_type = ReferenceFrameType(kwargs['type'])
        else:
            raise TypeError("The type is not InterpolatorType or int")

        if kwargs['tref'] is None:
            kwargs['tref'] = (0, 0, 0, 0, 0, 0)
        elif kwargs['points'] is None:
            kwargs['points'] = (0, 0, 0, 0, 0, 0, 0, 0, 0)

        kwargs['tref'] = list(kwargs['tref'])
        for i in range(3, 6):
            if self.get_use_radians():
                continue
            else:
                kwargs['tref'][i] = indy_util.to_radian(kwargs['tref'][i])

        glob_ref_frame = prop.MotionProperty(refFrame=prop.ReferenceFrame(refFrameType=ref_type.value,
                                                                          refFrameTRef=kwargs['tref'],
                                                                          refFramePoints=kwargs['points']))
        self._stub.setGlobalReferenceFrame(glob_ref_frame)

    def set_global_tcp(self, tcp):
        tcp = list(tcp)
        for i in range(3, 6):
            if self._use_radians:
                continue
            else:
                tcp[i] = indy_util.to_radian(tcp[i])

        if not isinstance(tcp, list) and not isinstance(tcp, tuple) and not isinstance(tcp, ToolCenterPoint):
            raise TypeError("The ToolCenterPoint is not correct type")
        elif isinstance(tcp, ToolCenterPoint):
            tcp = tcp.get()

        glob_tcp = prop.MotionProperty(tcp=prop.ToolCenterPoint(tcp=tcp,
                                                                useGlobal=False))
        self._stub.setGlobalToolCenterPoint(glob_tcp)
