# -*- coding: utf-8 -*- 

#TODO implement RobotDataRetriever grpc Client class
from nrmkindy import IndyGeneral_pb2 as genr
from nrmkindy import IndyRemoteServices_pb2_grpc as remote_rpc

from indy_abstract_ctrl import *

__metaclass__ = type

class RobotDataRetriever(AbstractIndyController):
    def __init__(self, model_name):
        super(RobotDataRetriever, self).__init__(model_name)

    def connect(self, ip_addr):
        try:
            self._grpc_channel = grpc.insecure_channel(ip_addr + ':51912')
            self._stub = remote_rpc.RobotDataRetrieverStub(self._grpc_channel)
            _connected = True
            return True
        except Exception as e:
            print(e)
            return False

    # set simulation mode
    def set_sim_mode(self, mode):
        req = genr.BooleanMessage()
        req.value = mode
        self._stub.setSimulationMode(req)

    # get JointPos from SharedMemory
    def get_joint_pos(self):
        ret = self._stub.getJointPos(genr.EmptyMessage())
        return tuple(ret.value)

    # get JointVel from SharedMemory
    def get_joint_vel(self):
        ret = self._stub.getJointVel(genr.EmptyMessage())
        return tuple(ret.value)

    # get TaskPos from SharedMemory
    def get_task_pos(self):
        ret = self._stub.getTaskPos(genr.EmptyMessage())
        return tuple(ret.value)

    # get TaskVel from SharedMemory
    def get_task_vel(self):
        ret = self._stub.getTaskVel(genr.EmptyMessage())
        return tuple(ret.value)

    # get TaskPos from SharedMemory based on Robot Base
    def get_task_pos_base(self):
        ret = self._stub.getTaskPosBase(genr.EmptyMessage())
        return tuple(ret.value)

    # get TaskVel from SharedMemory based on Robot Base
    def get_task_vel_base(self):
        ret = self._stub.getTaskVelBase(genr.EmptyMessage())
        return tuple(ret.value)

    # get Torque from SharedMemory
    def get_torque(self):
        ret = self._stub.getTorque(genr.EmptyMessage())
        return tuple(ret.value)

    # get Basic Robot Control State from SharedMemory
    def get_basic_robot_control_state(self):
        ret = dict()
        basic_ctrl = self._stub.getBasicRobotControlState(genr.EmptyMessage())
        ret['jointPos'] = tuple(basic_ctrl.q)
        ret['jointVel'] = tuple(basic_ctrl.qdot)
        ret['torque'] = tuple(basic_ctrl.tau)
        ret['taskPos'] = tuple(basic_ctrl.p)
        ret['taskVel'] = tuple(basic_ctrl.pdot)
        ret['taskPosBase'] = tuple(basic_ctrl.pBase)
        ret['taskVelBase'] = tuple(basic_ctrl.pdotBase)
        return ret

    # get Real Robot Status from SharedMemory
    def get_robot_status(self):
        ret = dict()
        rb_status = self._stub.getRobotStatus(genr.EmptyMessage())
        ret['isSimulationMode'] = rb_status.isSimulationMode
        ret['isReady'] = rb_status.isReady
        ret['isViolationState'] = rb_status.isViolationState
        ret['isEmergencyState'] = rb_status.isEmergencyState
        ret['isCollided'] = rb_status.isCollided
        ret['isAutoMode'] = rb_status.isAutoMode
        ret['isBusy'] = rb_status.isBusy
        ret['isMoveFinished'] = rb_status.isMoveFinished
        ret['isHome'] = rb_status.isHome
        ret['isZero'] = rb_status.isZero
        ret['isInResetting'] = rb_status.isInResetting
        ret['isInTeaching'] = rb_status.isInTeaching
        ret['isInDirectTeaching'] = rb_status.isInDirectTeaching
        ret['operatingMode'] = rb_status.operatingMode
        ret['operatingSource'] = rb_status.operatingSource
        return ret
