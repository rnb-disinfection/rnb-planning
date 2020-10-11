# -*- coding: utf-8 -*- 

import math
import grpc

from nrmkindy import IndyGeneral_pb2 as genr
from nrmkindy import IndyRemoteServices_pb2 as remote
from nrmkindy import IndyRemoteServices_pb2_grpc as remote_rpc

from nrmkindy import indy_model
# from model import *
from nrmkindy import indy_util
# import robot_ctrl_state

class ScriptDirector:
    def __init__(self, model_name):
        if model_name == indy_model.NAME_INDY_7:
            self.__model = indy_model.Indy7Model()
        elif indy_model == indy_model.NAME_INDY_RP2:
            self.__model = indy_model.IndyRP2Model()
        else:
            raise AttributeError("Undefined Indy Model.")

        self.__connected = False
        self.__grpc_channel = None
        self.__script_director_stub = None

        self.__use_radians = False

    def connect(self, ip_addr):
        try:
            self.__grpc_channel = grpc.insecure_channel(ip_addr + ':51911')
            self.__script_director_stub = remote_rpc.ScriptDirectorStub(self.__grpc_channel)
            return True
        except Exception as e:
            print(e)
            return False

    def disconnect(self):
        self.__grpc_channel.close()
        self.__grpc_channel = None
        self.__script_director_stub = None

    def startProgram(self):
        pass

    def stopProgram(self):
        pass

    def pauseProgram(self):
        pass

    def resumeProgram(self):
        pass