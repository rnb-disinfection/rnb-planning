# -*- coding: utf-8 -*- 

from nrmkindy import IndyGeneral_pb2 as genr
from nrmkindy import IndyIOState_pb2 as io_state
from nrmkindy import IndyComServices_pb2_grpc as io_rpc

from indy_data_io import *
from indy_abstract_ctrl import *

__metaclass__ = type

class IOController(AbstractController):
    def __init__(self):
        super(IOController, self).__init__()

    def connect(self, ip_addr):
        try:
            self._grpc_channel = grpc.insecure_channel(ip_addr + ':51915')
            self._stub = io_rpc.IOControllerStub(self._grpc_channel)
            _connected = True
            return True
        except Exception as e:
            print(e)
            return False

    # noinspection PyMethodMayBeStatic
    def _create_dio_set(self, dio_set):
        ret = list()
        for i in range(len(dio_set)):
            ref = io_state.DIO(idx=dio_set[i].get()[0],
                               value=dio_set[i].get()[1])
            ret.append(ref)
        ret = io_state.DIOSet(values=ret)
        return ret

    # noinspection PyMethodMayBeStatic
    def _create_aio_set(self, aio_set):
        ret = list()
        for i in range(len(aio_set)):
            ref = io_state.AIO(idx=aio_set[i].get()[0],
                               value=aio_set[i].get()[1])
            ret.append(ref)
        ret = io_state.AIOSet(values=ret)
        return ret

    def get_io(self):
        ret = self._stub.getIOState(genr.EmptyMessage())
        return ret

    def get_di(self):
        ret = self._stub.getDI(genr.EmptyMessage())
        di = list()
        for i in range(len(ret.values)):
            di.append(ret.values[i])
        return tuple(di)

    def get_do(self):
        ret = self._stub.getDO(genr.EmptyMessage())
        do = list()
        for i in range(len(ret.values)):
            do.append(ret.values[i])
        return tuple(do)

    def set_do(self, *args):
        if len(args) is 1 and isinstance(args[0], DIOSet):
            dio_set = args[0].get()
        else:
            dio_set = DIOSet(*args).get()
        msg_dio_set = self._create_dio_set(dio_set)
        self._stub.setDO(msg_dio_set)

    def get_ai(self):
        ret = self._stub.getAI(genr.EmptyMessage())
        ai = list()
        for i in range(len(ret.values)):
            ai.append(ret.values[i])
        return tuple(ai)

    def get_ao(self):
        ret = self._stub.getAO(genr.EmptyMessage())
        ao = list()
        for i in range(len(ret.values)):
            ao.append(ret.values[i])
        return tuple(ao)

    def set_ao(self, *args):
        if len(args) is 1 and isinstance(args[0], AIOSet):
            aio_set = args[0].get()
        else:
            aio_set = AIOSet(*args).get()
        msg_aio_set = self._create_aio_set(aio_set)
        self._stub.setAO(msg_aio_set)

    def get_safe_di(self):
        ret = self._stub.getSafetyDI(genr.EmptyMessage())
        safe_di = list()
        for i in range(len(ret.values)):
            safe_di.append(ret.values[i])
        return tuple(safe_di)

    def get_safe_do(self):
        ret = self._stub.getSafetyDO(genr.EmptyMessage())
        safe_do = list()
        for i in range(len(ret.values)):
            safe_do.append(ret.values[i])
        return tuple(safe_do)

    def set_safe_do(self, *args):
        if len(args) is 1 and isinstance(args[0], DIOSet):
            dio_set = args[0].get()
        else:
            dio_set = DIOSet(*args).get()
        msg_dio_set = self._create_dio_set(dio_set)
        self._stub.setSafetyDO(msg_dio_set)

    def get_endtool_di(self):
        ret = self._stub.getEndtoolDI(genr.EmptyMessage())
        endtool_di = list()
        for i in range(len(ret.values)):
            endtool_di.append(ret.values[i])
        return tuple(endtool_di)

    def get_endtool_do(self):
        ret = self._stub.getEndtoolDO(genr.EmptyMessage())
        endtool_do = list()
        for i in range(len(ret.values)):
            endtool_do.append(ret.values[i])
        return tuple(endtool_do)

    def set_endtool_do(self, *args):
        if len(args) is 1 and isinstance(args[0], DIOSet):
            dio_set = args[0].get()
        else:
            dio_set = DIOSet(*args).get()
        msg_dio_set = self._create_dio_set(dio_set)
        self._stub.setEndtoolDO(msg_dio_set)
