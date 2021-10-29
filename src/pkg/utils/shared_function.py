"""@package shared_function
This package is a helper to make function runs in a totally separatedshared process,    \n
just as single process                                                                  \n
                                                                                        \n
###### EXAMPLE USAGE #####                                                              \n
------------------ test_module.py -------------------                                   \n
import shared_function                                                                  \n
import subprocess                                                                       \n
                                                                                        \n
@shared_function.shared_fun(                                                            \n
    CallType.SYNC, ResProb(1, (1,), bool), ArgProb("arg1", (1,), float))                \n
def hi(arg1):                                                                           \n
    return [arg1[0]>0]                                                                  \n
                                                                                        \n
## add server initialization on main so that created subprocess can start server        \n
if __name__ == "__main__":                                                              \n
    shared_function.serve_forever()                                                     \n
------------------------------------------------------                                  \n
                                                                                        \n
--------------- Your main procedure ------------------                                  \n
## run separated process                                                                \n
self.subp = subprocess.Popen(['python', 'test_module.py'])                              \n
                                                                                        \n
## call function just as single process program, but must call with kwargs              \n
res = hi(arg1=1)                                                                        \n
print(res)                                                                              \n
------------------------------------------------------                                  \n
                                                                                        \n
"""

import SharedArray as sa
from decorator import decorator
from functools import wraps
from collections import namedtuple
from enum import Enum
import numpy as np
import copy
from threading import Thread, Event

##
# @brief flag to distinguish master and slave \n
#        set SERVING = True to run function body, as in serve_forever
SERVING = False

##
# @brief shared function loop period is 1ms by default
SF_PERIOD = 1e-3

SHARED_FUN_URI_FORM = "shared_fun.{}.{}"

SHARED_FUNC_ALL = []


def shared_fun_uri(addr, identifier):
    return SHARED_FUN_URI_FORM.format(addr, identifier)

##
# @brief get full name of given function including parent class
def fullname(fun):
    if hasattr(fun, '__qualname__'):
        return ".".join(fun.__qualname__.split(".")[-2:])
    elif hasattr(fun, 'im_class'):
        return "{}.{}".format(fun.im_class.__name__, fun.__name__)
    else:
        return fun.__name__


##
# @class ArgProb
# @brief properties for shared memory variable
# @param name   name of function argument
# @param shape tuple
# @param etype extended data type,
#              basic data type in SharedArray is any subtype of bool, int, float but not other object s.t. str
#              extended data type support str and json
ArgProb = namedtuple("ArgProb", ["name", "shape", "etype"])

##
# @class ResProb
# @brief same as VarProb but for results
# @param index return value is distinguished by index
ResProb = namedtuple("ResProb", ["index", "shape", "etype"])


##
# @class CallType
# @brief flag for waiting for response (sync) or not (async)
class CallType(Enum):
    ASYNC = 0       # wait and return response
    SYNC = 1        # do not wait response, you can get response by check_response later



class ExtendedData:
    def __init__(self, addr, shape, etype):
        self.addr, self.shape, self.etype = addr, shape, etype
        self.dtype = dtype = self.dtype(etype)
        sm_dict = {sm.name.decode(): sm for sm in sa.list()}
        if addr in sm_dict:
            ex_shape, ex_dtype = sm_dict[addr].dims, sm_dict[addr].dtype
            if ex_shape == shape and ex_dtype == self.dtype:
                self.sm = sa.attach("shm://" + addr)
            else:
                raise (TypeError(
                    "SharedArray {} exists but property does not match: ({}, {}) <- ({},{})".format(
                        addr, ex_shape, ex_dtype, shape, dtype)))
        else:
            self.sm = sa.create("shm://" + addr, shape=shape, dtype=dtype)


    def assign(self, data):
        if self.etype == dict:
            sjson = json.dumps(data, cls=NumpyEncoder, ensure_ascii=False)
            self.sm[:len(sjson)] = map(ord, sjson)
        elif self.etype == str:
            self.sm[:len(data)] = map(ord, data)
        else:
            self.sm[:] = data

    def read(self):
        if self.etype == dict:
            data = map(chr, self.sm[:])
            sjson = "".join(data[:np.where(sm[:] == 0)[0][0]])
            return json.loads(rjson)
        elif self.etype == str:
            data = map(chr, self.sm[:])
            return "".join(data[:np.where(sm[:] == 0)[0][0]])
        else:
            return np.copy(self.sm)

    @classmethod
    def dtype(cls, etype):
        if etype == bool:
            return np.bool_
        elif etype == int:
            return np.int64
        elif etype == float:
            return np.float64
        elif etype == dict:
            return np.uint8
        elif etype == str:
            return np.uint8
        else:
            return etype

def shared_fun(ctype, *var_props):
    def __decorator(func):
        SHARED_FUNC_ALL.append(func)
        fullname_fun = fullname(func)
        func.request = ExtendedData(shared_fun_uri(fullname_fun, "__request__"), shape=(1,), etype=bool)
        func.response = ExtendedData(shared_fun_uri(fullname_fun, "__request__"), shape=(1,), etype=bool)
        func.kwargs = {}
        func.returns = {}
        func.ctype = ctype
        for var_prop in var_props:
            if isinstance(var_prop, ResProb):
                addr = shared_fun_uri(fullname_fun, var_prop.index)
                func.returns[var_prop.index] = \
                    ExtendedData(addr, shape=var_prop.shape, etype=var_prop.etype)
            elif isinstance(var_prop, ArgProb):
                addr = shared_fun_uri(fullname_fun, var_prop.name)
                func.kwargs[var_prop.name] = \
                    ExtendedData(addr, shape=var_prop.shape, etype=var_prop.etype)
            else:
                raise (TypeError("arguments for shared_fun should be either ArgProb or ResProb"
                                 "but {} is given".format(type(var_prop))))
        func.returns = [func.returns[idx] for idx in sorted(func.returns.keys())]

        @wraps(func)
        def __wrapper(**kwargs):
            if SERVING:
                res = __run_send_response(func)
            if ctype == CallType.ASYNC:
                res = __send_request(func, **kwargs)
            elif ctype == CallType.SYNC:
                res = __send_request_wait(func, **kwargs)
            else:
                raise (TypeError("ctype should be CallType member"))
            return res

        return __wrapper

    return __decorator

def __run_send_response(func):
    returns = func(**{k: v.read() for k, v in func.kwargs.items()})
    for edat, val in zip(func.returns, returns):
        edat.assign(src)
    func.request.assign(False)
    func.response.assign(True)


def __send_request(func, **kwargs):
    for k, val in kwargs.items():
        func.kwargs[k].assign(val)
    func.response.assign(False)
    func.request.assign(True)


def __send_request_wait(func, **kwargs):
    __send_request(func, **kwargs)
    return check_response(func, wait=True)


##
# @breif    get response from shared program. \n
#           return None if not request is not finished and wait=False
def check_response(func, wait=False, timeout=None):
    if func.response.read()[0]:
        return get_return(func)
    elif wait:
        time_start = 0
        while not func.response.read()[0]:
            time.sleep(SF_PERIOD)
            if time.time() - time_start > timeout:
                break
        if func.response.read()[0]:
            return get_return(func)
        else:
            return None
    else:
        return None

def get_return(func):
    res = map(ExtendedData.read, func.returns)
    if res is None or len(res) == 0:
        res = None
    elif len(res) == 1:
        res = res[0]
    return res


##
# @brief check if a computation server is running with given server_id
def check_paired(server_id="all"):
    pairing_uri = shared_fun_uri("server.{}".format(server_id), "paired")
    paired = ExtendedData(pairing_uri, (1,), bool)
    return paired


##
# @brief serve forever
# @param shared_funs  list of all shared functions to be served
# @param server_id    to prevent duplicated execution, use unique and consistent server_id
def serve_forever(shared_funcs=SHARED_FUNC_ALL, server_id="all"):
    global SERVING
    SERVING = True
    try:
        paired = check_paired(server_id)
        assert paired[0], "[FATAL] A server paired with id {} already exists. shutting down all"
        assign_val(paired, True)
        print("=========================")
        print("[INFO] Start serving for:")
        for func in shared_funcs:
            print(" * {}".format(fullname(func)))
        print("=========================")
        while paired[0]:
            for func in shared_funcs:
                if func.request.read()[0]: func()
            time.sleep(SF_PERIOD)
    except Exception as e:
        print(e)
    finally:
        assign_val(paired, False)
        time.sleep(0.1)
        sa.delete(pairing_uri)

import json

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)
