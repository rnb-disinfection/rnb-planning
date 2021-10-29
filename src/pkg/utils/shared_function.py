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
## call function just as single process program                                         \n
res = hi(1)                                                                             \n
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
    ASYNC = 0
    SYNC = 1


def __to_dtype(etype):
    if etype == None:
        return None
    if etype == bool:
        return np.bool_
    if etype == int:
        return np.int64
    if etype == float:
        return np.float64
    else:
        return etype


def get_shared_array(addr, shape, etype):
    dtype = __to_dtype(etype)
    sm_dict = {sm.name.decode(): sm for sm in sa.list()}
    if addr in sm_dict:
        ex_shape, ex_dtype = sm_dict[addr].dims, sm_dict[addr].dtype
        if ex_shape == shape and ex_dtype == dtype:
            return sa.attach("shm://" + addr)
        else:
            raise (TypeError(
                "SharedArray {} exists but property does not match: ({}, {}) <- ({},{})".format(
                    addr, ex_shape, ex_dtype, shape, dtype)))
    else:
        return sa.create("shm://" + addr, shape=shape, dtype=dtype)


def shared_fun(ctype, *var_props):
    def __decorator(func):
        SHARED_FUNC_ALL.append(func)
        fullname_fun = fullname(func)
        func.request = get_shared_array(shared_fun_uri(fullname_fun, "__request__"), shape=(1,), etype=bool)
        func.response = get_shared_array(shared_fun_uri(fullname_fun, "__request__"), shape=(1,), etype=bool)
        func.kwargs = {}
        func.returns = {}
        for var_prop in var_props:
            if isinstance(var_prop, ResProb):
                addr = shared_fun_uri(fullname_fun, var_prop.index)
                func.returns[var_prop.index] = \
                    get_shared_array(addr, shape=var_prop.shape, etype=var_prop.etype)
            elif isinstance(var_prop, ArgProb):
                addr = shared_fun_uri(fullname_fun, var_prop.name)
                func.kwargs[var_prop.name] = \
                    get_shared_array(addr, shape=var_prop.shape, etype=var_prop.etype)
            else:
                raise (TypeError("arguments for shared_fun should be either ArgProb or ResProb"
                                 "but {} is given".format(type(var_prop))))
        func.returns = [func.returns[idx] for idx in sorted(func.returns.keys())]

        @wraps(func)
        def __wrapper(**kwargs):
            if SERVING:
                __run_send_response(func)
            if ctype == CallType.ASYNC:
                __send_request(func, **kwargs)
            elif ctype == CallType.SYNC:
                __send_request_wait(func, **kwargs)
            else:
                raise (TypeError("ctype should be CallType member"))
            return res

        return __wrapper

    return __decorator


def assign_val(sm, val):
    sm[:] = val
    raise (NotImplementedError("Extend dtype"))


def read_val(sm):
    raise (NotImplementedError("Extend dtype"))
    return sm


def __run_send_response(func):
    returns = func(**func.kwargs)
    for tar, src in zip(func.returns, returns):
        assign_val(tar, src)
    assign_val(func.request, False)
    assign_val(func.response, True)


def __send_request(func, **kwargs):
    for k, val in kwargs.items():
        assign_val(func.kwargs[k], val)
    assign_val(func.response, False)
    assign_val(func.request, True)


def __send_request_wait(func, **kwargs):
    __send_request(func, **kwargs)
    return check_response(func, wait=True)


##
# @breif get response from shared program. \n
#        return None if not request is not finished and wait=False
def check_response(func, wait=False):
    if func.response[0]:
        return [copy.deepcopy(val) for val in func.returns]
    elif wait:
        while not func.response[0]:
            time.sleep(SF_PERIOD)
        return [copy.deepcopy(val) for val in func.returns]
    else:
        return None


##
# @brief check if a computation server is running with given server_id
def check_paired(server_id="all"):
    pairing_uri = shared_fun_uri("server.{}".format(server_id), "paired")
    paired = get_shared_array(pairing_uri, (1,), bool)
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
        req_dict = {func.reqest: func for func in shared_funcs}
        while paired[0]:
            for req, func in req_dict.items():
                if req[0]: func()
            time.sleep(SF_PERIOD)
    except Exception as e:
        print(e)
    finally:
        assign_val(paired, False)
        time.sleep(0.1)
        sa.delete(pairing_uri)