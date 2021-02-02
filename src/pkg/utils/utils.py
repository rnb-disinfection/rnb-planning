
from itertools import permutations, combinations, product

def get_all_mapping(A, B):
    return [{a:b for a,b in zip(A,item)} for item in list(product(B,repeat=len(A)))]

import datetime
def get_now():
    return str(datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))

def try_mkdir(path):
    try: os.mkdir(path)
    except: pass

import time
import collections
import numpy as np
from .singleton import Singleton
import socket

def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]

def differentiate(X, dt):
    V = (X[1:]-X[:-1])/dt
    return np.concatenate([V, [V[-1]]], axis=0)

def integrate(V, dt, X0=0):
    return X0 + (np.cumsum(V, axis=0) - V) * dt

def interpolate_double(X):
    X_med = (X[:-1] + X[1:]) / 2
    return np.concatenate(
        [np.reshape(np.concatenate([X[:-1],X_med], axis=1), (-1, X.shape[1])),
        X[-1:]], axis=0)

def matmul_md(A, B):
    return np.sum((np.expand_dims(A, axis=-1) * np.expand_dims(B, axis=-3)), axis=-2)

def get_mean_std(X, outlier_count=2):
    X_ex = [x[0] for x in sorted(zip(X,np.linalg.norm(X-np.mean(X, axis=0), axis=1)), key=lambda x: x[1])][:-outlier_count]
    return np.mean(X_ex, axis=0), np.std(X_ex, axis=0)


class GlobalTimer(Singleton):
    def __init__(self, scale=1000, timeunit='ms'):
        self.reset(scale, timeunit)
        
    def reset(self, scale=1000, timeunit='ms'):
        self.scale = scale
        self.timeunit = timeunit
        self.name_list = []
        self.ts_dict = {}
        self.time_dict = collections.defaultdict(lambda: 0)
        self.min_time_dict = collections.defaultdict(lambda: 1e10)
        self.max_time_dict = collections.defaultdict(lambda: 0)
        self.count_dict = collections.defaultdict(lambda: 0)
        self.timelist_dict = collections.defaultdict(list)
        self.switch(True)
        
    def switch(self, onoff):
        self.__on = onoff
    
    def tic(self, name):
        if self.__on:
            if name not in self.name_list:
                self.name_list.append(name)
            self.ts_dict[name] = time.time()
        
    def toc(self, name, stack=False):
        if self.__on:
            dt = (time.time() - self.ts_dict[name]) * self.scale
            self.time_dict[name] = self.time_dict[name] + dt
            self.min_time_dict[name] = min(self.min_time_dict[name], dt)
            self.max_time_dict[name] = max(self.max_time_dict[name], dt)
            self.count_dict[name] = self.count_dict[name] + 1
            if stack:
                self.timelist_dict[name].append(dt)
            return dt
            
    def toctic(self, name_toc, name_tic, stack=False):
        dt = self.toc(name_toc, stack=stack)
        self.tic(name_tic)
        return dt
        
    def print_time_log(self, names=None):
        if names is None:
            names = self.name_list
        for name in names:
            print("{name}: \t{tot_T} {timeunit}/{tot_C} = {per_T} {timeunit} ({minT}/{maxT})\n".format(
                name=name, tot_T=np.round(np.sum(self.time_dict[name])), tot_C=self.count_dict[name], 
                per_T= np.round(np.sum(self.time_dict[name])/self.count_dict[name], 3),
                timeunit=self.timeunit, minT=round(self.min_time_dict[name],3), maxT=round(self.max_time_dict[name],3)
            ))
            
    def __str__(self):
        strout = "" 
        names = self.name_list
        for name in names:
            strout += "{name}: \t{tot_T} {timeunit}/{tot_C} = {per_T} {timeunit} ({minT}/{maxT})\n".format(
                name=name, tot_T=np.round(np.sum(self.time_dict[name])), tot_C=self.count_dict[name], 
                per_T= np.round(np.sum(self.time_dict[name])/self.count_dict[name], 3),
                timeunit=self.timeunit, minT=round(self.min_time_dict[name],3), maxT=round(self.max_time_dict[name],3)
            )
        return strout

    def block(self, key):
        return BlockTimer(self, key)

    def __enter__(self):
        self.tic("block")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.toc("block")

class BlockTimer:
    def __init__(self, gtimer, key):
        self.gtimer, self.key = gtimer, key

    def __enter__(self):
        self.gtimer.tic(self.key)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.gtimer.toc(self.key)
    
    
import os

def allow_korean_comment_in_dir(packge_dir):
    python_files = os.listdir(packge_dir)
    for filename in python_files:
        if '.py' in filename:
            file_path = os.path.join(packge_dir, filename)
            prepend_line(file_path, "# -*- coding: utf-8 -*- \n")
            
            
def prepend_line(file_name, line):
    """ Insert given string as a new line at the beginning of a file """
    # define name of temporary dummy file
    dummy_file = file_name + '.bak'
    # open original file in read mode and dummy file in write mode
    with open(file_name, 'r') as read_obj, open(dummy_file, 'w') as write_obj:
        # Write given line to the dummy file
        write_obj.write(line + '\n')
        # Read lines from original file one by one and append them to the dummy file
        for line in read_obj:
            write_obj.write(line)
    # remove original file
    os.remove(file_name)
    # Rename dummy file as the original file
    os.rename(dummy_file, file_name)

from functools import wraps

def record_time(func):
    gtimer = GlobalTimer.instance()
    @wraps(func)
    def __wrapper(*args, **kwargs):
        gtimer.tic(func.__name__)
        res = func(*args, **kwargs)
        gtimer.toc(func.__name__)
        return res
    return __wrapper

import json

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

def send_recv(sdict, host, port):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    client_socket.connect((host, port))

    rdict = {}
    try:
        sjson = json.dumps(sdict, cls=NumpyEncoder)
        sbuff = sjson.encode()
        client_socket.sendall(sbuff)

        rjson = client_socket.recv(1024)
        # rjson = "".join(map(chr, rjson))
        rdict = json.loads(rjson)
        if rdict is None:
            rdict = {}
        rdict = {str(k): v for k,v in rdict.items()}
    finally:
        client_socket.close()
    return rdict

# from filterpy.kalman import KalmanFilter
# from filterpy.common import Q_discrete_white_noise
#
# def createKF(dim_z, dt, P, R, Q, X0=None):
#     dim_x = dim_z*2
#     f = KalmanFilter (dim_x=dim_x, dim_z=dim_z)
#     f.x = np.array([0., 0.]*dim_z) if X0 is None else X0
#     f.F = np.identity(dim_x)  # state transition matrix
#     for i_F in range(dim_z):
#         f.F[i_F*2, i_F*2+1] = 1
#     f.H = np.identity(dim_x)[::2,:] # measurement function
#
#     f.P *= P #covariance matrix
#     f.R *= R # measurement noise
#     f.Q = np.identity(dim_x)
#     for i_Q in range(dim_z):
#         f.Q[i_Q*2:i_Q*2+2,i_Q*2:i_Q*2+2] = Q_discrete_white_noise(dim=2, dt=dt, var=Q)
#     return f

import inspect

def divide_kwargs(kwargs, func1, func2):
    keys1 = inspect.getargspec(func1).args
    keys2 = inspect.getargspec(func2).args
    kwargs1 = {k:v for k,v in kwargs.items() if k in keys1}
    kwargs2 = {k:v for k,v in kwargs.items() if k in keys2}
    return kwargs1, kwargs2

def inspect_arguments(func):
    argspec = inspect.getargspec(func)
    defaults = argspec.defaults if argspec.defaults is not None else []
    len_kwargs = len(defaults)
    args = argspec.args[:-len_kwargs] if len_kwargs > 0 else argspec.args
    return args, {k:v for k,v in zip(argspec.args[-len_kwargs:], defaults)}

def CallHolder(caller, arg_keys, *args, **kwargs):
    def fun(*args_rt, **kwargs_rt):
        kwargs_rt.update({k:v for k,v in zip(arg_keys, args_rt) if k is not None})
        kwargs_rt.update(kwargs)
        return caller(*args, **kwargs_rt)
    fun.caller=caller
    fun.arg_keys=arg_keys
    fun.args=args
    fun.kwargs=kwargs
    return fun


##
# @class LockBLock
# @brief block wrapper for Lock instance
class LockBlock:
    ##
    # @param lock Lock instance from multiprocessing or threading
    def __init__(self, lock):
        self.lock = lock

    def __enter__(self):
        self.lock.acquire()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.lock.release()


##
#@ class dummy class to imitate multiprocess.Value
class SingleValue:
    def __init__(self, _type, _value):
        self.type = _type
        self.value = _value

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise (RuntimeError('Boolean value expected.'))

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise(RuntimeError('Boolean value expected.'))
        
def round_it_str(iterable, dec=3):
    dec_str="%.{}f".format(dec)
    return ",".join(map(lambda x:dec_str%x, iterable))

def str_num_it(strnum, deliminater=","):
    if deliminater in strnum:
        return map(float, strnum.split(deliminater))
    else:
        return None

def str2num_split_blank(string, dtype=float):
    return map(dtype, " ".join(string.split()).split(" "))

import pickle
def save_pickle(filename, data):
    with open(filename, 'wb') as f:
        pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)

def load_pickle(filename):
    with open(filename, 'rb') as f:
        data = pickle.load(f)
        return data

def save_json(filename, data):
    with open(filename, "w") as json_file:
        json.dump(data, json_file, cls=NumpyEncoder,indent=2)

def load_json(filename):
    with open(filename, "r") as st_json:
        st_python = json.load(st_json)
    return st_python

def read_file(filename):
    buffer = ""
    with open(filename, 'r') as f:
        while True:
            line = f.readline()
            buffer += line
            if not line: break
    return buffer

def list2dict(item_list, item_names):
    return {jname: jval for jname, jval in zip(item_names, item_list)}

def dict2list(item_dict, item_names):
    return [item_dict[jname] for jname in item_names]

class Logger:
    ERROR_FOLDER = "logs"
    def __init__(self, countout=3):
        self.count=0
        self.countout=countout
        try: os.mkdir(self.ERROR_FOLDER)
        except: pass

    def log(self, error, prefix="", print_now=True):
        if prefix != "":
            prefix += "_"
        if print_now: print(error)
        self.count += 1
        save_json(os.path.join(self.ERROR_FOLDER, prefix + get_now()), error)
        return self.count < self.countout
