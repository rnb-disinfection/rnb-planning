
from itertools import permutations, combinations, product

def get_all_mapping(A, B):
    return [{a:b for a,b in zip(A,item)} for item in list(product(B,repeat=len(A)))]



import time
import collections
import numpy as np
from .singleton import Singleton
import socket

def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]

class GlobalTimer(Singleton):
    def __init__(self, scale=1000):
        self.name_list = []
        self.ts_dict = {}
        self.time_dict = collections.defaultdict(lambda: 0)
        self.count_dict = collections.defaultdict(lambda: 0)
        self.scale = scale
        self.switch(True)
        
    def reset(self):
        self.name_list = []
        self.ts_dict = {}
        self.time_dict = collections.defaultdict(lambda: 0)
        self.count_dict = collections.defaultdict(lambda: 0)
        self.switch(True)
        
    def switch(self, onoff):
        self.__on = onoff
    
    def tic(self, name):
        if self.__on:
            if name not in self.name_list:
                self.name_list += [name]
            self.ts_dict[name] = time.time()
        
    def toc(self, name):
        if self.__on:
            self.time_dict[name] = self.time_dict[name]+(time.time() - self.ts_dict[name]) * self.scale
            self.count_dict[name] = self.count_dict[name] + 1
            
    def toctic(self, name_toc, name_tic):
        self.toc(name_toc)
        self.tic(name_tic)
        
    def print_time_log(self, names=None, timeunit="ms"):
        if names is None:
            names = self.name_list
        for name in names:
            print("{name}: \t{tot_T} {timeunit}/{tot_C} = {per_T} {timeunit}".format(
                name=name, tot_T=np.round(np.sum(self.time_dict[name])), tot_C=self.count_dict[name], 
                per_T= np.round(np.sum(self.time_dict[name])/self.count_dict[name], 3),
                timeunit=timeunit
            ))
            
    def __str__(self):
        strout = ""
        timeunit="ms"
        names = self.name_list
        for name in names:
            strout += "{name}: \t{tot_T} {timeunit}/{tot_C} = {per_T} {timeunit} \n".format(
                name=name, tot_T=np.round(np.sum(self.time_dict[name])), tot_C=self.count_dict[name], 
                per_T= np.round(np.sum(self.time_dict[name])/self.count_dict[name], 3),
                timeunit=timeunit
            )
        return strout
    
    
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