import time
import collections
import numpy as np

class Singleton:
    __instance = None

    @classmethod
    def __getInstance(cls):
        return cls.__instance

    @classmethod
    def instance(cls, *args, **kargs):
        cls.__instance = cls(*args, **kargs)
        cls.instance = cls.__getInstance
        return cls.__instance

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
    