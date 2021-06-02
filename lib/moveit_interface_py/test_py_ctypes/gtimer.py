import time
import collections
import numpy as np


##
# @class    Singleton
# @brief    Template to make a singleton class.
# @remark   Inherit this class to make a class a singleton.
#           Do not call the class constructor directly, but call <class name>.instance() to get singleton instance.
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


##
# @class    GlobalTimer
# @brief    A singleton timer to record timings anywhere in the code.
# @remark   Call GlobalTimer.instance() to get the singleton timer.
#           To see the recorded times, just print the timer: print(global_timer)
# @param    scale       scale of the timer compared to a second. For ms timer, 1000
# @param    timeunit    name of time unit for printing the log
# @param    stack   default value for "stack" in toc
class GlobalTimer(Singleton):
    def __init__(self, scale=1000, timeunit='ms', stack=False):
        self.reset(scale, timeunit, stack)

    ##
    # @brief    reset the timer.
    # @param    scale       scale of the timer compared to a second. For ms timer, 1000
    # @param    timeunit    name of time unit for printing the log
    # @param    stack   default value for "stack" in toc
    def reset(self, scale=1000, timeunit='ms', stack=False):
        self.stack = stack
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

    ##
    # @brief    switch for recording time. switch-off to prevent time recording for optimal performance
    def switch(self, onoff):
        self.__on = onoff

    ##
    # @brief    mark starting point of time record
    # @param    name    name of the section to record time.
    def tic(self, name):
        if self.__on:
            if name not in self.name_list:
                self.name_list.append(name)
            self.ts_dict[name] = time.time()

    ##
    # @brief    record the time passed from last call of tic with same name
    # @param    name    name of the section to record time
    # @param    stack   to stack each time duration to timelist_dict, set this value to True,
    #                   don't set this value to use default setting
    def toc(self, name, stack=None):
        if self.__on:
            dt = (time.time() - self.ts_dict[name]) * self.scale
            self.time_dict[name] = self.time_dict[name] + dt
            self.min_time_dict[name] = min(self.min_time_dict[name], dt)
            self.max_time_dict[name] = max(self.max_time_dict[name], dt)
            self.count_dict[name] = self.count_dict[name] + 1
            if stack or (stack is None and self.stack):
                self.timelist_dict[name].append(dt)
            return dt

    ##
    # @brief    record and start next timer in a line.
    def toctic(self, name_toc, name_tic, stack=None):
        if self.__on:
            dt = self.toc(name_toc, stack=stack)
            self.tic(name_tic)
            return dt

    ##
    # @brief you can just print the timer instance to see the record
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
    