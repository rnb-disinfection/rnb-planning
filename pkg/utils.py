
from itertools import permutations, combinations, product

def get_all_mapping(A, B):
    return [{a:b for a,b in zip(A,item)} for item in list(product(B,repeat=len(A)))]


import time
import collections
import numpy as np

class GlobalTimer:
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
    
    
    
class LowPassFilter:
    def __init__(self, alpha, y_init=0):
        self.alpha = alpha
        self.y_pre = y_init
        
    def reset(self):
        self.y_pre = 0
        
    def __call__(self, y):
        self.y_pre = y*self.alpha + self.y_pre*(1-self.alpha)
        return self.y_pre
    
    
class CompareInterface:
        
    def __eq__(self, other):
        if isinstance(other, CompareInterface):
            return str(self) == str(other)
        return NotImplemented
    
    def __ne__(self, other):
        result = self.__eq__(other)
        if result is NotImplemented:
            return result
        return not result
        
    def __gt__(self, other):
        if isinstance(other, CompareInterface):
            return str(self) > str(other)
        return NotImplemented
        
    def __ge__(self, other):
        if isinstance(other, CompareInterface):
            return str(self) >= str(other)
        return NotImplemented
        
    def __lt__(self, other):
        if isinstance(other, CompareInterface):
            return str(self) < str(other)
        return NotImplemented
        
    def __le__(self, other):
        if isinstance(other, CompareInterface):
            return str(self) <= str(other)
        return NotImplemented
        