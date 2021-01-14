# -*- coding: utf-8 -*- 

import math
from indy_data_io import *


def to_radian(var_deg):
    if isinstance(var_deg, tuple):
        return tuple(math.radians(v) for v in var_deg)
    elif isinstance(var_deg, list):
        return [math.radians(v) for v in var_deg]
    elif isinstance(var_deg, int) or isinstance(var_deg, float):
        return math.radians(var_deg)
    else:
        Exception("The argument must be an instance of int, float, list. or tuple.")


def to_degree(var_rad):
    if isinstance(var_rad, tuple):
        return tuple(math.degrees(v) for v in var_rad)
    elif isinstance(var_rad, list):
        return [math.degrees(v) for v in var_rad]
    elif isinstance(var_rad, int) or isinstance(var_rad, float):
        return math.degrees(var_rad)
    else:
        Exception("The argument must be an instance of int, float, list. or tuple.")


def to_io_set(**kwargs):
    for key, value in kwargs.items():
        if key == 'di' or key == 'ai' or key == 'ei':
            if len(value) == 1 and isinstance(value, DIOSet):
                return value
            elif len(value) == 1 and isinstance(value, AIOSet):
                return value

            if key == 'di' or key == 'ei':
                return DIOSet(*value).get()
            elif key == 'ai':
                return AIOSet(*value).get()

