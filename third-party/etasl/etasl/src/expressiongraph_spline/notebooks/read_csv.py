# reads a csv file and corresponding header.

import csv
import numpy as np;


def read(fn):
    f = open(fn,'r')
    data = csv.reader(f)
    h=data.next()
    f.close()
    count=0
    names={};
    for fld in h:
        names[count] = fld
        names[fld] = count
        count = count + 1
    data=np.loadtxt(fn,skiprows=1,delimiter=",")
    return (data,names)
