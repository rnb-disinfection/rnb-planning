import os;
import numpy as np;
# ## Spline 4 Example
# Testing the performance of the spline routines in combination with the expressiongraphs.

import math;
n=99000;
t=np.linspace(0,2*math.pi,n)
x=3*np.cos(t)+1;
y=3*np.sin(t);

A=np.vstack((t,x,y)).transpose()
A.shape

np.savetxt('input2.csv',A);


os.system('./bin/spline4')

# <markdowncell>

# Conclusion: you really do not have to care about the amount of points.  It always will
# be evaluated very quickly...., for a interpolating a spline (and computing the derivative) with 99000 points at 100000 points, and this 100 times
# after each other:  total time 16.24s or 1.6 microsec.
# 
# Most of the time is spend on reading the spline from the disk.

# <markdowncell>


