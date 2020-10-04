
# coding: utf-8

# In[2]:


from pkg.global_config import *
import ctypes
from ctypes import *


# In[3]:


clib = ctypes.cdll.LoadLibrary(os.path.join(TF_GMT_ETASL_DIR, "openGJK/lib/libopenGJKlib.so"))
clib.gjk_flat_batch.restype = ctypes.c_double


# In[4]:


OBJ_MAX = 100
VTX_MAX = 20
COL_MAX = 1000


# In[5]:


MAX_VTX_TYPE = c_double * 3 * VTX_MAX
MAX_VTX_ARR_TYPE = c_double * (VTX_MAX * 3)
IDX_ARR_TYPE = c_int * COL_MAX
DIST_ARR_TYPE = c_double * COL_MAX

class bd(Structure):
    _fields_ = [("numpoints", c_int),
                ("coord", POINTER(c_void_p)),
                ("s", c_double*3),
               ]

class bd_flt(Structure):
    _fields_ = [("numpoints", c_int),
                ("vtx_flat", MAX_VTX_ARR_TYPE)
               ]
    
BD_ARR_TYPE = bd * OBJ_MAX
BD_FLT_ARR_TYPE = bd_flt * OBJ_MAX


# In[6]:


points = []
for i in range(2):
    for j in range(2):
        for k in range(2):
            points.append((i,j,k))
points = np.array(points)


# In[7]:


points1 = points.copy()
points2 = points + 2


# In[8]:


bd_flt_arr = BD_FLT_ARR_TYPE()


# In[9]:


points_arr = [points1, points2]


# In[10]:


def assign_points_c_bd(bd_flt_arr, points_arr):
    for bd_flt, points in zip(bd_flt_arr, points_arr):
        bd_flt.numpoints = len(points)
        bd_flt.vtx_flat = MAX_VTX_ARR_TYPE(*points.flatten().tolist())


# In[11]:


assign_points_c_bd(bd_flt_arr, points_arr)


# In[16]:


len_col = len(points_arr)
dist_arr = (c_double*len_col)()
out = clib.gjk_flat_batch(bd_flt_arr, c_int(len_col), 
                   (c_int*len_col)(*[0]), (c_int*len_col)(*[1]), 
                    c_int(1), cast(dist_arr, POINTER(c_double)))

