import sys
import os
from collections import Iterable
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], "examples/boost_python_example"))
try:
    import boost_python_example as bpe
except Exception as e:
    
    if hasattr(e, "args") and isinstance(e.args, Iterable) and len(e.args)>0 and e.args[0].startswith('No module named'):
        print("===========================================================\n"
              "boost_python_example is not built.\n"
              "run 'cmake -DCMAKE_BUILD_TYPE=Release . && make' on terminal.\n"
              "===========================================================\n")
    raise(e)

print("Test greet function")
recv = bpe.hello("python")
print("received: "+recv)

print("Generate c++ Eigen vector instance")
v = bpe.Vec3()

print("Access Eigen vector by index")
v[0], v[1], v[2] = 1, 2, 3
print(v[0], v[1], v[2])

print("Generate c++ stl container (of vectors)")
v_list = bpe.Vec3List()

print("At first, the size is 0")
print(len(v_list))

print("After appending item, the size gets 1")
v_list.append(v)
print(len(v_list))
