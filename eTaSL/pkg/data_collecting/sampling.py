import numpy as np

def sample_Trbt(Nrbt, robot_names, _wdh, _min_dist_robot):
    Trbt = []
    while len(Trbt)<Nrbt:
        _T = (np.random.random(3)*(_wdh[:2]+(_wdh[2]-2,)), (0,0,np.random.uniform(0,np.pi*2)))
        if not Trbt:
            Trbt.append(_T)
        else:
            if all([np.linalg.norm(_T[0]-_Told[0]) > _min_dist_robot for _Told in Trbt]):
                Trbt.append(_T)
    return {k:v for k,v in zip(robot_names, Trbt)}