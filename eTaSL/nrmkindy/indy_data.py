# -*- coding: utf-8 -*- 

# __JOINT_DOF = 6
#
# def set_joint_dof(dof):
#     global __JOINT_DOF
#     __JOINT_DOF = dof


class JointVec:
    __JOINT_DOF = 6

    def __init__(self, *q, **kwargs):
        if len(q) is self.__JOINT_DOF:
            self.q = q
            return
        elif len(q) is 1 and (isinstance(q[0], list) or isinstance(q[0], tuple)):
            if len(q) is not self.__JOINT_DOF:
                raise TypeError("JointPos values must be {} sized".format(self.__JOINT_DOF))
            self.q = tuple(q[0])
            return
        elif len(q) is 0:
            q2 = [0.0] * self.__JOINT_DOF
            for i in range(self.__JOINT_DOF):
                if "q{}".format(i) in kwargs:
                    q2[i] = kwargs["q{}".format(i)]
                    if not isinstance(q2[i], float) and not isinstance(q2[i], int):
                        raise TypeError("The value type must be float or int")
            self.q = tuple(q2)
            return
        else:
            raise TypeError("JointPos values must be {} sized".format(self.__JOINT_DOF))

    def get(self):
        return self.q

    def __str__(self):
        return "{}, {}, {}, {}, {}, {}".format(
            self.q[0], self.q[1], self.q[2], self.q[3], self.q[4], self.q[5])

    @classmethod
    def __set_dof(cls, dof):
        cls.__JOINT_DOF = dof


# JointPos = JointVec
class JointPos(JointVec):
    pass


class TaskVec:
    __TASK_LEN = 6

    def __init__(self, *p, **kwargs):
        if len(p) is self.__TASK_LEN:
            self.p = p
            return
        elif len(p) is 1 and (isinstance(p[0], list) or isinstance(p[0], tuple)):
            if len(p) is not self.__TASK_LEN:
                raise TypeError("TaskPos values must be {} sized".format(self.__TASK_LEN))
            self.p = tuple(p[0])
            return
        elif len(p) is 0:
            p2 = [0.0] * self.__TASK_LEN
            p2[0] = kwargs["x"] if "x" in kwargs else 0.0
            p2[1] = kwargs["y"] if "y" in kwargs else 0.0
            p2[2] = kwargs["z"] if "z" in kwargs else 0.0
            p2[3] = kwargs["u"] if "u" in kwargs else 0.0
            p2[4] = kwargs["v"] if "v" in kwargs else 0.0
            p2[5] = kwargs["w"] if "w" in kwargs else 0.0
            for i in range(self.__TASK_LEN):
                if not isinstance(p2[i], float) and not isinstance(p2[i], int):
                    raise TypeError("The value type must be float or int")
            self.p = tuple(p2)
            return
        else:
            raise TypeError("TaskPos values must be {} sized".format(self.__TASK_LEN))

    def get(self):
        return self.p

    def __str__(self):
        return "{}, {}, {}, {}, {}, {}".format(
            self.p[0], self.p[1], self.p[2], self.p[3], self.p[4], self.p[5])


class TaskPos(TaskVec):
    pass


class TaskPosProcess:
    pass
