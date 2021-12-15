import numpy as np
from enum import Enum

##
# @class RobotType
# @brief Robot type declaration
class RobotType(Enum):
    indy7=0
    panda=1
    indy7gripper=2
    indy5dof=3
    kmb=4
    indy7kiro=102


##
# @class RobotTemplate
# @brief Robot spec template
class RobotTemplate:
    ##
    # @param robot_name robot type name
    # @param base_name base link name for the robot
    # @param base_name base link name for the robot, without robot name prefix
    # @param tip_name tip link name for the robot, without robot name prefix
    # @param joint_names movable joint names from base to tip order
    # @param home_pose home joint configuration, base to tip order
    # @param joint_limits list of (lower, upper) limits
    # @param vel_limits velocity limit
    # @param acc_limits acceleration limit, recorded as effort in urdf
    # @param shoulder_reach reach from the shoulder joint (usually the 2nd joint)
    def __init__(self, robot_name, base_name, tip_name, joint_names, home_pose, joint_limits, vel_limits, acc_limits, shoulder_reach):
        self.robot_name, self.base_name, self.tip_name, self.joint_names, \
            self.home_pose, self.joint_limits, self.vel_limits, self.acc_limits, self.shoulder_reach = \
            robot_name, base_name, tip_name, joint_names, home_pose, joint_limits, vel_limits, acc_limits, shoulder_reach


##
# @class RobotSpecs
# @brief Global robot spec definition
class RobotSpecs:
    SPEC_DICT = {
        RobotType.indy7: RobotTemplate(robot_name='indy', base_name="link0", tip_name="tcp",
                                       joint_names=["joint{}".format(idx) for idx in range(6)],
                                       home_pose=[0, 0, -np.pi / 2, 0, -np.pi / 2, 0],
#                                        joint_limits=[(-np.pi*2/3, np.pi*2/3), (-np.pi/2, np.pi/2)] \
                                       joint_limits=[(-3.05432619099, 3.05432619099)]*2 \
                                                    +[(-3.05432619099, 3.05432619099)]*3 \
                                                    +[(-3.75245789179, 3.75245789179)],
                                       vel_limits=np.deg2rad([150, 150, 150, 180, 180, 180])/2,
                                       acc_limits=np.deg2rad([180]*6)/2,
                                       shoulder_reach=1.04),
        RobotType.indy7gripper: RobotTemplate(robot_name='indy', base_name="link0", tip_name="tcp",
                                       joint_names=["joint{}".format(idx) for idx in range(6)],
                                       home_pose=[0, 0, -np.pi / 2, 0, -np.pi / 2, 0],
                                       # joint_limits=[(-np.pi*2/3, np.pi*2/3), (-np.pi/2, np.pi/2)] \
                                       joint_limits=[(-3.05432619099, 3.05432619099)]*2 \
                                                    +[(-3.05432619099, 3.05432619099)]*3 \
                                                    +[(-3.75245789179, 3.75245789179)],
                                       vel_limits=np.deg2rad([150, 150, 150, 180, 180, 180])/2,
                                       acc_limits=np.deg2rad([180]*6)/2,
                                       shoulder_reach=1.04),
        RobotType.panda: RobotTemplate(robot_name='panda', base_name="link0", tip_name="hand",
                                       joint_names=["joint{}".format(idx) for idx in range(1,8)],
                                       home_pose=[0, -np.pi / 8, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 2],
                                       joint_limits=[(-2.75, 2.75), (-1.70, 1.70), (-2.75, 2.75),
#                                        joint_limits=[(-np.pi*2/3, np.pi*2/3), (-1.70, 1.70), (-np.pi*2/3, np.pi*2/3),
                                                     (-2.9, -0.1), (-2.75, 2.75), (0.1, 3.6), (-2.75, 2.75)],
                                       vel_limits=np.deg2rad([150, 150, 150, 150, 180, 180, 180])/2,
                                       acc_limits=np.deg2rad([180]*7)/2,
                                       shoulder_reach=0.85),
        RobotType.indy5dof: RobotTemplate(robot_name='indy', base_name="link0", tip_name="tcp",
                                       joint_names=["joint{}".format(idx) for idx in [0,1,2,4,5]],
                                       home_pose=[0, 0, -np.pi / 2, -np.pi / 2, 0],
                                       joint_limits=[(-np.pi*2/3, np.pi*2/3), (-np.pi/2, np.pi/2)] \
                                       # joint_limits=[(-3.05432619099, 3.05432619099)]*2 \
                                                    +[(-3.05432619099, 3.05432619099)] \
                                                    # +[(-0, 0)] \
                                                    +[(-3.75245789179, 3.75245789179)]*2,
                                       vel_limits=np.deg2rad([150, 150, 150,  180, 180])/2,
                                       acc_limits=np.deg2rad([180]*5)/2,
                                       shoulder_reach=1.04),
        RobotType.kmb: RobotTemplate(robot_name='kmb', base_name="link0", tip_name="platform",
                                       joint_names=["joint{}".format(idx) for idx in range(6)],
                                       home_pose=[0,]*6,
                                       joint_limits=[None,None, None, (-1e-3,1e-3), (-1e-3,1e-3), (-1e-3,1e-3)],
                                       vel_limits=[None,None, None, 0,0,0],
                                       acc_limits=[None,None, None, 0,0,0],
                                       shoulder_reach=1e1),
        RobotType.indy7kiro: RobotTemplate(robot_name='indy', base_name="link0", tip_name="tcp",
                                       joint_names=["joint{}".format(idx) for idx in range(7)],
                                       home_pose=np.deg2rad([  0., 50.,  -70.,  -0.,  -90., 0, -45]),
#                                        joint_limits=[(-np.pi*2/3, np.pi*2/3), (-np.pi/2, np.pi/2)] \
                                       joint_limits=[(-2.95, 2.95)]*2 \
                                                    +[(-3.00, 3.00)]*3 \
                                                    +[(-3.70, 3.70)] \
                                                    # +[(-np.pi*1/4-0.001, -np.pi*1/4+0.001)], \
                                                    +[(-np.pi*5/18, np.pi*5/18)],
                                       vel_limits=np.deg2rad([150, 150, 150, 180, 180, 180, 180])/2,
                                       acc_limits=np.deg2rad([180]*7)/2,
                                       shoulder_reach=1.5),
    }

    @classmethod
    def get_robot_name(cls, _type):
        return cls.SPEC_DICT[_type].robot_name
    @classmethod
    def get_home_pose(cls, _type):
        return cls.SPEC_DICT[_type].home_pose

    @classmethod
    def get_base_name(cls, _type, rname):
        return rname + "_" + cls.SPEC_DICT[_type].base_name

    @classmethod
    def get_tip_name(cls, _type, rname):
        return rname + "_" + cls.SPEC_DICT[_type].tip_name

    @classmethod
    def get_joint_names(cls, _type, rname):
        return [rname + "_" + jname for jname in cls.SPEC_DICT[_type].joint_names]

    @classmethod
    def get_joint_limits(cls, _type, none_as_inf=False):
        if none_as_inf:
            return map(lambda x: x if x is not None else (-np.inf, np.inf),
                       cls.SPEC_DICT[_type].joint_limits)
        else:
            return cls.SPEC_DICT[_type].joint_limits

    @classmethod
    def get_vel_limits(cls, _type):
        return cls.SPEC_DICT[_type].vel_limits

    @classmethod
    def get_acc_limits(cls, _type):
        return cls.SPEC_DICT[_type].acc_limits

    @classmethod
    def get_shoulder_reach(cls, _type):
        return cls.SPEC_DICT[_type].shoulder_reach


##
# @class RobotConfig
# @brief Robot configuration containing index, type, location and address
class RobotConfig:
    ##
    # @param idx index of robot
    # @param type type of robot, declared in robot_config.py
    # @param xyzrpy location of robot in tuple (xyz(m), rpy(rad))
    # @param address ip address of robot string
    # @param specs  dictionary to describe additional characteristics
    def __init__(self, idx, type, xyzrpy, address, root_on="base_link", specs=None):
        self.idx, self.type, self.xyzrpy, self.address = idx, type, xyzrpy, address
        self.root_on = root_on
        self.specs = {} if specs is None else specs

    ##
    # @brief get robot name + index (id for urdf)
    def get_indexed_name(self):
        return "{}{}".format(RobotSpecs.get_robot_name(self.type), self.idx)
