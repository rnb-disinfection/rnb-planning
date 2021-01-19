from .repeater.indy_repeater import *
from .repeater.panda_repeater import *
from .robot_config import *
from ..environment_builder import *


JOINT_LIM_DICT = []

##
# @class CombinedRobot
# @brief control interface for combined robot
class CombinedRobot:
    def __init__(self, robots_on_scene, connection_list, vel_scale=0.5, acc_scale=0.5):
        ## @brief velocity limit scale
        self.vel_scale = vel_scale
        ## @brief acceleration limit scale
        self.acc_scale = acc_scale
        self.__set_robots_on_scene(robots_on_scene)
        self.reset_connection(connection_list)

    def __set_robots_on_scene(self, robots_on_scene):
        ## @brief list of robot config for robots on scene
        self.robots_on_scene = robots_on_scene
        self.joint_names = []
        self.home_pose = []
        self.idx_dict = {}
        ## @brief dictionary of robot control interface
        self.robot_dict = {}
        self.robot_names = []
        self.custom_limits = defaultdict(dict)
        self.address_list = []
        self.xyz_rpy_robots = {}
        for rbt_config in self.robots_on_scene:
            r_id, _type = rbt_config.idx, rbt_config.type
            name = rbt_config.get_indexed_name()
            self.xyz_rpy_robots[name] = rbt_config.xyzrpy
            self.address_list.append(rbt_config.address)
            i0 = len(self.joint_names)
            joint_names_cur = RobotSpecs.get_joint_names(_type, name)
            self.joint_names += joint_names_cur
            for jname, lim_pair, vellim, acclim in zip(joint_names_cur, RobotSpecs.get_joint_limits(_type),
                                                        RobotSpecs.get_vel_limits(_type), RobotSpecs.get_acc_limits(_type)):
                self.custom_limits[jname].update({"lower":lim_pair[0], "upper":lim_pair[1],
                                                  "velocity": vellim*self.vel_scale, "effort": acclim*self.acc_scale})
            self.home_pose += RobotSpecs.get_home_pose(_type)
            self.idx_dict[name] = range(i0, len(self.joint_names))
            self.robot_dict[name] = None
            self.robot_names.append(name)
        self.joint_num = len(self.joint_names)
        self.home_pose = np.array(self.home_pose)
        self.home_dict = list2dict(self.home_pose, self.joint_names)

    ##
    # @brief reset connection
    # @param connection_list boolean list
    # @param address_list address list, None for default to use stored address
    def reset_connection(self, connection_list, address_list=None):
        self.connection_list = connection_list
        self.address_list = address_list or self.address_list
        print("connection_list")
        print(connection_list)
        for rbt, cnt, addr in zip(self.robots_on_scene, self.connection_list, self.address_list):
            name = rbt.get_indexed_name()
            _type = rbt.type
            if cnt:
                if _type == RobotType.indy7:
                    if not self.robot_dict[name]:
                        self.robot_dict[name] = indytraj_client(server_ip=addr, robot_name="NRMK-Indy7")
                    with self.robot_dict[name]:
                        self.robot_dict[name].set_collision_level(5)
                        self.robot_dict[name].set_joint_vel_level(3)
                        self.robot_dict[name].set_task_vel_level(3)
                        self.robot_dict[name].set_joint_blend_radius(20)
                        self.robot_dict[name].set_task_blend_radius(0.2)
                elif _type == RobotType.panda:
                    if self.robot_dict[name]:
                        if hasattr(self.robot_dict[name], 'alpha_lpf'):
                            self.robot_dict[name].set_alpha_lpf(self.robot_dict[name].alpha_lpf)
                        if hasattr(self.robot_dict[name], 'd_gain'):
                            self.robot_dict[name].set_d_gain(self.robot_dict[name].d_gain)
                        if hasattr(self.robot_dict[name], 'k_gain'):
                            self.robot_dict[name].set_k_gain(self.robot_dict[name].k_gain)
                    else:
                        self.robot_dict[name] = PandaRepeater(*addr.split("/"))
            else:
                if self.robot_dict[name] is not None:
                    self.robot_dict[name].disconnect()
                    self.robot_dict[name] = None

    ##
    # @brief get robot_on_scene in dictionalry format
    def get_scene_dict(self):
        return {rp.get_indexed_name(): rp for rp in self.robots_on_scene}

    ##
    # @brief get list of robot controller interface
    def get_robot_list(self):
        return [self.robot_dict[name] for name in self.robot_names]

    ##
    # @brief get list of each robot's joint indexes
    def get_indexing_list(self):
        return [self.idx_dict[name] for name in self.robot_names]

    ##
    # @brief move to joint pose target
    # @param Q motion target(rad)
    def joint_make_sure(self, Q):
        for name, rconfig in zip(self.robot_names, self.robots_on_scene):
            _type = rconfig.type
            if _type == RobotType.indy7:
                self.robot_dict[name].joint_move_make_sure(np.rad2deg(Q[self.idx_dict[name]]), N_repeat=2, connect=True)
            elif _type == RobotType.panda:
                self.robot_dict[name].move_joint_interpolated(Q[self.idx_dict[name]], N_div=200)

    ##
    # @brief execute grasping action
    # @param grasp_dict boolean grasp commands in dictionary form {robot_name: grasp_bool}
    def grasp_by_dict(self, grasp_dict):
        grasp_seq = [(k, v) for k, v in grasp_dict.items()]
        grasp_seq = list(sorted(grasp_seq, key=lambda x: not x[1]))
        for grasp in grasp_seq:
            self.__grasp_fun(grasp[0], grasp[1])

    def __grasp_fun(self, name, grasp):
        scence_dict = self.get_scene_dict()
        if scence_dict[name] == RobotType.indy7 and self.robot_dict[name] is not None:
            self.robot_dict[name].grasp(grasp, connect=True)
        elif scence_dict[name] == RobotType.panda and self.robot_dict[name] is not None:
            self.robot_dict[name].move_finger(grasp)

    ##
    # @brief get current robot's pose or home pose if not connected (radian)
    def get_real_robot_pose(self):
        Q = []
        for name in self.robot_names:
            if self.robot_dict[name] is not None:
                Q += list(self.robot_dict[name].get_qcur())
            else:
                Q += list(self.home_pose[self.idx_dict[name]])
        return np.array(Q)

    ##
    # @brief wait for the first robot's ROS control duration
    def wait_step(self, rate_off):
        if all(self.connection_list):
            self.robot_dict[self.robot_names[0]].rate_x1.sleep()
        else:
            rate_off.sleep()

    # def detect_robots(self, cam):
    #     self.xyz_rpy_robots = cam.detect_robots(self.robots_on_scene)

    # def update_urdf(self):
    #     xcustom, self.joint_names, self.link_names, self.urdf_content = set_custom_robots(self.robots_on_scene,
    #                                                                                       self.xyz_rpy_robots,
    #                                                                                       self.custom_limits)
    #     return xcustom, self.joint_names, self.link_names, self.urdf_content