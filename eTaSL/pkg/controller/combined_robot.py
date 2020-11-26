from .indy_repeater import *
from .panda_repeater import *
from ..global_config import *
from ..environment_builder import *

ROBOTS_ON_SCENE_DEFAULT = [("indy0", RobotType.indy7_robot), ("panda1", RobotType.panda_robot)]
ROBOTS_ADDRESS_DEFAULT = [DEFAULT_INDY_IP, "{}/{}".format(DEFAULT_REPEATER_IP, DEFAULT_ROBOT_IP)]
XYZ_RPY_ROBOTS_DEFAULT = {'indy0': ([-0.44648051261901855, 0.251528263092041, 0.009795188903808594],
                                    [7.722511439072125e-05, 0.012837732857963772, -1.5843305292051728]),
                          'panda1': ([0.5117020606994629, 0.16830319166183472, 0.014661192893981934],
                                     [0.0037190383704881766, 0.013066991871646852, -1.6051065831214242])}

JOINT_HOME_DICT = {RobotType.indy7_robot: [0, 0, -np.pi / 2, 0, -np.pi / 2, 0],
                   RobotType.panda_robot: [0, -np.pi / 8, 0, -np.pi / 2, 0, np.pi / 2, np.pi / 2]}


class CombinedRobot:
    def __init__(self, connection_list,
                 robots_on_scene=ROBOTS_ON_SCENE_DEFAULT, xyz_rpy_robots=XYZ_RPY_ROBOTS_DEFAULT,
                 address_list=ROBOTS_ADDRESS_DEFAULT, indy_joint_vel_level=3, indy_task_vel_level=3):
        self.set_robots_on_scene(robots_on_scene)
        self.reset_connection(connection_list, address_list)
        self.xyz_rpy_robots = xyz_rpy_robots
        self.indy_joint_vel_level = indy_joint_vel_level
        self.indy_task_vel_level = indy_task_vel_level

    def set_robots_on_scene(self, robots_on_scene):
        self.robots_on_scene = robots_on_scene
        self.joint_names = []
        self.home_pose = []
        self.idx_dict = {}
        self.robot_dict = {}
        self.robot_names = []
        for name, _type in self.robots_on_scene:
            i0 = len(self.joint_names)
            self.joint_names += RobotType.get_joints(_type, name)
            self.home_pose += JOINT_HOME_DICT[_type]
            self.idx_dict[name] = range(i0, len(self.joint_names))
            self.robot_dict[name] = None
            self.robot_names.append(name)
        self.joint_num = len(self.joint_names)
        self.home_pose = np.array(self.home_pose)
        self.home_dict = list2dict(self.home_pose, self.joint_names)

    def reset_connection(self, connection_list, address_list=ROBOTS_ADDRESS_DEFAULT):
        self.connection_list = connection_list
        self.address_list = address_list
        print("connection_list")
        print(connection_list)
        for rbt, cnt, addr in zip(self.robots_on_scene, self.connection_list, self.address_list):
            name = rbt[0]
            _type = rbt[1]
            if cnt:
                if _type == RobotType.indy7_robot:
                    if not self.robot_dict[name]:
                        self.robot_dict[name] = indytraj_client(server_ip=addr, robot_name="NRMK-Indy7")
                    with self.robot_dict[name]:
                        self.robot_dict[name].set_collision_level(5)
                        self.robot_dict[name].set_joint_vel_level(self.indy_joint_vel_level)
                        self.robot_dict[name].set_task_vel_level(self.indy_task_vel_level)
                        self.robot_dict[name].set_joint_blend_radius(20)
                        self.robot_dict[name].set_task_blend_radius(0.2)
                elif _type == RobotType.panda_robot:
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

    def get_scene_dict(self):
        return {rp[0]: rp[1] for rp in self.robots_on_scene}

    def get_robot_list(self):
        return [self.robot_dict[name] for name, _type in self.robots_on_scene]

    def get_indexing_list(self):
        return [self.idx_dict[name] for name, _type in self.robots_on_scene]

    def joint_make_sure(self, Q):
        for name, _type in self.robots_on_scene:
            if _type == RobotType.indy7_robot:
                self.robot_dict[name].joint_move_make_sure(np.rad2deg(Q[self.idx_dict[name]]), N_repeat=2, connect=True)
            elif _type == RobotType.panda_robot:
                self.robot_dict[name].move_joint_interpolated(Q[self.idx_dict[name]], N_div=200)

    def grasp_by_dict(self, grasp_dict):
        grasp_seq = [(k, v) for k, v in grasp_dict.items()]
        grasp_seq = list(sorted(grasp_seq, key=lambda x: not x[1]))
        for grasp in grasp_seq:
            self.grasp_fun(grasp[0], grasp[1])

    def grasp_fun(self, name, grasp):
        scence_dict = self.get_scene_dict()
        if scence_dict[name] == RobotType.indy7_robot and self.robot_dict[name] is not None:
            self.robot_dict[name].grasp(grasp, connect=True)
        elif scence_dict[name] == RobotType.panda_robot and self.robot_dict[name] is not None:
            self.robot_dict[name].move_finger(grasp)

    def get_real_robot_pose(self):
        Q = []
        for name, _ in self.robots_on_scene:
            if self.robot_dict[name] is not None:
                Q += list(self.robot_dict[name].get_qcur())
            else:
                Q += list(self.home_pose[self.idx_dict[name]])
        return np.array(Q)

    def wait_step(self, rate_off):
        if all(self.connection_list):
            self.robot_dict[self.robots_on_scene[0][0]].rate_x1.sleep()
        else:
            rate_off.sleep()

    def detect_robots(self, cam):
        self.xyz_rpy_robots = cam.detect_robots(self.robots_on_scene)

    def update_urdf(self):
        xcustom, self.joint_names, self.link_names, self.urdf_content = set_custom_robots(self.robots_on_scene,
                                                                                          self.xyz_rpy_robots,
                                                                                          self.joint_names)
        return xcustom, self.joint_names, self.link_names, self.urdf_content