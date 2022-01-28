from .trajectory_client.trajectory_client import *
from .trajectory_client import indy_trajectory_client
from .trajectory_client import indy_trajectory_client_nosdk
from .trajectory_client import panda_trajectory_client
from .trajectory_client.kiro import kiro_udp_client
from .trajectory_client.kiro import indy_7dof_client
from .trajectory_client.kiro import indy_7dof_client_nosdk
from .robot_config import *
from collections import defaultdict
import numpy as np


JOINT_LIM_DICT = []

##
# @class CombinedRobot
# @brief control interface for combined robot
# @remark call CombinedRobot.simulator.set_gscene(gscene) to automatically visualize robot
class CombinedRobot:
    ##
    # @param robots_on_scene list of rnb-planning.src.pkg.controller.robot_config.RobotConfig
    def __init__(self, robots_on_scene, connection_list, vel_scale=0.5, acc_scale=0.5):
        ## @brief velocity limit scale
        self.vel_scale = vel_scale
        ## @brief acceleration limit scale
        self.acc_scale = acc_scale
        self.__set_robots_on_scene(robots_on_scene)
        self.reset_connection(*connection_list)

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
                if lim_pair is not None:
                    self.custom_limits[jname].update({"lower":lim_pair[0], "upper":lim_pair[1]})
                if vellim is not None:
                    self.custom_limits[jname].update({"velocity": vellim*self.vel_scale})
                if acclim is not None:
                    self.custom_limits[jname].update({"effort": acclim*self.acc_scale})
            self.home_pose += list(RobotSpecs.get_home_pose(_type))
            self.idx_dict[name] = range(i0, len(self.joint_names))
            self.robot_dict[name] = None
            self.robot_names.append(name)
        self.joint_num = len(self.joint_names)
        self.home_pose = np.array(self.home_pose)
        self.home_dict = list2dict(self.home_pose, self.joint_names)
        self.simulator = RobotVisualModel(self,self.home_pose)

    def __set_clients(self):
        for rbt, addr, cnt in zip(self.robots_on_scene, self.address_list, self.connection_list):
            name = rbt.get_indexed_name()
            _type = rbt.type
            if _type in [RobotType.indy7, RobotType.indy7gripper]:
                if "no_sdk" in rbt.specs and rbt.specs["no_sdk"]:
                    self.robot_dict[name] = indy_trajectory_client_nosdk.IndyTrajectoryClientNoSDK(server_ip=addr)
                else:
                    self.robot_dict[name] = indy_trajectory_client.IndyTrajectoryClient(server_ip=addr)
            elif _type == RobotType.indy7kiro:
                if "no_sdk" in rbt.specs and rbt.specs["no_sdk"]:
                    self.robot_dict[name] = indy_7dof_client_nosdk.Indy7DofClientNoSDK(server_ip=addr)
                else:
                    self.robot_dict[name] = indy_7dof_client.Indy7DofClient(server_ip=addr)
            elif _type == RobotType.panda:
                if cnt:
                    self.robot_dict[name] = panda_trajectory_client.PandaTrajectoryClient(*addr.split("/"))
                else:
                    self.robot_dict[name] = panda_trajectory_client.PandaTrajectoryClient(None, None)
            elif _type == RobotType.kmb:
                if addr is not None:
                    self.robot_dict[name] = kiro_udp_client.KiroUDPClient(*addr.split("/"))
                else:
                    self.robot_dict[name] = kiro_udp_client.KiroUDPClient(None, None)
            elif _type == RobotType.pmb:
                self.robot_dict[name] = postech_mobile_client.PostechMobileClient(server_ip=addr)
            else:
                self.robot_dict[name] = TrajectoryClient(server_ip=addr)

    ##
    # @brief update robot position
    # @param name name of robot
    # @param xyzrpy robot position (xyz(m), rpy(rad))
    def update_robot_pos(self, name, xyzrpy):
        if name in self.robot_names:
            self.get_robot_config_dict()[name].xyzrpy = xyzrpy
            self.xyz_rpy_robots[name] = xyzrpy

    ##
    # @brief update robot position by dictionary
    # @param xyz_rpy_robots dictionary of robot positions, {robot name: (xyz(m), rpy(rad))}
    def update_robot_pos_dict(self, xyz_rpy_robots):
        for k,v in xyz_rpy_robots.items():
            self.update_robot_pos(k, v)

    ##
    # @brief reset connection
    # @param connection_list boolean list
    # @param address_list address list, None for default to use stored address
    def reset_connection(self, *args, **kwargs):
        assert np.logical_xor(len(args)>0, len(kwargs)>0), \
            "Give bool connection state for each robot as *args or **kwargs"
        if len(args)>0:
            self.connection_list = args
        else:
            self.connection_list = [kwargs[rname] for rname in self.robot_names]

        print("connection command:")
        for rname, connection in zip(self.robot_names, self.connection_list):
            print("{}: {}".format(rname, connection))

        self.__set_clients()
        self.simulator.set_model()

    ##
    # @brief get a dictionary of rnb-planning.src.pkg.controller.robot_config.RobotConfig on scene
    def get_robot_config_dict(self):
        return {rp.get_indexed_name(): rp for rp in self.robots_on_scene}

    ##
    # @brief get {robot name:base_link}
    def get_robot_base_dict(self):
        return {rname: RobotSpecs.get_base_name(rconfig.type, rname)
                for rname, rconfig in self.get_robot_config_dict().items()}

    ##
    # @brief get {robot name:tip_link}
    def get_robot_tip_dict(self):
        return {rname: RobotSpecs.get_tip_name(rconfig.type, rname)
                for rname, rconfig in self.get_robot_config_dict().items()}

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
    def joint_move_make_sure(self, Q, auto_stop=True):
        for name, rconfig in zip(self.robot_names, self.robots_on_scene):
            _type = rconfig.type
            robot = self.robot_dict[name]
            if robot is not None:
                robot.joint_move_make_sure(Q[self.idx_dict[name]], auto_stop=auto_stop)

    ##
    # @brief move joint with waypoints, one-by-one
    # @param trajectory numpy array (trajectory length, joint num)
    # @error_stop   max. error from the trajectory to stop the robot and return False (degree)
    def move_joint_wp(self, trajectory, vel_scale=None, acc_scale=None, auto_stop=True, wait_motion=True, error_stop=5):
        trajectory = np.array(trajectory)
        vel_scale = vel_scale or self.vel_scale
        acc_scale = acc_scale or self.acc_scale
        robots_in_act = []
        traj_act_all = []
        vel_lims_all = []
        acc_lims_all = []
        traj_freqs = []
        for name, rconfig in zip(self.robot_names, self.robots_on_scene):
            _type = rconfig.type
            robot = self.robot_dict[name]
            if robot is None:
                print("WARNING: {} is not connected - skip motion".format(name))
                continue
            traj_cur_rbt = trajectory[:,self.idx_dict[name]]
            diff_abs_arr = np.abs(traj_cur_rbt - traj_cur_rbt[0:1, :])
            if np.max(diff_abs_arr) > 1e-3:
                traj_cur_rbt = np.concatenate([[robot.get_qcur()], traj_cur_rbt])
                vel_limits = np.multiply(RobotSpecs.get_vel_limits(_type), vel_scale)
                acc_limits = np.multiply(RobotSpecs.get_acc_limits(_type), acc_scale)
                traj_act_all.append(traj_cur_rbt)
                vel_lims_all.append(vel_limits)
                acc_lims_all.append(acc_limits)
                traj_freqs.append(robot.traj_freq)
                robots_in_act.append((name, robot))
        if len(robots_in_act) == 0:
            return 0

        traj_act_all = np.concatenate(traj_act_all, axis=-1)
        vel_lims_all = np.concatenate(vel_lims_all, axis=-1)
        acc_lims_all = np.concatenate(acc_lims_all, axis=-1)
        assert np.all(np.array(traj_freqs) == traj_freqs[0]), "trajectory frequency should be same for all robots"
        traj_freq = np.min(traj_freqs)
        t_all, traj_tot = calc_safe_trajectory(1.0/traj_freq, traj_act_all,
                                               vel_lims=vel_lims_all, acc_lims=acc_lims_all)
        for Q in traj_tot:
            jidx_rbt = 0
            for rname, robot in robots_in_act:
                jnum_rbt = len(self.idx_dict[rname])
                robot.push_Q(Q[jidx_rbt:jidx_rbt+jnum_rbt])
                jidx_rbt = jidx_rbt + jnum_rbt

        for rname, robot in robots_in_act:
            robot.start_tracking()

        if wait_motion:
            done = self.wait_queue_empty(traj_tot, error_stop)

            if auto_stop or not done:
                for rname, robot in robots_in_act:
                    robot.stop_tracking()
            if not done:
                return self.get_real_robot_pose()

        return t_all[-1]

    def start_tracking(self):
        for robot in self.robot_dict.values():
            if robot is not None:
                robot.start_tracking()

    def stop_tracking(self):
        for robot in self.robot_dict.values():
            if robot is not None:
                robot.stop_tracking()

    def get_robots_in_act(self, trajectory, skip_not_connected=True):
        robots_in_act = []
        Q_init = trajectory[0]
        Q_last = trajectory[-1]
        diff_max_all = np.max(np.abs(np.array(trajectory) - trajectory[-1]), axis=0)
        for rname in self.robot_names:
            robot = self.robot_dict[rname]
            if skip_not_connected and robot is None:
                print("WARNING: {} is not connected - skip motion".format(rname))
                continue

            diff_abs_arr = diff_max_all[self.idx_dict[rname]]
            if np.max(diff_abs_arr) > 1e-3:
                robots_in_act.append((rname, robot))
        return robots_in_act

    ##
    # @brief move joint with waypoints, one-by-one
    # @param trajectory numpy array (trajectory length, joint num)
    # @error_stop   max. error from the trajectory to stop the robot and return False (degree)
    def move_joint_traj(self, trajectory, auto_stop=True, wait_motion=True, one_by_one=False, error_stop=10):
        robots_in_act = self.get_robots_in_act(trajectory)

        if len(robots_in_act) == 0:
            return True
        if one_by_one:
            for rname, robot in robots_in_act:
                Q_init = trajectory[0][self.idx_dict[rname]]
                Q_cur = robot.get_qcur()
                if np.max(np.abs((np.subtract(Q_init, Q_cur)))) > 5e-2:
                    print("move_joint_traj: {} pose does not match with trajectory initial state. calling joint_move_make_sure".format(rname))
                    print(np.round(np.rad2deg(Q_init), 1))
                    print(np.round(np.rad2deg(Q_cur), 1))
                    robot.joint_move_make_sure(Q_init)
                    print("joint_move_make_sure done")
                robot.move_joint_traj(trajectory[:, self.idx_dict[rname]], auto_stop=auto_stop, wait_motion=wait_motion)
                if error_stop is not None:
                    if np.sum(np.abs(np.subtract(trajectory[-1, self.idx_dict[rname]], robot.get_qcur())))\
                            >np.deg2rad(error_stop):
                        print("not in sync in {} deg: {}".format(error_stop, np.rad2deg(self.get_real_robot_pose())))
                        return False
        else:
            for Q in trajectory:
                for rname, robot in robots_in_act:
                    robot.push_Q(Q[self.idx_dict[rname]])

            for rname, robot in robots_in_act:
                robot.start_tracking()

            if wait_motion:
                done = self.wait_queue_empty(trajectory, error_stop)

                if auto_stop or not done:
                    for rname, robot in robots_in_act:
                        robot.stop_tracking()
                if not done:
                    return False
        return True

    ##
    # @brief execute grasping action
    # @param args boolean grasp commands for each robot
    # @param kwargs boolean grasp commands for each robot, robot_name=grasp_bool
    def grasp(self, *args, **kwargs):
        if len(args)>0:
            kwargs = {k: v for k,v in zip(self.robot_names, args)}
        else:
            assert kwargs is not None, "Grasp status should be passed either in arguments or keyward arguments "
        grasp_seq = [(k, v) for k, v in kwargs.items()]
        grasp_seq = list(sorted(grasp_seq, key=lambda x: not x[1]))
        for grasp in grasp_seq:
            self.__grasp_fun(grasp[0], grasp[1])

    def __grasp_fun(self, name, grasp):
        if self.robot_dict[name] is not None:
            self.robot_dict[name].grasp(grasp)

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
    # @brief wait for duration (in seconds)
    def wait_step(self, duration):
        time.sleep(duration)

    ##
    # @brief    Wait until the queue on the robots are empty. This also means the trajectory motion is finished.\n
    #           Stop and return False if the deviation from the trajectory gets larget than error_stop.\n
    #           Return True if the loop is finished because the queue is empty.
    # @trajectory   trajectory that the robot is currently following.
    # @error_stop   max. error from the trajectory to stop the robot and return False (degree)
    def wait_queue_empty(self, trajectory=None, error_stop=10):
        if trajectory is not None:
            rpairs = self.get_robots_in_act(trajectory)
            rnames = [rname for rname, robot in rpairs]
            robots = [robot for rname, robot in rpairs]
            robots_mask = sorted(np.concatenate([self.idx_dict[rname] for rname in rnames]))
            while np.sum([robot.get_qcount() for robot in robots]) > 0:
                if error_stop is not None:
                    if (np.min(np.sum(np.abs(np.subtract(trajectory, self.get_real_robot_pose())[:, robots_mask]), axis=1))
                            > np.deg2rad(error_stop)):
                        print("not in sync in {} deg: {}".format(error_stop, np.rad2deg(self.get_real_robot_pose())))
                        return False
                    self.wait_step(0.05)
                else:
                    self.wait_step(0.2)
        return True

    def get_connected_robot_names(self):
        return [rname for rname, connection in zip(self.robot_names, self.connection_list) if connection]

    def get_joint_limits(self):
        return np.concatenate(
                map(lambda x: RobotSpecs.get_joint_limits(x, none_as_inf=True),
                    [self.get_robot_config_dict()[rname].type for rname in self.robot_names])
                )

# @remark call set_gscene(gscene) to automatically visualize robot
class RobotVisualModel:
    def __init__(self, crob, Qhome, gscene=None, simul_speed=10):
        self.gscene, self.crob = gscene, crob
        self.qstack_dict = {rname: [] for rname in crob.robot_names}
        self.Q_cur = np.copy(Qhome)
        self.simul_speed = simul_speed

    def set_gscene(self, gscene):
        self.gscene = gscene

    def set_model(self):
        robot_names = self.crob.robot_names
        for rname in robot_names:
            self.__wrap_client(rname)

    def __wrap_client(self, rname):
        robot = self.crob.robot_dict[rname]
        idx = self.crob.idx_dict[rname]
        qstack = self.qstack_dict[rname]
        connected = rname in self.crob.get_connected_robot_names()

        def reset():
            while qstack:
                qstack.pop()
            if connected:   return type(robot).reset(robot)
            else:           print("[SIMUL] {} reset".format(rname))

        def get_qcount():
            if connected:
                qcount = type(robot).get_qcount(robot)
            else:
                qcount = max(len(qstack) - self.simul_speed, 0)
            while len(qstack) > qcount:
                self.Q_cur[idx] = qstack.pop(0)
            if self.gscene is not None:
                self.gscene.show_pose(self.Q_cur)
            return qcount
        robot.get_qcount = get_qcount

        def get_qcur():
            if connected:   return type(robot).get_qcur(robot)
            else:           return self.Q_cur[idx]
        robot.get_qcur = get_qcur

        def send_qval(qval):
            qstack.append(qval)
            if connected:   return type(robot).send_qval(robot, qval)
            else:           return {'qcount': len(qstack)}
        robot.send_qval = send_qval

        def joint_move_make_sure(Q, *args, **kwargs):
            while qstack:
                qstack.pop()
            if connected:
                type(robot).joint_move_make_sure(robot, Q, *args, **kwargs)
            self.Q_cur[idx] = Q
            if self.gscene is not None:
                self.gscene.show_pose(self.Q_cur)
        robot.joint_move_make_sure = joint_move_make_sure

        def start_tracking():
            if connected:   return type(robot).start_tracking(robot)
            else:           print("[SIMUL] {} start tracking".format(rname))
        robot.start_tracking = start_tracking

        def stop_tracking():
            if connected:   return type(robot).stop_tracking(robot)
            else:           print("[SIMUL] {} stop tracking".format(rname))
        robot.stop_tracking = stop_tracking

        def terminate_loop():
            if connected:   return type(robot).terminate_loop(robot)
            else:           print("[SIMUL] {} terminated".format(rname))
        robot.terminate_loop = terminate_loop

        if not connected:
            def _grasp(grasp):
                print("[SIMUL] {} grasp {}".format(rname, grasp))
            robot.grasp = _grasp