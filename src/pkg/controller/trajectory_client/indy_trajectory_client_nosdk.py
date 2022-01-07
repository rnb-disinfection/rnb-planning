from .trajectory_client import *
from .indy_utils.indydcp_client import IndyDCPClient
from .indy_utils.indy_program_maker import JsonProgramComponent, \
    POLICY_KEEP_PAUSE, POLICY_RESUME_AFTER, POLICY_STOP_PROGRAM, POLICY_NO_COLLISION_DETECTION

INDY_DOF = 6
INDY_NAME = "NRMK-Indy7"


##
# @class IndyTrajectoryClientNoSDK
# @brief    Trajectory and DCP client for Indy.
# @remark   DCP client to use when IndySDK and Trajectory server is not available.
class IndyTrajectoryClientNoSDK(IndyDCPClient, TrajectoryClient):
    def __init__(self, server_ip, *args, **kwargs):
        kwargs_indy, kwargs_otic = divide_kwargs(kwargs, IndyDCPClient.__init__, TrajectoryClient.__init__)
        if "robot_name" not in kwargs_indy:
            kwargs_indy["robot_name"]="NRMK-Indy7"
        IndyDCPClient.__init__(self, *args, server_ip=server_ip, **kwargs_indy)
        TrajectoryClient.__init__(self, server_ip=self.server_ip, **kwargs_otic)
        self.traj_vel = 1
        self.traj_blend = 3
        self.grasp_ref = True
        self.collision_policy = POLICY_NO_COLLISION_DETECTION
        self.COL_LEVEL = 5
        self.QVEL_LEVEL = 3
        self.TVEL_LEVEL = 1
        self.QBLEND_RAD = 5
        self.TBLEND_RAD = 0.1

    def get_qcount(self):
        # raise(RuntimeError("get_qcount is not supported without indy sdk"))
        with self:
            # print("indy qcount: {}".format(not self.get_robot_status()['movedone']))
            return not self.get_robot_status()['movedone']

    def get_qcur(self):
        with self:
            Q = self.get_joint_pos()
        return np.deg2rad(Q)

    def send_qval(self, qval):
        raise(RuntimeError("send_qval is not supported without indy sdk"))

    def terminate_loop(self):
        raise(RuntimeError("terminate_loop is not supported without indy sdk"))

    ##
    # @brief    Send target pose to the server and store the queue count.
    # @param online If this flag is set True, it will wait the queue on the server to sync the motion.
    def push_Q(self, Q, online=False):
        raise(RuntimeError("push_Q is not supported without indy sdk"))

    ##
    # @brief send off-line s-surve trajectory to move joint to target position
    # @param qtar           target joint configuration
    # @param N_div          the number of divided steps (default=100)
    # @param wait_finish    send trajectory off-line but wait until finish (default=100)
    # @param start_tracking to reset trajectory and start tracking
    # @param auto_stop      auto-stop trajectory-following after finishing the motion
    def move_joint_s_curve(self, qtar, q0=None, N_div=100, wait_finish=True, start_tracking=True, auto_stop=True):
        with self:
            self.joint_move_to(np.rad2deg(qtar))
        if wait_finish:
            time.sleep(0.5)
            with self:
                self.wait_for_move_finish()
        else:
            assert not auto_stop, "Cannot auto-stop trajectory following not waiting end of the motion"

    ##
    # @brief    send s-surve trajectory on-line to move joint to target position.
    #           To test on-line trajectory following for adaptive motion
    # @param qtar         target joint configuration
    # @param N_div          the number of divided steps (default=100)
    # @param start_tracking to reset trajectory and start tracking
    # @param auto_stop      auto-stop trajectory-following after finishing the motion
    def move_joint_s_curve_online(self, qtar, q0=None, N_div=100, auto_stop=True):
        raise(RuntimeError("move_joint_s_curve_online is not supported without indy sdk"))

    ##
    # @param trajectory radian
    # @param vel_lims radian/s, scalar or vector
    # @param acc_lims radian/s2, scalar or vector
    # @return interpolated trajecotry, expected motion time
    def move_joint_wp(self, trajectory, vel_lims, acc_lims, auto_stop=True):
        trajectory = np.concatenate([[self.get_qcur()], trajectory])
        traj_wps = simplify_traj(trajectory, step_fractions=[0, 1])

        with self:
            #         self.joint_waypoint_clean()
            for Q in traj_wps:
                time.sleep(0.5)
                self.wait_for_move_finish()
                self.joint_move_to(np.rad2deg(Q))
                # self.joint_waypoint_append(np.rad2deg(Q))
            #         self.joint_waypoint_execute()
            time.sleep(0.5)
            self.wait_for_move_finish()
        return traj_wps, float(len(traj_wps)) / self.traj_freq

    ##
    # @brief move joint with waypoints, one-by-one
    # @param trajectory numpy array (trajectory length, joint num)
    def move_joint_traj(self, trajectory, auto_stop=True, wait_motion=True):
        Q_init = trajectory[0]
        Q_last = trajectory[-1]
        Q_cur = self.get_qcur()
        # assert np.max(np.abs((np.subtract(Q_init, Q_cur)))) < 5e-2, \
        #     "MOVE robot to trajectory initial: current robot pose does not match with trajectory initial state"

        traj_wps = simplify_traj(trajectory, step_fractions=[0, 1])

        # Joint Move
        with self:
            prog = JsonProgramComponent(policy=self.collision_policy, resume_time=2)
            for Q in traj_wps:
                prog.add_joint_move_to(np.rad2deg(Q).tolist(), vel=self.traj_vel, blend=self.traj_blend)

            prog_json = prog.program_done()
            self.set_and_start_json_program(prog_json)

        if wait_motion:
            time.sleep(0.5)
            with self:
                self.wait_for_move_finish()

    ##
    # @brief Make sure the joints move to Q using the indy DCP joint_move_to function.
    # @param Q radian
    def joint_move_make_sure(self, Q, N_repeat=2):
        with self:
            for _ in range(N_repeat):
                self.joint_move_to(np.rad2deg(Q))
                self.wait_motion()

    ##
    # @brief Surely move joints to Q using the indy DCP joint_move_to function.
    # @param Q radian
    def grasp(self, grasp):
        with self:
            gstate = self.get_endtool_do(0)
            grasp = self.grasp_ref if grasp else not self.grasp_ref
            if gstate != grasp:
                self.set_endtool_do(0, grasp)
                time.sleep(0.5)

    ##
    # @brief Wait for indy DCP motion
    # @param Q radian
    def wait_motion(self, period=1e-1):
        while not self.get_robot_status()['movedone']:
            time.sleep(period)

    ##
    # @brief Wait for specific digital input
    # @param Q radian
    def wait_di(self, idx, period=1e-1):
        while True:
            time.sleep(period)
            if self.get_di()[idx]:
                break

    ##
    # @brief connecting wrapper for indy functions
    # @remark usage: connect_and(fun, *args_for_fun, **kwargs_for_fun)
    def connect_and(self, func, *args, **kwargs):
        with self:
            return func(*args, **kwargs)

    ##
    # @brief override reset_robot in IndyDCPClient. reset robot and wait until resetting is done
    def reset_robot(self):
        IndyDCPClient.reset_robot(self)
        reset_done = False
        while not reset_done:
            robot_state = self.get_robot_status()
            reset_done = all([not robot_state["resetting"], robot_state["ready"],
                              not robot_state["emergency"], not robot_state["error"]])
            time.sleep(0.5)
        self.set_collision_level(self.COL_LEVEL)
        self.set_joint_vel_level(self.QVEL_LEVEL)
        self.set_task_vel_level(self.TVEL_LEVEL)
        self.set_joint_blend_radius(self.QBLEND_RAD)
        self.set_task_blend_radius(self.TBLEND_RAD)

    ##
    # @brief reset robot and trajectory client
    def reset(self):
        with self:
            self.reset_robot()
        return {}


    def start_tracking(self):
        return {}

    ##
    # @brief override stop_tracking in IndyDCPClient. reset the robot and trajectory client, and stop tracking.
    # @remark   reset_robot is added here because it resets the internal robot pose reference.
    #           If reset_robot is not called, it will immediately move to the original reference pose.
    def stop_tracking(self):
        return {}

    ##
    # @brief block entrance that connects to indy dcp server
    # @remark usage: with indy_instance: ~
    def __enter__(self):
        self.connect()
        return self

    ##
    # @brief block exit that disconnects from indy dcp server
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

## do vacuum mode
    # def grasp(self, grasp, depth=0.009):
    #     gstate = self.get_do()[self.indy_grasp_DO]
    #     if gstate != grasp:
    #         self.set_do(self.indy_grasp_DO, int(grasp))
    #         if grasp:
    #             uvw_cur = self.get_task_pos()[3:]
    #             Rbe_cur = Rot_zyx(*np.deg2rad(uvw_cur)[[2, 1, 0]])
    #             time.sleep(0.1)
    #             self.task_move_by(np.matmul(Rbe_cur, [0, 0, depth]).tolist() + [0] * 3)
    #             time.sleep(0.1)
    #             self.wait_motion()
    #             self.task_move_by(np.matmul(Rbe_cur, [0, 0, -depth]).tolist() + [0] * 3)
    #             time.sleep(0.1)
    #             self.wait_motion()

