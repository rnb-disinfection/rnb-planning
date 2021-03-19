from .trajectory_client import *
from .indy_utils.indydcp_client import IndyDCPClient

INDY_DOF = 6
INDY_NAME = "NRMK-Indy7"


##
# @class IndyTrajectoryClient
# @brief    Trajectory and DCP client for Indy.
# @remark   DCP and Trajectory servers have separate motion command protocols, and they can conflict to each other. \n
#           In principle, push_Q(Q) sends pose to the trajectory server and joint_move_to sends pose to DCP server. \n
#           start_tracking() lets the trajectory server take the charge until stop_tracking() is called. \n
#           Be careful when calling these functions
#           because the pose mismatch between Trajectory/DCP server can cause abrupt step motion.
class IndyTrajectoryClient(IndyDCPClient, TrajectoryClient):
    def __init__(self, server_ip, *args, **kwargs):
        kwargs_indy, kwargs_otic = divide_kwargs(kwargs, IndyDCPClient.__init__, TrajectoryClient.__init__)
        if "robot_name" not in kwargs_indy:
            kwargs_indy["robot_name"]="NRMK-Indy7"
        IndyDCPClient.__init__(self, *args, server_ip=server_ip, **kwargs_indy)
        TrajectoryClient.__init__(self, server_ip=self.server_ip, **kwargs_otic)
        with self:
            self.set_collision_level(5)
            self.set_joint_vel_level(1)
            self.set_task_vel_level(1)
            self.set_joint_blend_radius(5)
            self.set_task_blend_radius(0.1)

    ##
    # @brief Make sure the joints move to Q using the indy DCP joint_move_to function.
    # @param Q radian
    def joint_move_make_sure(self, Q, N_repeat=2):
        self.stop_tracking()
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

    ##
    # @brief reset robot and trajectory client
    def reset(self):
        with self:
            self.reset_robot()
        return TrajectoryClient.reset(self)

    ##
    # @brief override stop_tracking in IndyDCPClient. reset the robot and trajectory client, and stop tracking.
    # @remark   reset_robot is added here because it resets the internal robot pose reference.
    #           If reset_robot is not called, it will immediately move to the original reference pose.
    def stop_tracking(self):
        self.reset()
        return TrajectoryClient.stop_tracking(self)

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

