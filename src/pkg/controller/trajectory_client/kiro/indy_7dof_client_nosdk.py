from ..indy_trajectory_client_nosdk import *
import kiro_tool


##
# @class Indy7DofClientNoSDK
# @brief    Trajectory client for special 7 Dof indy used in disinfection project
# @remark   Sends 7the joint angle to USB port
class Indy7DofClientNoSDK(IndyTrajectoryClientNoSDK):
    def __init__(self, server_ip, *args, **kwargs):
        IndyTrajectoryClientNoSDK.__init__(self,server_ip, *args, **kwargs)
        self.ktool = kiro_tool.KiroToolPort()
        self.qstack = []
        self.qtool = 0
        self.tool_dt = 0.1

    ##
    # @brief Make sure the joints move to Q using the indy DCP joint_move_to function.
    # @param Q radian
    def joint_move_to(self, Q, N_repeat=2):
        IndyTrajectoryClientNoSDK.joint_move_to(self, Q[:6])
        self.move_tool(Q[-1])

    ##
    # @brief reset robot and trajectory client
    def reset(self):
        self.qstack = []
        return IndyTrajectoryClientNoSDK.reset(self)


    ##
    # @return qcur 7 dof angle
    def get_qcur(self):
        self.qcur = IndyTrajectoryClientNoSDK.get_qcur(self)
        self.qcur = np.concatenate([self.qcur, [self.qtool]])
        return self.qcur

    def get_qcount(self): # tool angle is synced when qcount is taken
        with self:
            moving = not self.get_robot_status()['movedone']
            if moving: # if more than 0 que is left, send first(current) value to tool
                Q6cur = np.deg2rad(self.get_joint_pos())
        if moving and len(len(self.qstack)>0)>0: # if more than 0 que is left, send first(current) value to tool
            diffs = np.sum(np.abs(np.asarray(self.qstack)[:,:6] - Q6cur), axis=-1)
            idx_cur = np.argmin(diffs)
            self.qstack = self.qstack[idx_cur:]
            self.move_tool(self.qstack[0][-1])
        else: # no que left
            if len(self.qstack)>0: # if non-used qstack is left, send last(most recent) value to tool
                self.move_tool(self.qstack[-1][-1])
            self.qstack = []
        return len(self.qstack)

    ##
    # @brief move joint with waypoints, one-by-one
    # @param trajectory numpy array (trajectory length, joint num)
    def move_joint_traj(self, trajectory, auto_stop=True, wait_motion=True):
        for Q in trajectory:
            self.send_qval(Q)

        if not kiro_tool.OFFLINE_MODE:
            IndyTrajectoryClientNoSDK.move_joint_traj(
                self, np.array(trajectory[:,:-1]), auto_stop=auto_stop, wait_motion=False)

        if wait_motion:
            time.sleep(0.1)
            while self.get_qcount():
                time.sleep(self.tool_dt)

    ## for gscene syncronization
    def send_qval(self, qval):
        self.qstack.append(qval)

    def move_tool(self, qtool):
        print("Move Tool: {} ({})".format(np.rad2deg(qtool), np.round(time.time(), 3)))
        self.ktool.send_degree(np.rad2deg(self.qtool)) # send value
        self.qtool = qtool # update saved value
