from ..indy_trajectory_client import *
import kiro_tool


##
# @class Indy7DofClient
# @brief    Trajectory client for special 7 Dof indy used in disinfection project
# @remark   Sends 7the joint angle to USB port
class Indy7DofClient(IndyTrajectoryClient):
    def __init__(self, server_ip, *args, **kwargs):
        IndyTrajectoryClient.__init__(self,server_ip, *args, **kwargs)
        self.ktool = kiro_tool.KiroToolPort()
        self.qstack = []
        self.qtool = 0

    ##
    # @brief Make sure the joints move to Q using the indy DCP joint_move_to function.
    # @param Q radian
    def joint_move_make_sure(self, Q, N_repeat=1):
        self.stop_tracking()
        with self:
            for _ in range(N_repeat):
                self.joint_move_to(np.rad2deg(Q[:6]))
                self.move_tool(Q[-1])
                self.wait_motion()

    ##
    # @brief reset robot and trajectory client
    def reset(self):
        self.qstack = []
        return IndyTrajectoryClient.reset(self)


    ##
    # @return qcur 7 dof angle
    def get_qcur(self):
        self.qcur = IndyTrajectoryClient.get_qcur(self)
        self.qcur = np.concatenate([self.qcur, [self.qtool]])
        return self.qcur

    ##
    # @brief stack q values to sync tool angle
    # @param qval 7 dof
    def send_qval(self, qval):
        self.qstack.append(qval)
        return IndyTrajectoryClient.send_qval(self, qval[:6])

    def get_qcount(self): # tool angle is synced when qcount is taken
        qcount= IndyTrajectoryClient.get_qcount(self)
        if qcount > 0: # if more than 0 que is left, send first(current) value to tool
            self.qstack = self.qstack[-qcount:]
            self.move_tool(self.qstack[0][-1])
        else: # no que left
            if len(self.qstack)>0: # if non-used qstack is left, send last(most recent) value to tool
                self.move_tool(self.qstack[-1][-1])
            self.qstack = []
        return qcount

    def move_tool(self, qtool):
#         print("Move Tool: {} ({})".format(np.rad2deg(qtool), np.round(time.time(), 3)))
        self.ktool.send_degree(np.rad2deg(self.qtool)) # send value
        self.qtool = qtool # update saved value
