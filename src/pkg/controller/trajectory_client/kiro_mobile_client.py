from .trajectory_client import *
from ...utils.utils import *
from ...utils.ros_utils import *
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionFeedback, MoveBaseActionResult
from enum import Enum
import time
import subprocess

class MOVE_STATE(Enum):
    PENDING     = 0
    ACTIVE      = 1 # The goal is currently being processed by the action server
    PREEMPTED   = 2 # The goal received a cancel request after it started executing
                    #   and has since completed its execution (Terminal State)
    SUCCEEDED   = 3 # The goal was achieved successfully by the action server (Terminal State)
    ABORTED     = 4 # # The goal was aborted during execution by the action server due
                    #    to some failure (Terminal State)
    REJECTED    = 5 # The goal was rejected by the action server without being processed,
                    #    because the goal was unattainable or invalid (Terminal State)
    PREEMPTING  = 6 # The goal received a cancel request after it started executing
                    #    and has not yet completed execution
    RECALLING   = 7 # The goal received a cancel request before it started executing,
                    #    but the action server has not yet confirmed that the goal is canceled
    RECALLED    = 8 # The goal received a cancel request before it started executing
                    #    and was successfully cancelled (Terminal State)
    LOST        = 9 # An action client can determine that a goal is LOST. This should not be
                    #    sent over the wire by an action server

##
# @class KiroMobileClient
# @brief    Trajectory client for KiroMobileRobot.
# @remark   set server by writing below to ~/.bashrc \n
#           export ROS_MASTER_URI=http://{master-ip}:11311 \n
#           export ROS_HOSTNAME={slave-ip}
class KiroMobileClient(TrajectoryClient):
    ##
    # @param server_ip not required because the server is set as rosmaster
    def __init__(self, server_ip=None, node_name="kmb_controller"):
        TrajectoryClient.__init__(self, server_ip)
        try:
            subprocess.Popen(['roscore'])
            rospy.init_node(node_name, anonymous=True, disable_signals=True)
        except:
            TextColors.YELLOW.println("ros_node already initialized somewhere else")
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pos_listener = Listener(topic_name="/move_base/feedback", topic_type=MoveBaseActionFeedback)
        self.res_listener = Listener(topic_name="/move_base/result", topic_type=MoveBaseActionResult)
        self.goal_msg = PoseStamped()
        self.goal_msg.header.frame_id = '/map'

    def publish_Q(self, Q):
        self.goal_msg.header.seq += 1
        self.goal_msg.header.stamp = rospy.Time.now()
        R = Rot_axis(3, Q[2])
        xyzw = Rotation.from_dcm(R).as_quat()
        self.goal_msg.pose.position.x = Q[0]
        self.goal_msg.pose.position.y = Q[1]
        self.goal_msg.pose.orientation.x = xyzw[0]
        self.goal_msg.pose.orientation.y = xyzw[1]
        self.goal_msg.pose.orientation.z = xyzw[2]
        self.goal_msg.pose.orientation.w = xyzw[3]
        self.goal_pub.publish(self.goal_msg)

    def get_qcount(self):
        if self.res_listener.last_dat is None:
            return True
        else:
            return self.res_listener.last_dat.status.status != MOVE_STATE.SUCCEEDED.value

    def get_qcur(self):
        pose = self.pos_listener.last_dat.feedback.base_position.pose
        Q = np.zeros(6)
        Q[:2] = pose.position.x, pose.position.y
        Q[2] = Rot2axis(Rotation.from_quat((
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)).as_dcm(), 3)
        return Q

    def send_qval(self, qval):
        raise(RuntimeError("send_qval is not supported with KiroMobileClient"))

    def terminate_loop(self):
        raise(RuntimeError("terminate_loop is not supported with KiroMobileClient"))

    ##
    # @brief    Send target pose to the server and store the queue count.
    # @param online If this flag is set True, it will wait the queue on the server to sync the motion.
    def push_Q(self, Q, online=False):
        raise(RuntimeError("push_Q is not supported with KiroMobileClient"))

    ##
    # @brief publish qtar trajectory will be generated on the mobile robot
    def move_joint_s_curve(self, qtar, *args, **kwargs):
        self.joint_move_make_sure(qtar)

    ##
    # @brief    send s-surve trajectory on-line to move joint to target position.
    #           To test on-line trajectory following for adaptive motion
    # @param qtar         target joint configuration
    # @param N_div          the number of divided steps (default=100)
    # @param start_tracking to reset trajectory and start tracking
    # @param auto_stop      auto-stop trajectory-following after finishing the motion
    def move_joint_s_curve_online(self, qtar, q0=None, N_div=100, auto_stop=True):
        raise(RuntimeError("move_joint_s_curve_online is not supported with KiroMobileClient"))

    ##
    # @param trajectory radian
    # @return interpolated trajecotry, expected motion time
    def move_joint_wp(self, trajectory, *args, **kwargs):
        trajectory = np.concatenate([[self.get_qcur()], trajectory])
        traj_wps = simplify_traj(trajectory, step_fractions=[0, 1])

        #         self.joint_waypoint_clean()
        for Q in traj_wps:
            self.joint_move_make_sure(Q)
        return traj_wps, float(len(traj_wps)) / self.traj_freq

    ##
    # @brief move joint with waypoints, one-by-one
    # @param trajectory numpy array (trajectory length, joint num)
    def move_joint_traj(self, trajectory, auto_stop=True, wait_motion=True):
        if not wait_motion:
            TextColors.RED.println("KiroMobileClient always wait for motion")
        self.move_joint_wp(trajectory)


    ##
    # @brief Make sure the joints move to Q using the indy DCP joint_move_to function.
    # @param Q radian
    def joint_move_make_sure(self, Q, *args, **kwargs):
        self.publish_Q(Q)
        time.sleep(0.5)
        self.wait_queue_empty()

    ##
    # @brief Wait until the queue on the server is empty. This also means the trajectory motion is finished.
    def wait_queue_empty(self, max_dur=20):
        time_start = time.time()
        while self.get_qcount()>0 \
                or (self.pos_listener.last_dat.feedback.base_position.header.stamp
                    <= self.res_listener.last_dat.header.stamp):
            time.sleep(1.0/self.traj_freq)
            if (time.time() - time_start) > max_dur:
                break
        # print("pose_stamp: {}".format(self.pos_listener.last_dat.feedback.base_position.header.stamp))
        # print("res_stamp: {}".format(self.res_listener.last_dat.header.stamp))
        # print(self.pos_listener.last_dat.feedback.base_position.pose)

    ##
    # @brief Surely move joints to Q using the indy DCP joint_move_to function.
    # @param Q radian
    def grasp(self, grasp):
        return

    ##
    # @brief reset robot and trajectory client
    def reset(self):
        self.qcount = self.get_qcount()


    def start_tracking(self):
        return {}

    ##
    # @brief override stop_tracking in IndyDCPClient. reset the robot and trajectory client, and stop tracking.
    # @remark   reset_robot is added here because it resets the internal robot pose reference.
    #           If reset_robot is not called, it will immediately move to the original reference pose.
    def stop_tracking(self):
        return {}

##
# @class    MobileRobotDummy
# @brief    Simulator for ros communication with mobile robot
class MobileRobotDummy:
    def __init__(self):
        self.xyz = (0, 0, 0)
        self.xyzw = (0, 0, 0, 1)

        self.rate = rospy.Rate(10)  # 10hz
        self.pos_msg = MoveBaseActionFeedback()
        self.res_msg = MoveBaseActionResult()
        self.pos_msg.header.frame_id = "/map"
        self.res_msg.header.frame_id = "/map"

        self.goal_listener = Listener(topic_name='/move_base_simple/goal', topic_type=PoseStamped,
                                      callback=self.callback_goal)
        self.pos_pub = rospy.Publisher("/move_base/feedback", MoveBaseActionFeedback, queue_size=10)
        self.res_pub = rospy.Publisher("/move_base/result", MoveBaseActionResult, queue_size=10)

    def pub_pos(self):
        self.pos_msg.feedback.base_position.header.frame_id = "/map"
        self.pos_msg.feedback.base_position.header.seq += 1
        self.pos_msg.feedback.base_position.header.stamp = rospy.Time.now()

        self.pos_msg.header.seq += 1
        self.pos_msg.header.stamp = rospy.Time.now()
        self.pos_pub.publish(self.pos_msg)

    def callback_goal(self, msg):
        print("======= got goal msg =======")
        print(msg)
        self.res_msg.header.seq += 1
        self.res_msg.header.stamp = rospy.Time.now()
        self.res_msg.status.status = 1
        self.res_pub.publish(self.res_msg)
        time.sleep(3)
        self.xyz = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.xyzw = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.apply_goal(self.xyz, self.xyzw)
        print("======= goal arrived =======")

    def apply_goal(self, xyz, xyzw):
        self.pos_msg.feedback.base_position.pose.position.x = xyz[0]
        self.pos_msg.feedback.base_position.pose.position.y = xyz[1]
        self.pos_msg.feedback.base_position.pose.position.z = xyz[2]
        self.pos_msg.feedback.base_position.pose.orientation.x = xyzw[0]
        self.pos_msg.feedback.base_position.pose.orientation.y = xyzw[1]
        self.pos_msg.feedback.base_position.pose.orientation.z = xyzw[2]
        self.pos_msg.feedback.base_position.pose.orientation.w = xyzw[3]

        self.res_msg.header.seq += 1
        self.res_msg.header.stamp = rospy.Time.now()
        self.res_msg.status.status = 3
        self.res_pub.publish(self.res_msg)
        print("res_stamp: {}".format(self.res_msg.header.stamp))
        print("pose_stamp: {}".format(self.pos_msg.feedback.base_position.header.stamp))

    def pub_pos_loop(self):
        while True:
            self.pub_pos()
            self.rate.sleep()

# # TEST SCRIPT
# rospy.init_node("mobile_dummy")
# mob = MobileRobotDummy()
# t = Thread(target=mob.pub_pos_loop)
# t.daemon = True
# t.start()