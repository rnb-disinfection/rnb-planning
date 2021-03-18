from .trajectory_client import *
import rospy
from control_msgs.msg import GripperCommandActionGoal
import subprocess


class PandaTrajectoryClient(TrajectoryClient):
    def __init__(self, server_ip, robot_ip, server_port=1189, **kwargs):
        TrajectoryClient.__init__(self, server_ip=server_ip, server_port=server_port, **kwargs)
        self.robot_ip = robot_ip
        self.reset()
        self.clear()
        self.start_gripper_server()
        self.finger_pub = rospy.Publisher('/franka_gripper/gripper_action/goal', GripperCommandActionGoal,
                                          tcp_nodelay=True, queue_size=1)
        self.finger_cmd = GripperCommandActionGoal()
        self.close_bool = False

    def start_gripper_server(self):
        self.__kill_existing_subprocess()
        self.subp = subprocess.Popen(['roslaunch', 'franka_gripper', 'franka_gripper.launch', 'robot_ip:={robot_ip}'.format(robot_ip=self.robot_ip)])


    def __kill_existing_subprocess(self):
        if hasattr(self, 'subp') and self.subp is not None:
            self.subp.terminate()
        self.subp = None

    def clear(self):
        self.__kill_existing_subprocess()

    def grasp(self, close_bool, max_width=0.039, min_width=0.025, effort=1):
        if close_bool != self.close_bool:
            self.finger_cmd.goal.command.position = (max_width-min_width)*(1-close_bool)+min_width
            self.finger_cmd.goal.command.max_effort = effort
            self.finger_cmd.header.seq += 1
            self.finger_cmd.goal_id.stamp = self.finger_cmd.header.stamp = rospy.Time.now()
            self.finger_pub.publish(self.finger_cmd)
            time.sleep(0.5)
        self.close_bool = close_bool
        return self.close_bool

    ##
    # @param Q radian
    def joint_move_make_sure(self, Q):
        self.move_joint_s_curve(Q, N_div=200, auto_stop=True)

    def disconnect(self):
        pass

