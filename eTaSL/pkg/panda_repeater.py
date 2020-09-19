from .utils import *
import rospy
from control_msgs.msg import GripperCommandActionGoal
import subprocess

ROBOT_IP = '192.168.0.13'
HOST = '192.168.0.172'
PORT_REPEATER = 1189
CONTROL_RATE_PANDA = 100

class PandaRepeater:
    def __init__(self, host=HOST, port=PORT_REPEATER, robot_ip=ROBOT_IP):
        self.host, self.port, self.robot_ip = host, port, robot_ip
        self.set_alpha_lpf(-1)
        self.set_k_gain(-1)
        self.set_d_gain(-1)
        self.get_qcur()
        self.clear()
        self.start_gripper_server()
        self.rate = rospy.Rate(CONTROL_RATE_PANDA)  # 10hz
        self.finger_pub = rospy.Publisher('/franka_gripper/gripper_action/goal', GripperCommandActionGoal,
                                          tcp_nodelay=True, queue_size=1)
        self.finger_cmd = GripperCommandActionGoal()

    def start_gripper_server(self):
        self.kill_existing_subprocess()
        self.subp = subprocess.Popen(['roslaunch', 'franka_gripper', 'franka_gripper.launch', 'robot_ip:={robot_ip}'.format(robot_ip=self.robot_ip)])


    def kill_existing_subprocess(self):
        if hasattr(self, 'subp') and self.subp is not None:
            self.subp.terminate()
        self.subp = None

    def clear(self):
        self.kill_existing_subprocess()

    def set_alpha_lpf(self, alpha_lpf):
        self.alpha_lpf = send_recv({'alpha_lpf': alpha_lpf}, self.host, self.port)['alpha_lpf']
        return self.alpha_lpf

    def set_k_gain(self, k_gain):
        self.k_gain = send_recv({'k_gain': k_gain}, self.host, self.port)['k_gain']
        return self.k_gain

    def set_d_gain(self, d_gain):
        self.d_gain = send_recv({'d_gain': d_gain}, self.host, self.port)['d_gain']
        return self.d_gain

    def get_qcount(self):
        return send_recv({'qcount': 0}, self.host, self.port)['qcount']

    def get_qcur(self):
        self.qcur = send_recv({'getq': 0}, self.host, self.port)['qval']
        return self.qcur

    def send_qval(self, qval):
        return send_recv({'qval': qval}, self.host, self.port)

    def stop_tracking(self):
        return send_recv({'stop': True}, self.host, self.port)

    def reset(self):
        return send_recv({'reset': True, "period_s": 1.0/CONTROL_RATE_PANDA}, self.host, self.port)

    def move_finger(self, close_bool, max_width=0.039, min_width=0.025, effort=1):
        self.close_bool = close_bool
        self.finger_cmd.goal.command.position = (max_width-min_width)*(1-close_bool)+min_width
        self.finger_cmd.goal.command.max_effort = effort
        self.finger_cmd.header.seq += 1
        self.finger_cmd.goal_id.stamp = self.finger_cmd.header.stamp = rospy.Time.now()
        self.finger_pub.publish(self.finger_cmd)
        return self.close_bool

    def move_joint_interpolated(self, qtar, N_div=100, N_step=None):
        if N_step is None or N_step > N_div:
            N_step = N_div
        qcur = np.array(self.get_qcur())
        DQ = qtar - qcur
        for i_step in range(N_step):
            self.send_qval(qcur + DQ / N_div * i_step)
            self.rate.sleep()

