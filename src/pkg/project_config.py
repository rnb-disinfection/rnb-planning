from global_config import *
from controller.robot_config import *

INDY_IP = '192.168.21.6'

PANDA_ROBOT_IP = '192.168.21.13'
PANDA_REPEATER_IP = '192.168.21.14'

ROBOTS_ON_SCENE = [
    RobotConfig(0, RobotType.indy7, ([-0.44648051261901855, 0.251528263092041, 0.009795188903808594],
                                     [7.722511439072125e-05, 0.012837732857963772, -1.5843305292051728]),
                INDY_IP),
    RobotConfig(1, RobotType.panda, ([0.5117020606994629, 0.16830319166183472, 0.014661192893981934],
                                     [0.0037190383704881766, 0.013066991871646852, -1.6051065831214242]),
                "{}/{}".format(PANDA_REPEATER_IP, PANDA_ROBOT_IP))
    ]