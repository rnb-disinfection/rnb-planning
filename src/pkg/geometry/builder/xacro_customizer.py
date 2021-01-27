import subprocess
from urdf_parser_py.urdf import URDF

from ...global_config import *
from ..geometry import *
from ...utils.singleton import Singleton
from time import sleep

XACRO_PATH_SRC = '{}src/robots/custom_robots_src.urdf.xacro'.format(RNB_PLANNING_DIR)
XACRO_PATH_DEFAULT = '{}src/robots/custom_robots.urdf.xacro'.format(RNB_PLANNING_DIR)
URDF_PATH_DEFAULT = '{}src/robots/custom_robots.urdf'.format(RNB_PLANNING_DIR)

URDF_PATH = os.path.join(WORKING_DIR, "robots", "custom_robots.urdf")

##
# @class    XacroCustomizer
# @brief    Custom Xacro file generator
# @remark   This generates customized xacro file for ROS functions in
#           XACRO_PATH_DEFAULT = 'src/robots/custom_robots.urdf.xacro' and convert it to urdf.
class XacroCustomizer(Singleton):
    def __init__(self):
        pass

    ##
    # @brief    create xacro file with given information
    # @remark   This generates customized xacro file for ROS robot modeling in
    #           XACRO_PATH_DEFAULT = 'src/robots/custom_robots.urdf.xacro' and convert it to urdf.
    # @param    robots  list of rnb-planning.src.pkg.controller.robot_config.RobotConfig
    # @param    xacro_path by default XACRO_PATH_DEFAULT is used, but you can specify your own path.
    def initialize(self, robots, xacro_path=None):
        if xacro_path is None:
            xacro_path = XACRO_PATH_DEFAULT
        self.xacro_path = xacro_path
        if not hasattr(self, 'subp'): self.subp = None
        self.clear()
        for rbt in robots:
            self.__add_robot(rbt.idx, rbt.type, rbt.xyzrpy[0], rbt.xyzrpy[1])
        self.__write_xacro()

    def __add_robot(self, rid, rtype, xyz=[0,0,0], rpy=[0,0,0]):
        rexpression = \
            '<xacro:{rtype} robot_id="{rid}" xyz="{xyz}" rpy="{rpy}" connected_to="base_link"/>'.format(
                rtype=rtype.name, rid=rid, xyz='{} {} {}'.format(*xyz), rpy='{} {} {}'.format(*rpy)
            )
        self.rexpression_list += [rexpression]
        
    def __write_xacro(self):
        xacro_file = open(XACRO_PATH_SRC, "r")

        new_xacro_content = ""
        toggle_body = False
        for line in xacro_file:
            stripped_line = line.strip()
            if "BODY BEGIN" in stripped_line:
                toggle_body = True
                new_xacro_content += stripped_line +"\n"
            elif "BODY END" in stripped_line:
                toggle_body = False
                for rexpression in self.rexpression_list:
                    new_xacro_content += rexpression +"\n"
                new_xacro_content += stripped_line +"\n"
            elif not toggle_body:
                new_xacro_content += stripped_line +"\n"
        xacro_file.close()

        xacro_file = open(self.xacro_path, "w")
        xacro_file.write(new_xacro_content)
        xacro_file.close()

    ##
    # @brief    convert xacro file to urdf and get urdf content
    # @remark   This generates urdf file for ROS robot modeling in
    #           URDF_PATH_DEFAULT = '{}src/robots/custom_robots.urdf'.format(RNB_PLANNING_DIR)
    # @param    urdf_path URDF_PATH_DEFAULT is used by default, specifying your own value is not recommended
    # @param    joint_limit_dict    joint limit dictionary, in format of {"joint_name":{"upper": rad, "lower": rad, "velocity": rad/s, "effort": rad/s^2}}
    #                               we use effort as acceleration limit here.
    # @param    joint_fix_dict joint list of name strings to fix (ex: finger joints). joints with a name that includes one of these strings will be fixed.
    def convert_xacro_to_urdf(self, urdf_path=URDF_PATH_DEFAULT, joint_limit_dict=None, joint_fix_dict=None):
        if joint_limit_dict is None:
            joint_limit_dict={}
        if joint_fix_dict is None:
            joint_fix_dict={}
        urdf_content = subprocess.check_output(['xacro', self.xacro_path])
        self.urdf_content = URDF.from_xml_string(urdf_content)
        self.urdf_path = urdf_path
        for joint in self.urdf_content.joints:
            if any([jkey in joint.name for jkey in joint_fix_dict.keys()]):
                lim_dir = [v for k,v in joint_fix_dict.items() if k in joint.name][0]
                joint.type='fixed'
                joint.origin.xyz = list(np.add(joint.origin.xyz, 
                                               np.multiply(joint.axis, getattr(joint.limit, lim_dir))
                                              )
                                       )
                joint.axis = None
                joint.limit  = None

            if joint.name in joint_limit_dict.keys():
                jlim = joint_limit_dict[joint.name]
                for key in jlim:
                    setattr(joint.limit, key, jlim[key])
                
        f = open(self.urdf_path, "w")
        f.write(URDF.to_xml_string(self.urdf_content))
        f.close()
        
        self.joint_names = [joint.name for joint in self.urdf_content.joints if joint.type != 'fixed']
        self.link_names = [link.name for link in self.urdf_content.links]
        return self.joint_names, self.link_names, self.urdf_content, self.urdf_path

    ##
    # @brief    start rviz with converted urdf file in URDF_PATH_DEFAULT
    def start_rviz(self):
        self.__kill_existing_subprocess()
        self.subp = subprocess.Popen(['roslaunch', '{}src/launch/gui_custom_robots.launch'.format(RNB_PLANNING_DIR)])
        
    def __kill_existing_subprocess(self):
        if self.subp is not None:
            self.subp.terminate()
        self.subp = None

    ##
    # @brief    shutdown rviz and clear added robots
    def clear(self):
        self.__kill_existing_subprocess()
        self.rexpression_list = []





