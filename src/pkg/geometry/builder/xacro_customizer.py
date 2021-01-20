import subprocess
from urdf_parser_py.urdf import URDF

from ...global_config import *
from ..geometry import *
from ...utils.singleton import Singleton
from time import sleep

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
        xacro_file = open(self.xacro_path, "r")

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
                
        f = open(urdf_path, "w")
        f.write(URDF.to_xml_string(self.urdf_content))
        f.close()
        
        self.joint_names = [joint.name for joint in self.urdf_content.joints if joint.type != 'fixed']
        self.link_names = [link.name for link in self.urdf_content.links]
        return self.joint_names, self.link_names, self.urdf_content

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




from xml.dom import minidom
from ...utils.joint_utils import *
import os
import urdf_parser_py

def __get_chain(link_name_cur, urdf_content, base_link=None):
    chain = []
    while link_name_cur != base_link and link_name_cur in urdf_content.parent_map:
        parent_joint = get_parent_joint(link_name_cur, urdf_content)
        chain = [(parent_joint, link_name_cur)] + chain
        link_name_cur = urdf_content.joint_map[parent_joint].parent
    return chain

## @brief write srdf for a joint group - for use in moveit motion planning
def write_srdf(robot_names, urdf_content, urdf_path, link_names, joint_names,
               binder_links=None, chain_dict=None, base_link="base_link"):
    root = minidom.Document()

    xml = root.createElement('robot')
    xml.setAttribute('name', urdf_content.name)

    for rname in robot_names:
        grp = root.createElement("group")
        grp.setAttribute('name', rname)

        chain = root.createElement("chain")
        if chain_dict is None:
            chain.setAttribute('base_link', [lname for lname in link_names if rname in lname and urdf_content.parent_map[lname][1] == "base_link"][0])
            chain.setAttribute('tip_link', [bl_name for bl_name in binder_links if rname in bl_name][0])
        else:
            chain.setAttribute('base_link', base_link)
            chain.setAttribute('tip_link', chain_dict[rname]['tip_link'])
        grp.appendChild(chain)
        xml.appendChild(grp)

        grp_stat = root.createElement("group_state")
        grp_stat.setAttribute('name', "all-zeros")
        grp_stat.setAttribute('group', rname)

        if chain_dict is None:
            for jname in joint_names:
                if rname in jname:
                    jstat = root.createElement("joint")
                    jstat.setAttribute('name', jname)
                    jstat.setAttribute('value', "0")
                    grp_stat.appendChild(jstat)
        else:
            for jname in chain_dict[rname]['joint_names']:
                jstat = root.createElement("joint")
                jstat.setAttribute('name', jname)
                jstat.setAttribute('value', "0")
                grp_stat.appendChild(jstat)


        xml.appendChild(grp_stat)
    vjoint = root.createElement("virtual_joint")
    vjoint.setAttribute('name', "fixed_base")
    vjoint.setAttribute('type', "fixed")
    vjoint.setAttribute('parent_frame', "world")
    vjoint.setAttribute('child_link', base_link)
    xml.appendChild(vjoint)

    link_adjacency_map, link_adjacency_map_ext = get_link_adjacency_map(urdf_content)
    for idx1 in range(len(link_names)):
        lname1 = link_names[idx1]
        for lname2 in link_names[idx1:]:
            if lname1 == lname2 or lname1 == base_link or lname2 == base_link:
                continue
            if lname2 in link_adjacency_map[lname1]:
                dcol = root.createElement("disable_collisions")
                dcol.setAttribute('link1', lname1)
                dcol.setAttribute('link2', lname2 )
                dcol.setAttribute('reason', 'Adjacent')
                xml.appendChild(dcol)


    root.appendChild(xml)

    xml_str = root.toprettyxml(indent ="\t")

    save_path_file = urdf_path.replace("urdf", "srdf")

    with open(save_path_file, "w") as f:
        f.write(xml_str)
    return save_path_file


## @brief save converted chain - for use in dual motion planning in moveit
def save_converted_chain(urdf_content, urdf_path, robot_new, base_link, end_link):
    urdf_path_new = os.path.join(os.path.dirname(urdf_path),
                                 os.path.basename(urdf_path).split(".")[0] + "_{}.urdf".format(robot_new))
    urdf_content_new = URDF.from_xml_string(URDF.to_xml_string(urdf_content))
    Tinv_joint_next = np.identity(4)
    chain_base = __get_chain(base_link, urdf_content)

    for linkage in reversed(chain_base):
        jname, lname = linkage
        joint = urdf_content_new.joint_map[jname]
        link = urdf_content_new.link_map[lname]
        xyz_bak, rpy_bak = joint.origin.xyz, joint.origin.rpy
        j_xyz, j_rpy = Tinv_joint_next[:3, 3], Rot2rpy(Tinv_joint_next[:3, :3])
        joint.parent, joint.child = joint.child, joint.parent
        joint.origin.xyz = j_xyz.tolist()
        joint.origin.rpy = j_rpy.tolist()

        if joint.limit:
            joint.limit.lower, joint.limit.upper = -joint.limit.upper, -joint.limit.lower
        if joint.safety_controller:
            joint.safety_controller.soft_lower_limit, joint.safety_controller.soft_upper_limit = \
                joint.limit.lower, joint.limit.upper
        for gtem in link.collisions + link.visuals:
            if gtem.origin is None:
                gtem.origin = urdf_parser_py.urdf.Pose([0, 0, 0], [0, 0, 0])
            Tg_new = np.matmul(Tinv_joint_next, SE3(Rot_rpy(gtem.origin.rpy), gtem.origin.xyz))
            gtem.origin.xyz, gtem.origin.rpy = Tg_new[:3, 3].tolist(), Rot2rpy(Tg_new[:3, :3]).tolist()
        T_joint = SE3(Rot_rpy(rpy_bak), xyz_bak)
        Tinv_joint_next = SE3_inv(T_joint)
    urdf_content_new.add_link(urdf_parser_py.urdf.Link(name="stem"))
    urdf_content_new.add_joint(urdf_parser_py.urdf.Joint(
        name="stem_joint_base_link", joint_type="fixed", parent="stem", child=joint.child,
        origin=urdf_parser_py.urdf.Pose(Tinv_joint_next[:3, 3].tolist(), Rot2rpy(Tinv_joint_next[:3, :3]).tolist())))
    joint.child = "stem"

    f = open(urdf_path_new, "w")
    f.write(URDF.to_xml_string(urdf_content_new))
    f.close()
    urdf_content_new = URDF.from_xml_file(urdf_path_new)
    new_chain = __get_chain(end_link, urdf_content_new)
    new_joints = [linkage[0] for linkage in new_chain if
                  linkage[0] and urdf_content_new.joint_map[linkage[0]].type != "fixed"]

    srdf_path_new = write_srdf(robot_names=[robot_new], urdf_content=urdf_content_new, urdf_path=urdf_path_new,
                               link_names=sorted(urdf_content_new.link_map.keys()), joint_names=new_joints,
                               chain_dict={robot_new: {'tip_link': end_link, 'joint_names': new_joints}},
                               base_link=base_link)
    return urdf_content_new, urdf_path_new, srdf_path_new, new_joints