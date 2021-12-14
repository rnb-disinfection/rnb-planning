from ..geometry.geometry import *
from ..utils.utils import list2dict
from ..planning.constraint.constraint_subject import *
from ..planning.constraint.constraint_actor import PlacePlane


##
# @brief add indy_gripper_asm2 mesh and collision boundary for the gripper
# @param gscene     rnb-planning.src.pkg.geometry.geometry.GeometryScene
# @param robot_name full indexed name of the robot
def add_indy_gripper_asm2(gscene, robot_name):
    gscene.create_safe(GEOTYPE.MESH, "{}_gripper_vis".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.1,0.1,0.1), center=(0,0,0), rpy=(0,0,np.pi/2),
                       color=(0.1,0.1,0.1,1), display=True, fixed=True, collision=False,
                       uri="package://my_mesh/meshes/stl/indy_gripper_asm2_res.STL", scale=(1,1,1))

    gscene.create_safe(GEOTYPE.BOX, "{}_gripper".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.06,0.08,0.06), center=(0,0,0.04), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger1".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.03,0.03,0.095), center=(0.006,0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger2".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.03,0.03,0.095), center=(-0.006,0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger3".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.03,0.03,0.095), center=(0.006,-0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_finger4".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.03,0.03,0.095), center=(-0.006,-0.045,0.1), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)
    raise(DeprecationWarning("add_indy_gripper_asm2 is deprecated. Set RobotType to indy7gripper instead."))

##
# @brief add indy_gripper_asm2 mesh and collision boundary for the gripper, for neuromeka tech value eval demo
# @param gscene     rnb-planning.src.pkg.geometry.geometry.GeometryScene
# @param robot_name full indexed name of the robot
def add_indy_gripper_asm3(gscene, robot_name):
    gscene.create_safe(GEOTYPE.MESH, "{}_gripper_vis".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.1,0.1,0.1), center=(0,0,0), rpy=(0,0,np.pi/2),
                       color=(0.1,0.1,0.1,1), display=True, fixed=True, collision=False,
                       uri="package://my_mesh/meshes/stl/indy_gripper_asm3_res.STL", scale=(1,1,1))

    gscene.create_safe(GEOTYPE.BOX, "{}_gripper".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.06,0.08,0.06), center=(0,0,0.04), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.BOX, "{}_finger1".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.044, 0.034, 0.07), center=(0, 0.045, 0.085), rpy=(0, 0, 0),
                       color=(0.0, 0.8, 0.0, 0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.BOX, "{}_finger2".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.044, 0.034, 0.07), center=(0, -0.045, 0.085), rpy=(0, 0, 0),
                       color=(0.0, 0.8, 0.0, 0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_fingertip1".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.044, 0.044, 0.034), center=(0, 0.045, 0.12), rpy=(np.pi / 2, 0, 0),
                       color=(0.0, 0.8, 0.0, 0.5), display=True, fixed=True, collision=True)

    gscene.create_safe(GEOTYPE.CYLINDER, "{}_fingertip2".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.044, 0.044, 0.034), center=(0, -0.045, 0.12), rpy=(np.pi / 2, 0, 0),
                       color=(0.0, 0.8, 0.0, 0.5), display=True, fixed=True, collision=True)
    gscene.create_safe(GEOTYPE.BOX, "{}_plug".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.05,0.1,0.05), center=(0,0.07,-0.03), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)
    gscene.create_safe(GEOTYPE.BOX, "{}_plug2".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.05,0.08,0.05), center=(0,0.07,0.01), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)


##
# @brief add add_sweep_tool to indy
# @param gscene     rnb-planning.src.pkg.geometry.geometry.GeometryScene
# @param robot_name full indexed name of the robot
def add_indy_sweep_tool(gscene, robot_name, face_name="brush_face", tool_offset=0.12):
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="{}_fts".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.02), dims=(0.07, 0.07, 0.04), rpy=(0, 0, 0), color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="{}_fts_col".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.02), dims=(0.11, 0.11, 0.04), rpy=(0, 0, 0), color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="{}_pole".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.071), dims=(0.03, 0.03, 0.062), rpy=(0, 0, 0), color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="{}_pole_col".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       center=(0, 0, 0.071), dims=(0.07, 0.07, 0.062), rpy=(0, 0, 0), color=(0.0, 0.8, 0.0, 0.2),
                       collision=True, fixed=True)

    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_brushbase".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       center=(0, 0, tool_offset-0.025), dims=(0.06, 0.14, 0.02), rpy=(0, 0, 0), color=(0.8, 0.8, 0.8, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name=face_name, link_name="{}_tcp".format(robot_name),
                       center=(0, 0, tool_offset-0.005), dims=(0.05, 0.13, 0.02), rpy=(0, 0, 0), color=(1.0, 1.0, 0.94, 1),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_col".format(face_name), link_name="{}_tcp".format(robot_name),
                       center=(0, 0, tool_offset-0.015), dims=(0.08, 0.15, 0.03), rpy=(0, 0, 0), color=(0.0, 0.8, 0.0, 0.5),
                       collision=True, fixed=True)
    gscene.create_safe(GEOTYPE.BOX, "{}_plug".format(robot_name),
                       link_name="{}_tcp".format(robot_name),
                       dims=(0.05,0.1,0.05), center=(0,0.07,-0.03), rpy=(0,0,0),
                       color=(0.0,0.8,0.0,0.5), display=True, fixed=True, collision=True)


from pkg.planning.constraint.constraint_subject import Grasp2Point, FramePoint, SweepFrame, SweepLineTask, CustomObject
from pkg.planning.constraint.constraint_actor import Gripper2Tool, PlaceFrame, SweepFramer, AttachFramer
from pkg.planning.constraint.constraint_actor import KnobFramer, HingeFramer, FramedTool
from pkg.planning.constraint.constraint_subject import KnobFrame, HingeFrame, HingeTask

def add_drawer(pscene, dname="drawer", draw_len=0.2,
               center=(1, 1, 1), rpy=(0, 0, 0), dims=(0.4, 0.4, 0.2),
               wall_thickness=0.02, handle_size=(0.05, 0.1, 0.02),
               color=(0.8, 0.8, 0.8, 1), link_name="base_link", clearance=2e-3):
    gscene = pscene.gscene
    botname = "{}_bot".format(dname)
    drawer_bot = gscene.create_safe(GEOTYPE.BOX, botname, link_name=link_name,
                                    center=center, rpy=rpy,
                                    dims=(dims[0] + wall_thickness, dims[1] + wall_thickness, wall_thickness),
                                    fixed=True, collision=True, color=color)
    gscene.create_safe(GEOTYPE.BOX, "{}_ceil".format(dname), link_name=link_name,
                       center=(0, 0, dims[2]),
                       dims=(dims[0] + wall_thickness, dims[1] + wall_thickness, wall_thickness),
                       fixed=True, collision=True, color=color, parent=botname)
    gscene.create_safe(GEOTYPE.BOX, "{}_back".format(dname), link_name=link_name,
                       center=(dims[0] / 2, 0, dims[2] / 2),
                       dims=(wall_thickness, dims[1] + wall_thickness, dims[2] + wall_thickness),
                       fixed=True, collision=True, color=color, parent=botname)
    gscene.create_safe(GEOTYPE.BOX, "{}_left".format(dname), link_name=link_name,
                       center=(0, -dims[1] / 2, dims[2] / 2),
                       dims=(dims[0] + wall_thickness, wall_thickness, dims[2] + wall_thickness),
                       fixed=True, collision=True, color=color, parent=botname)
    gscene.create_safe(GEOTYPE.BOX, "{}_right".format(dname), link_name=link_name,
                       center=(0, dims[1] / 2, dims[2] / 2),
                       dims=(dims[0] + wall_thickness, wall_thickness, dims[2] + wall_thickness),
                       fixed=True, collision=True, color=color, parent=botname)
    Q0 = [0] * gscene.joint_num
    Tbb = drawer_bot.get_tf(Q0)
    center = np.matmul(Tbb, SE3(np.identity(3), (-wall_thickness - clearance / 2, 0, dims[2] / 2)))[:3, 3]
    drawer = gscene.create_safe(GEOTYPE.BOX, dname, link_name=link_name,
                                center=center, dims=np.subtract(dims, wall_thickness + clearance),
                                fixed=False, collision=True, color=color)

    pplane0 = pscene.create_binder(bname=drawer_bot.name + "_0", gname=drawer_bot.name, _type=PlaceFrame,
                                   point=((drawer_bot.dims[0] - wall_thickness) / 2, 0, wall_thickness / 2))
    pplane1 = pscene.create_binder(bname=drawer_bot.name + "_1", gname=drawer_bot.name, _type=PlaceFrame,
                                   point=((drawer_bot.dims[0] - wall_thickness) / 2 - draw_len, 0, wall_thickness / 2))

    handle = gscene.create_safe(GEOTYPE.BOX, "{}_handle".format(dname), link_name=link_name,
                                center=(-dims[0] / 2 + wall_thickness / 2 - handle_size[0] / 2, 0, 0), dims=handle_size,
                                fixed=False, collision=True, color=color, parent=drawer.name)

    Tbp0, Tbh = pplane0.get_tf_handle(Q0), handle.get_tf(Q0)
    Thp0 = np.matmul(SE3_inv(Tbh), Tbp0)
    gpoint = Grasp2Point("gp", handle, (0, 0, 0), (0, 0, np.pi / 2))
    ppoint = FramePoint("pp", handle, Thp0[:3, 3], Rot2rpy(Thp0[:3, :3]))
    handle_s = pscene.create_subject(oname="handle", gname=handle.name, _type=CustomObject,
                                     action_points_dict={gpoint.name: gpoint, ppoint.name: ppoint})
    drawer = pscene.create_binder(bname=handle.name, gname=handle.name, _type=SweepFramer, point=(0, 0, 0),
                                  rpy=(0, 0, 0))
    Tbd = drawer.get_tf_handle(Q0)
    Tdd = np.matmul(SE3_inv(Tbb), Tbd)
    gp1 = SweepFrame("h1", drawer_bot, Tdd[:3, 3], Rot2rpy(Tdd[:3, :3]))
    gp2 = SweepFrame("h2", drawer_bot, Tdd[:3, 3] - (draw_len, 0, 0), Rot2rpy(Tdd[:3, :3]))

    sweep = pscene.create_subject(oname=dname, gname=drawer_bot.name, _type=SweepLineTask,
                                  action_points_dict={gp1.name: gp1,
                                                      gp2.name: gp2}, one_direction=False)
    return drawer_bot, drawer, handle


def add_lever(pscene, knob, door_s=None, lname="lever", lever_ang=np.pi / 6, knob_offset=(0.09), dims=(0.02, 0.2, 0.02),
              link_name="base_link", color=(0.8, 0.8, 0.8, 1), clearance=1e-3):
    gscene = pscene.gscene
    Q0 = [0] * gscene.joint_num
    Tbk = knob.get_tf(Q0)
    Tbkp = np.matmul(Tbk, SE3(np.identity(3), (0, 0, knob.dims[2] / 2 + clearance)))
    Tbl = np.matmul(Tbkp, SE3(np.identity(3), (0, knob_offset, dims[2] / 2)))
    lever = gscene.create_safe(GEOTYPE.BOX, lname, link_name=link_name,
                               center=Tbl[:3, 3], rpy=Rot2rpy(Tbl[:3, :3]), dims=dims,
                               fixed=False, collision=True, color=color)
    lgrasp = Grasp2Point("gp", lever, (0, 0, 0), (np.pi / 2, 0, -np.pi / 2))
    lattach = FramePoint("cp", lever, (0, -knob_offset, -dims[2] / 2), (0, 0, 0), key=lname)

    lever_plug = pscene.create_binder(bname=lname + "_plug", gname=lname, _type=KnobFramer,
                                      point=(0, -knob_offset, -dims[2] / 2), key=knob.name)

    if door_s is not None:
        knob_plug = pscene.create_binder(bname=knob.name + "_plug", gname=lname,
                                         _type=FramedTool, point=(0, -knob_offset, -dims[2] / 2), rpy=(0, 0, 0),
                                         key=knob.name)
        knob_plug.available = False
        door_s.action_points_dict[knob.name] = FramePoint(knob.name, knob, (0, 0, knob.dims[2] / 2 + clearance),
                                                          (0, 0, lever_ang), key=knob.name)

    lever_s = pscene.create_subject(oname=lname, gname=lname, _type=CustomObject,
                                    action_points_dict={lgrasp.name: lgrasp, lattach.name: lattach},
                                    sub_binders_dict={lever_plug.name: lever_plug,
                                                      knob_plug.name: knob_plug})

    bd1 = (KnobFrame("r1", knob, (0, 0, knob.dims[2] / 2 + clearance), (0, 0, 0), key=knob.name),
           pscene.create_binder(bname=knob.name + "_1", gname=knob.name, _type=AttachFramer,
                                point=(0, 0, knob.dims[2] / 2 + clearance), key=lname))
    bd2 = (KnobFrame("r2", knob, (0, 0, knob.dims[2] / 2 + clearance), rpy=(0, 0, lever_ang), key=knob.name),
           pscene.create_binder(bname=knob.name + "_2", gname=knob.name, _type=AttachFramer,
                                point=(0, 0, knob.dims[2] / 2 + clearance), rpy=(0, 0, lever_ang), key=lname)
           )

    knob_s = pscene.create_subject(oname=knob.name, gname=knob.name, _type=KnobTask,
                                   binding_pairs=[bd1, bd2], knob_plug=knob_plug)
    return lever_s, knob_s


def add_door(pscene, dname="door", center=(0.5, 0, 0.5), rpy=(0, 0, 0),
             dims=(0.05, 0.4, 0.5), hinge_point=(-0.025, 0.2, 0),
             door_ang=np.pi * 3 / 4, door_div=3, link_name="base_link", clearance=1e-3,
             frame_depth=0.0, frame_thickness=0.01, color=(0.8, 0.8, 0.8, 1), add_frame=False):
    gscene = pscene.gscene
    door_frame = gscene.create_safe(GEOTYPE.BOX, dname + "_frame", link_name=link_name,
                                    center=center, rpy=Rot2rpy(np.matmul(Rot_rpy(rpy), Rot_axis(2, np.pi / 2))),
                                    dims=np.asarray(dims)[[2, 1, 0]],
                                    fixed=True, collision=False, color=(1, 0, 0, 0.1), display=True)
    door = gscene.create_safe(GEOTYPE.BOX, dname, link_name=link_name,
                              center=center, rpy=rpy, dims=dims,
                              fixed=False, collision=True, color=color)
    if add_frame:
        gscene.add_virtual_guardrail(door_frame, axis="xy", color=color, THICKNESS=frame_thickness,
                                     HEIGHT=dims[0] / 2 + frame_depth, margin=frame_thickness / 2 + clearance)

    hinge_p = FramePoint("hinge", door, hinge_point, (0, 0, 0), key=dname)

    door_hinge = pscene.create_binder(bname=door.name + "_hinge", gname=door.name, _type=HingeFramer,
                                      point=hinge_point, key=dname)

    door_s = pscene.create_subject(oname=dname, gname=dname, _type=CustomObject,
                                   action_points_dict={hinge_p.name: hinge_p},
                                   sub_binders_dict={door_hinge.name: door_hinge})

    hinge_bindings = []
    for i_div in range(door_div + 1):
        ang = door_ang / door_div * i_div
        hp = HingeFrame("h{}".format(i_div), door_frame,
                        np.asarray(hinge_point)[[2, 1, 0]],
                        Rot2rpy(np.matmul(Rot_axis(2, np.pi / 2).transpose(),
                                          Rot_axis(3, -ang))),
                        key=dname)
        bp = pscene.create_binder(bname=door_frame.name + "_{}".format(i_div),
                                  gname=door_frame.name, _type=AttachFramer,
                                  point=np.asarray(hinge_point)[[2, 1, 0]],
                                  rpy=Rot2rpy(np.matmul(Rot_axis(2, np.pi / 2).transpose(),
                                                        Rot_axis(3, -ang))),
                                  key=dname)
        hinge_bindings.append((hp, bp))

    hinge_s = pscene.create_subject(oname=door_frame.name, gname=door_frame.name, _type=HingeTask,
                                    binding_pairs=hinge_bindings)
    return door_s, hinge_s

def finish_L_shape(gscene, gtem_dict):
    if "l_shape" in gtem_dict:
        l_center = gtem_dict["l_shape"]
        gscene.create_safe(gtype=GEOTYPE.BOX, name="l_shape_x", link_name=l_center.link_name,
                           center=(l_center.dims[0], 0.0, 0.0), dims=l_center.dims, rpy=(0, 0, 0), color=l_center.color,
                           collision=l_center.collision, fixed=l_center.fixed, parent="l_shape")
        gscene.create_safe(gtype=GEOTYPE.BOX, name="l_shape_z", link_name=l_center.link_name,
                           center=(0.0, 0.0, l_center.dims[2]), dims=l_center.dims, rpy=(0, 0, 0), color=l_center.color,
                           collision=l_center.collision, fixed=l_center.fixed, parent="l_shape")


##
# @brief remove place points except for the current one
def use_current_place_point_only(pscene, current_state):
    for btf in current_state.binding_state.values():
        oname, aname, bname, bgname = btf.get_chain()
        obj = pscene.subject_dict[oname]
        if isinstance(obj, AbstractObject):
            pp_list = [ap for ap in obj.action_points_dict.values() if isinstance(ap, PlacePoint)]
            for pp in pp_list:
                if pp.name != aname:
                    del obj.action_points_dict[pp.name]
    pscene.update_subjects()


##
# @brief remove attached binders on objects except for the current one
def use_current_sub_binders_only(pscene, current_state):
    active_binders  = [btf.chain.actor_name
                       for btf in current_state.binding_state.values()
                       if btf.chain.actor_name is not None]

    for obj in pscene.subject_dict.values():
        for bname, binder in pscene.actor_dict.items():
            if binder.geometry.name in obj.geometry.get_family() and bname not in active_binders:
                pscene.remove_binder(bname)

##
# @brief set action points for l_shape object
def set_l_shape_object(pscene):
    if "l_shape" in pscene.subject_dict:
        l_sub = pscene.subject_dict["l_shape"]
        l_sub.action_points_dict = {}
        l_sub.add_place_points(l_sub.geometry)
        for gname in l_sub.geometry.children:
            child = pscene.gscene.NAME_DICT[gname]
            if child.collision and child.display:
                l_sub.add_grip_points(child, GRASP_DEPTH_MIN=0.015)
                l_sub.register_binders(pscene, PlacePlane, geometry=child)
        l_sub.set_conflict_dict()

import requests
from bs4 import BeautifulSoup
from time import sleep

def switch_command(ip_addr, on_off, UI_PORT=9990):
    uri = "http://{ip_addr}:{UI_PORT}/param_setting?control_force0={on_off}".format(ip_addr=ip_addr, UI_PORT=UI_PORT, on_off=int(on_off))
    print(uri)
    requests.get(uri)

def start_force_mode(indy, switch_delay=0.5):
    sleep(switch_delay)
    indy.reset()
    switch_command(indy.server_ip, True)
    sleep(switch_delay)

def stop_force_mode(indy, Qref, switch_delay=0.5):
    sleep(switch_delay)
    indy.send_qval(indy.get_qcur())
    indy.start_tracking()
    switch_command(indy.server_ip, False)
    indy.move_joint_s_curve(Qref, N_div=20, start_tracking=False, auto_stop=False)

from ..planning.mode_switcher import ModeSwitcherTemplate, CombinedSwitcher, GraspModeSwitcher

class ForceOnlyModeSwitcher(ModeSwitcherTemplate):
    def __init__(self, pscene, switch_delay=0.5, log_force=False, DT=1.0 / 2e3):
        ModeSwitcherTemplate.__init__(self, pscene, switch_delay=switch_delay)
        self.DT = DT
        self.log_force = log_force
        self.force_log = []

    def switch_in(self, snode_pre, snode_new):
        switch_state = False
        for n1, n2 in zip(snode_pre.state.node, snode_new.state.node):
            if n1 == 1 and n2 == 2:
                switch_state = True
                break
        if switch_state:
            indy = self.crob.robot_dict['indy0']
            if indy is not None:
                start_force_mode(indy, switch_delay=self.switch_delay)
        return switch_state

    def switch_out(self, switch_state, snode_new):
        if switch_state:
            indy = self.crob.robot_dict['indy0']
            if indy is not None:
                if self.log_force:
                    sleep(1)
                stop_force_mode(indy, Qref=snode_new.traj[-1][self.crob.idx_dict['indy0']],
                                switch_delay=self.switch_delay)
                if self.log_force:
                    sleep(self.switch_delay)
                    Fext = down_force_log(indy.server_ip, len(self.crob.idx_dict["indy0"]), DT=self.DT)
                    self.force_log.append(Fext)

class ForceModeSwitcher(CombinedSwitcher):
    def __init__(self, pscene, switch_delay=0.5, log_force=False):
        self.switch_list = [GraspModeSwitcher(pscene, switch_delay=switch_delay),
                            ForceOnlyModeSwitcher(pscene, switch_delay=switch_delay,log_force=log_force)]

    def reset_log(self):
        self.switch_list[1].force_log = []

    def get_log(self):
        return self.switch_list[1].force_log


import matplotlib.pyplot as plt

def down_force_log(ip_addr, JOINT_DOF, UI_PORT=9990, DT=1.0 / 2e3):
    uri = "http://{ip_addr}:{UI_PORT}/download_log".format(ip_addr=ip_addr, UI_PORT=UI_PORT)
    print(uri)
    log_dat = requests.get(uri)
    dat = log_dat.text
    lines = dat.split("\n")
    heads = lines[0].split(",")[:-1]
    data_mat = []
    for line in lines[1:]:
        data_line = list(map(float, line.split(",")[:-1]))
        if len(data_line) > 0:
            data_mat.append(data_line)
    data_mat = np.array(data_mat)
    Fext = data_mat[:, JOINT_DOF * 5:JOINT_DOF * 6]
    Fext = Fext[-int(15.0 / DT):]
    # idx_peak = np.argmax(Fext[:, 2])
    # print("peak: {}".format(np.round(Fext[idx_peak, 2], 1)))
    # Fext = Fext[idx_peak + int(1.0 / DT):idx_peak + int(4.0 / DT), 2]
    # print("force min/max: {} / {} in {}".format(np.round(np.min(Fext), 1), np.round(np.max(Fext), 1), len(Fext)))
    return Fext

def play_schedule_clearance_highlight(ppline, snode_schedule, tcheck, period, actor_name='brush_face',
                                      color_on=(0,1,0,0.3), color_off=(0.8,0.2,0.2,0.2)):
    snode_pre = snode_schedule[0]

    tcheck_res = False
    for snode in snode_schedule:
        ppline.pscene.set_object_state(snode_pre.state)
        ppline.pscene.gscene.update_markers_all()
        if snode.traj is None or len(snode.traj) == 0:
            snode_pre = snode
            continue
        ppline.pscene.gscene.clear_highlight()

        for sname in sname in ppline.pscene.subject_name_list:
            btf_pre, btf = snode_pre.state.binding_state[sname], snode.state.binding_state[sname]
            if not btf_pre.get_chain() == btf.get_chain():
                ppline.pscene.show_binding(btf)
        if period < 0.01:
            ppline.pscene.gscene.show_motion(snode.traj[::int(0.01 / period)], period=0.01)
        else:
            ppline.pscene.gscene.show_motion(snode.traj, period=period)
        sleep(period)
        ppline.pscene.gscene.show_pose(snode.traj[-1])
        snode_pre = snode
        for obj_name in ppline.pscene.subject_name_list:
            if isinstance(ppline.pscene.subject_dict[obj_name], AbstractTask):
                obj = ppline.pscene.subject_dict[obj_name]
                btf = snode.state.binding_state[obj_name]
                assert btf.chain.actor_name == actor_name, "someting wrong: actor name mismatch with BindingTransfrom"
                tcheck_res = tcheck.check(btf,
                                          list2dict(snode.state.Q, ppline.pscene.gscene.joint_names))
                for gtem in obj.clearance:
                    if tcheck_res:
                        gtem.color = color_on
                    else:
                        gtem.color = color_off
                    ppline.pscene.gscene.update_marker(gtem)

def play_schedule_clearance_highlight_full(ppline, snode_schedule_all, tcheck):
    actor_name='brush_face'
    color_off=(0.8,0.2,0.2,0.2)
    for obj_name in ppline.pscene.subject_name_list:
        if isinstance(ppline.pscene.subject_dict[obj_name], AbstractTask):
            actor, obj = ppline.pscene.actor_dict[actor_name], ppline.pscene.subject_dict[obj_name]
            for gtem in obj.clearance:
                gtem.color = color_off
                ppline.pscene.gscene.update_marker(gtem)
                ppline.pscene.set_object_state(snode_schedule_all[0][0].state)
    ppline.pscene.gscene.update_markers_all()
    sleep(1)
    for snode_schedule in snode_schedule_all:
    #     ppline.play_schedule(snode_schedule, period=0.001)
        play_schedule_clearance_highlight(ppline, snode_schedule, tcheck=tcheck, period=0.001)

def double_sweep_motions(snode_schedule):
    snode_pre = None
    for snode in snode_schedule:
        if snode_pre is not None:
            diff_list = [ntem_pre==1 and ntem==2
                         for ntem_pre, ntem in zip(snode_pre.state.node, snode.state.node)
                         if ntem_pre != ntem]
            if len(diff_list)==1 and diff_list[0]:
                traj_list = list(snode.traj)
                snode.traj = np.array(traj_list + list(reversed(traj_list))+traj_list)
        snode_pre = snode

from .joint_utils import *
from .gjk import get_point_list, get_gjk_distance

##
# @brief check collision with other geometries in a specific link
# @param gcheck GraspChecker
# @param geomtry GeometryItem
# @param Q_dict current joint configuration
# @param link_name name of link of interest, if not specified, the geometry's link is used
# @param margin minimum margin to be collision-free
def check_geometry_collision(gcheck, geometry, Q_dict, link_name=None, margin=1e-4):
    if link_name is None:
        link_name = geometry.link_name
    geo_family = geometry.get_family()
    gscene = gcheck.gscene
    object_geo_list = [geometry]
    T_dict = get_T_dict_foward(link_name, [link_name], Q_dict, gscene.urdf_content)

    object_vertice_list = []
    for obj_geo in object_geo_list:
        T = np.matmul(T_dict[obj_geo.link_name], obj_geo.Toff)
        verts, radius = obj_geo.get_vertice_radius()
        verts = np.matmul(verts, T[:3 ,:3].transpose() ) +T[:3 ,3]
        vert_point_list = get_point_list(verts)
        object_vertice_list.append((vert_point_list, radius))

    clearance_geo_list = [gtem for gtem in gscene
                          if gtem.link_name == link_name
                          and gtem.collision
                          and gtem.name not in geo_family]

    clearance_vertice_list = []
    for gtem_clr in clearance_geo_list:
        T = np.matmul(T_dict[gtem_clr.link_name], gtem_clr.Toff)
        verts, radius = gtem_clr.get_vertice_radius()
        verts = np.matmul(verts, T[:3 ,:3].transpose() ) +T[:3 ,3]
        vert_point_list = get_point_list(verts)
        clearance_vertice_list.append((vert_point_list, radius))

    dist_list = []
    for clear_vertice, clear_radius in clearance_vertice_list:
        for object_vertice, object_radius in object_vertice_list:
            dist_list.append(get_gjk_distance(clear_vertice, object_vertice) - clear_radius - object_radius)

    if len(dist_list)>0:
        res = np.min(dist_list) > + margin
    else:
        res = True
    return res


def move_objects_down_until_collision(obj_list, gcheck, Q_dict):
    for obj in obj_list:
        if isinstance(obj, AbstractObject):
            geometry = obj.geometry
        else:
            geometry = obj
            while geometry.parent is not None:
                geometry = geometry.gscene.NAME_DICT[geometry.parent]

        while(check_geometry_collision(gcheck, geometry, Q_dict=Q_dict)):
            if isinstance(obj, AbstractObject):
                obj.set_state(obj.binding, None)
            else:
                Toff = np.matmul(SE3(np.identity(3), [0, 0, -1e-3]), geometry.Toff)
                geometry.set_offset_tf(center=Toff[:3,3], orientation_mat=Toff[:3,:3])

def move_objects_up_until_no_collision(obj_list, gcheck, Q_dict):
    for obj in obj_list:
        if isinstance(obj, AbstractObject):
            geometry = obj.geometry
        else:
            geometry = obj
            while geometry.parent is not None:
                geometry = geometry.gscene.NAME_DICT[geometry.parent]

        while(not check_geometry_collision(gcheck, geometry, Q_dict=Q_dict)):
            if isinstance(obj, AbstractObject):
                obj.set_state(obj.binding, None)
            else:
                Toff = np.matmul(SE3(np.identity(3), [0, 0, 1e-3]), geometry.Toff)
                geometry.set_offset_tf(center=Toff[:3,3], orientation_mat=Toff[:3,:3])

##
# @brief for one-by-one single-line-define-and-sweep task
def clear_tasks(pscene):
    for k, v in pscene.subject_dict.items():
        if isinstance(v, AbstractTask):
            pscene.remove_subject(k)

##
# @brief for one-by-one single-line-define-and-sweep task
def set_single_sweep_line(pscene, idx, wp1, wp2, face, track_name="track_face"):
    clear_tasks(pscene)
    sweep_ = pscene.create_subject(oname="sweep{}".format(idx + 1), gname=track_name, _type=SweepLineTask,
                                   action_points_dict={
                                       wp1.name: SweepFrame(wp1.name, wp1, [0, 0, wp1.dims[2] / 2], [0, 0, 0]),
                                       wp2.name: SweepFrame(wp2.name, wp2, [0, 0, wp2.dims[2] / 2], [0, 0, 0])},
                                   clearance=[face])
    return sweep_


##
# @brief get full schedule time
def calc_schedule_time(dt_step, snode_schedule):
    traz_size_all = 0
    for snode in snode_schedule:
        traz_size_all += snode.traj_size
    return traz_size_all * dt_step


##
# @brief disperse_on a surface geometry
# @param pscene PlanningScene
# @param gcheck GraspChecker
# @param surface GeometryItem to be the surface
# @item_names    GeometryItem name that is already in the pscene.gscene
def disperse_on(pscene, gcheck, surface, item_names):
    gscene = pscene.gscene
    T_ref = surface.get_tf(pscene.combined_robot.home_dict)
    CLEARANCE = 1e-3
    halfXY = np.divide(surface.dims[:2], 2)

    gtem_dict = {}
    for gname in item_names:
        gtem = gscene.NAME_DICT[gname]
        offZ = surface.dims[2] / 2 + gtem.dims[2] / 2 + CLEARANCE
        col_ok = False
        while not col_ok:
            newcenter = np.matmul(T_ref, np.array(list(np.random.uniform(-1, 1, size=2) * halfXY) + [offZ] + [1])[:,
                                         np.newaxis])[:3, 0]
            newRot = np.matmul(T_ref[:3, :3], Rot_rpy((0, 0, np.random.uniform(2 * np.pi))))
            gtem.set_offset_tf(center=newcenter, orientation_mat=newRot)
            col_ok = check_geometry_collision(gcheck, gtem, pscene.combined_robot.home_dict,
                                              link_name=surface.link_name)
        gtem_dict[gname] = gtem
    gscene.update_markers_all()
    return gtem_dict

##
# @brief        show redundancy-applied action point
# @param btf    BindingTransform instance
def show_action_point(pscene, btf, Q):
    gscene = pscene.gscene
    subject, handle, actor = btf.get_instance_chain(pscene)
    Q_dict = Q if isinstance(Q, dict) else list2dict(Q, gscene.joint_names)
    if handle is not None:
        T_handle = np.matmul(handle.get_tf_handle(Q_dict, from_link=handle.geometry.link_name),
                             SE3_inv(btf.T_add_ah))
        gscene.add_highlight_axis("hl", "handle", link_name=handle.geometry.link_name,
                                  center=T_handle[:3, 3], orientation_mat=T_handle[:3, :3])
    if actor is not None:
        T_actor = np.matmul(actor.get_tf_handle(Q_dict, from_link=actor.geometry.link_name),
                            btf.T_add_ah)
        gscene.add_highlight_axis("hl", "actor", link_name=actor.geometry.link_name,
                                  center=T_actor[:3, 3], orientation_mat=T_actor[:3, :3])
    if handle is not None and actor is not None:
        T_hlink = SE3_inv(btf.T_loal)
        T_elink = btf.T_loal
        if handle.geometry.link_name != "base_link":
            gscene.add_highlight_axis("hl", "hlink", link_name=actor.geometry.link_name,
                                      center=T_hlink[:3, 3], orientation_mat=T_hlink[:3, :3])
        if actor.geometry.link_name != "base_link":
            gscene.add_highlight_axis("hl", "elink", link_name=handle.geometry.link_name,
                                      center=T_elink[:3, 3], orientation_mat=T_elink[:3, :3])

### resized image plot
# ratio = 1.0/3
# color_image_tmp = cv2.resize(color_image, dsize=None, fx=ratio, fy=ratio)
# cam_tmp = kn_config[0].copy()
# cam_tmp[0,0] *= ratio
# cam_tmp[1,1] *= ratio
# cam_tmp[1,2] *= ratio
# cam_tmp[1,2] *= ratio
# corner_dict_tmp = {k: v*ratio for k, v in corner_dict.items()}
#
# plt.figure(figsize=(25,15))
# color_image_out = draw_objects(color_image_tmp, aruco_map, {}, corner_dict_tmp, cam_tmp, kn_config[1], axis_len=0.1)#objectPose_dict, corner_dict
# plt.imshow(color_image_out[:,:,[2,1,0]])


### plot xy error bar
# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.collections import PatchCollection
# from matplotlib.patches import Rectangle
#
# # Number of data points
# n=5
#
# # Dummy data
# x=np.arange(0,n,1)
# y=np.random.rand(n)*5.
#
# # Dummy errors (above and below)
# xerr=np.random.rand(2,n)
# yerr=np.random.rand(2,n)
#
# # Create figure and axes
# fig,ax = plt.subplots(1)
#
# # Plot data points
# ax.errorbar(x,y,xerr=xerr,yerr=yerr,fmt='None',ecolor='k')
#
# # Function to plot error boxes
# def makeErrorBoxes(xdata,ydata,xerror,yerror,fc='r',ec='None',alpha=0.5):
#
#     # Create list for all the error patches
#     errorboxes = []
#
#     # Loop over data points; create box from errors at each point
#     for xc,yc,xe,ye in zip(xdata,ydata,xerror.T,yerror.T):
#         rect = Rectangle((xc-xe[0],yc-ye[0]),xe.sum(),ye.sum())
#         errorboxes.append(rect)
#
#     # Create patch collection with specified colour/alpha
#     pc = PatchCollection(errorboxes,facecolor=fc,alpha=alpha,edgecolor=ec)
#
#     # Add collection to axes
#     ax.add_collection(pc)
#
# # Call function to create error boxes
# makeErrorBoxes(x,y,xerr,yerr)
#
# # Add some space around the data points on the axes
# ax.margins(0.1)
#
# plt.show()



### compare single/stereo camera measurement error
#
# h_st_vec = []
# h_kn_vec = []
# for _ in range(30):
#     time.sleep(1)
#     kn_config, rs_config, T_c12 = calibrate_stereo(aruco_map, dictionary)
#
#     xyz_rpy_robots, xyz_rvec_cams, env_gen_dict, objectPose_dict, corner_dict, color_image  = \
#         detect_environment(
#             aruco_map, dictionary, robot_tuples=ROBOTS_ON_SCENE,
#             env_dict={'floor': CallHolder(GeoBox, ["center", "orientation"], BLH=(1.52,0.72,0.01)),
#                       'wall':CallHolder(GeoBox, ["center", "orientation"], BLH=(3,3,0.01))},
#             camT_dict={"cam0":np.identity(4), "cam1": T_c12},
#             ref_name='floor')
#
#     h_st_vec.append(np.matmul(SE3_inv(objectPose_dict["floor"]), objectPose_dict["box1"])[2,3])
#     h_st_vec.append(np.matmul(SE3_inv(objectPose_dict["floor"]), objectPose_dict["box2"])[2,3])
#     h_st_vec.append(np.matmul(SE3_inv(objectPose_dict["floor"]), objectPose_dict["box3"])[2,3])
#
#     color_image = get_kn_image()
#     objectPose_dict_kn, corner_dict_kn = get_object_pose_dict(color_image, aruco_map, dictionary, *kn_config)
#
#     h_kn_vec.append(np.matmul(SE3_inv(objectPose_dict_kn["floor"]), objectPose_dict_kn["box1"])[2,3])
#     h_kn_vec.append(np.matmul(SE3_inv(objectPose_dict_kn["floor"]), objectPose_dict_kn["box2"])[2,3])
#     h_kn_vec.append(np.matmul(SE3_inv(objectPose_dict_kn["floor"]), objectPose_dict_kn["box3"])[2,3])
#     print("="*100)
#     print("h_st_vec: {}/{}".format(round(np.mean(h_st_vec)*1000, 2), round(np.std(h_st_vec)*1000, 2)))
#     print("h_kn_vec: {}/{}".format(round(np.mean(h_kn_vec)*1000, 2), round(np.std(h_kn_vec)*1000, 2)))
#
# # [30, 100, 170]
# # ====================================================================================================
# # h_st_vec: 172.15/0.83 (173.56/171.1)
# # h_kn_vec: 166.7/1.42 (168.63/164.57)
# # ====================================================================================================
# # h_st_vec: 101.73/0.43 (102.57/100.92)
# # h_kn_vec: 96.82/1.76 (99.0/94.07)
# # ====================================================================================================
# # h_st_vec: 30.93/0.91 (32.37/29.64)
# # h_kn_vec: 26.81/2.57 (29.57/22.88)


def update_parents_recursive(snode_dict, snode, parents):
    raise(NotImplementedError("ON DEVELOPMENT"))
    snode.parents = parents
    snode.set_traj(snode.traj, snode_dict[parents[-1]].traj_tot)
    snode_dict[snode.idx] = snode
    for leaf in snode.leafs:
        update_parents_recursive(snode_dict, snode_dict[leaf], parents + [snode.idx])


def reconnect_snodes(snode_dict, snode_parent, snode_child):
    raise(NotImplementedError("ON DEVELOPMENT"))
    snode_parent.leafs += [snode_child.idx]
    snode_dict[snode_parent.idx] = snode_parent
    update_parents_recursive(snode_dict, snode_child, snode_parent.parents + [snode_parent.idx])


def remove_double_motion(ppline, snode_schedule, **kwargs):
    raise(NotImplementedError("ON DEVELOPMENT"))
    for snode_ppr, snode_cur in zip(snode_schedule[1:-2], snode_schedule[3:]):
        if snode_ppr.state.node == snode_cur.state.node:
            snode_parent = ppline.tplan.snode_dict[snode_ppr.parents[-1]]
            print("try reconnect {}->{}".format(snode_parent.idx, snode_cur.idx))
            state_to = snode_parent.state.copy(ppline.pscene)
            state_to.Q = snode_cur.state.Q
            try:
                traj, new_state, error, succ = ppline.test_connection(snode_parent.state, state_to, **kwargs)
            except Exception as e:
                succ = False
                print(e)
            print("success" if succ else "failure")
            if succ:
                reconnect_snodes(ppline.tplan.snode_dict, snode_parent, snode_cur)
    return ppline.tplan.idxSchedule2SnodeScedule(snode_schedule[-1].parents + [snode_schedule[-1].idx])


from ..planning.motion.moveit.moveit_py import PlannerConfig

##
# @brief get looking motion to a target point. com_link to tip link distance is maintained.
# @param target_point target point in global coords
# @param view_dir     looking direction in tip link
def get_look_motion(mplan, rname, from_Q, target_point, com_link, 
                    cam_link=None, view_dir=[0,0,1], timeout=1):
    gscene = mplan.gscene
    if isinstance(target_point, GeometryItem):
        target_point = target_point.get_tf(from_Q)[:3,3]
    if cam_link is None:
        cam_link = mplan.chain_dict[rname]['tip_link']
    ref_link = mplan.chain_dict[rname]['link_names'][0]
    Trb = SE3_inv(gscene.get_tf(ref_link, from_Q))
    target_point = np.matmul(Trb[:3,:3], target_point)+Trb[:3,3]
    Tcur = gscene.get_tf(cam_link, from_Q, from_link=ref_link)
    Tcom = gscene.get_tf(com_link, from_Q, from_link=ref_link)
    cur_dir = np.matmul(Tcur[:3,:3], view_dir)
    target_dir = target_point - Tcom[:3,3]
    dR = Rotation.from_rotvec(calc_rotvec_vecs(cur_dir, target_dir)).as_dcm()
    dRr = matmul_series(Tcom[:3,:3].transpose(), dR, Tcom[:3,:3])

    Toc = np.matmul(SE3_inv(Tcom), Tcur)
    Tot = np.matmul(SE3(dRr, (0,)*3), Toc)
    Ttar_ref = np.matmul(Tcom, Tot)
    retract_N = 10
    retract_step = 0.05
    mplan.update_gscene()
    for i_ret in range(retract_N):
        dP = np.multiply(view_dir, retract_step*i_ret)
        Ttar = np.copy(Ttar_ref)
        Ttar[:3,3] = Ttar_ref[:3,3] - np.matmul(Ttar_ref[:3,:3], dP)
        # xyzquat = T2xyzquat(Ttar)
        # traj, succ = mplan.planner.plan_py(rname, cam_link, xyzquat[0]+xyzquat[1], ref_link, from_Q,
        #                                    timeout=timeout)
        traj, succ = mplan.get_incremental_traj(cam_link, np.identity(4), Ttar,
                                                from_Q, step_size=0.01, ERROR_CUT=0.01,
                                                SINGULARITY_CUT=0.01, VERBOSE=False,
                                                ref_link=ref_link, VISUALIZE=False)
        if succ:
            Qtar = traj[-1]
            Qdiff = Qtar-from_Q
            steps = int(np.max(np.abs(Qdiff)) / 0.01)
            traj = from_Q[np.newaxis,:] + Qdiff[np.newaxis, :]*np.arange(steps+1)[:,np.newaxis].astype(float)/steps
            if mplan.validate_trajectory(traj):
                break
            else:
                succ = False
    return traj, succ