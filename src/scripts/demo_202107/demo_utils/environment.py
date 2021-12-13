from area_select import *
from pkg.controller.trajectory_client.kiro.kiro_udp_send import *
import time

def add_env(gscene):
    mobile_base = gscene.create_safe(gtype=GEOTYPE.BOX, name="mobile_base", link_name="base_link", 
                       dims=(0.6,0.4,0.439), center=(0,0,-0.439/2), rpy=(0,0,0), 
                       color=(0.8,0.8,0.8,0.5), display=True, fixed=True, collision=False)
    floor = gscene.create_safe(gtype=GEOTYPE.BOX, name="floor", link_name="base_link", 
                       dims=(10,10,0.01), center=(0,0,-0.439), rpy=(0,0,0), 
                       color=(0.8,0.8,0.8,0.5), display=True, fixed=True, collision=False)

def add_cam(gscene, tool_link="indy0_tcp", center=(-0.0785, 0, 0.013), rpy=(0, 0, 0)):
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="cam", link_name=tool_link,
                       dims=(0.061, 0.061, 0.026), center=center, rpy=rpy,
                       color=(0.8, 0.8, 0.8, 0.5), display=True, fixed=True, collision=False)

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="cam_col", link_name=tool_link,
                       dims=(0.081, 0.081, 0.046), center=(0,0,0), rpy=(0,0,0),
                       color=(0.8, 0.8, 0.8, 0.2), display=True, fixed=True, collision=True, parent="cam")

    viewpoint = gscene.create_safe(gtype=GEOTYPE.SPHERE, name="viewpoint", link_name=tool_link,
                                   dims=(0.01, 0.01, 0.01), center=(-0.013, 0, 0), rpy=(0, 0, -np.pi / 2),
                                   color=(1, 0, 0, 0.3), display=True, fixed=True, collision=False, parent="cam")

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="body", link_name=tool_link,
                       dims=(0.067, 0.067, 0.0335), center=(0, 0, -0.02975), rpy=(0,0,0),
                       color=(0.8, 0.8, 0.8, 1), display=True, fixed=True, collision=False, parent="cam")

    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="body_col", link_name=tool_link,
                       dims=(0.087, 0.087, 0.0535), center=(0, 0, -0.02975), rpy=(0,0,0),
                       color=(0.8, 0.8, 0.8, 0.2), display=True, fixed=True, collision=True, parent="cam")

    gscene.create_safe(gtype=GEOTYPE.SPHERE, name="backhead", link_name=tool_link,
                       dims=(0.067, 0.067, 0.067), center=(0, 0, -0.0465), rpy=(0,0,0),
                       color=(0.8, 0.8, 0.8, 1), display=True, fixed=True, collision=False, parent="cam")

    gscene.create_safe(gtype=GEOTYPE.SPHERE, name="backhead_col", link_name=tool_link,
                       dims=(0.087, 0.087, 0.087), center=(0, 0, -0.0465), rpy=(0,0,0),
                       color=(0.8, 0.8, 0.8, 0.2), display=True, fixed=True, collision=True, parent="cam")
    return viewpoint

def add_kiro_indytool_down(gscene, zoff=0, tool_link="indy1_tcp", face_name="brush_face", ext_off=0.032, tool_dim=(0.08, 0.32)):

    gscene.create_safe(gtype=GEOTYPE.MESH, name="indy_tool_vis", link_name=tool_link,
                       dims=(0.1,0.1,0.1), center=(0,0,ext_off), rpy=(0,0,0),
                       display=True, color=(0.8,0.8,0.8,1), collision=False, fixed=True,
                       uri="package://my_mesh/meshes/stl/kiro_indytool_down_res.stl")
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="hinge_bar_col", link_name=tool_link,
                       center=(0.08, 0, 0.10+ext_off), dims=(0.1, 0.1, 0.25), rpy=(0, np.pi / 4, 0),
                       display=True, color=(0.8, 0.8, 0.8, 0.7), collision=True, fixed=True)
    brush_face = gscene.create_safe(gtype=GEOTYPE.BOX, name=face_name, link_name=tool_link,
                       center=(0.27+zoff, 0, 0.236+ext_off), dims=(tool_dim[0]+0.02, tool_dim[1]+0.02, 0.01), rpy=(0, -np.pi/2, 0),
                       color=(1.0, 0.0, 0.0, 0.5),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_col".format(face_name), link_name=tool_link,
                       center=(0.225+zoff/2, 0, 0.236+ext_off), dims=(tool_dim[0]+0.02, tool_dim[1]+0.02, 0.09+zoff), rpy=(0, -np.pi/2, 0),
                       color=(0.8, 0.8, 0.8, 0.8),
                       collision=True, fixed=True)
    return brush_face

def add_brush(gscene, thickness=0.03, tool_link="indy1_tcp", face_name="brush_face",
              tool_dim=(0.08, 0.32), clearance=1e-2, clearance_side=1e-2,
              col_color=(1,1,1,0.2), brush_color=(0.8, 0.8, 0.8, 0.8)):
    brush_face = gscene.create_safe(gtype=GEOTYPE.BOX, name=face_name, link_name=tool_link,
                                    dims=(tool_dim[0], tool_dim[1], clearance),
                                    center=(0, 0, -thickness+clearance/2), rpy=(0, 0, 0),
                                    color=(1.0, 0.0, 0.0, 0.5), collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_body".format(face_name), link_name=tool_link,
                       dims=(tool_dim[0], tool_dim[1], thickness-clearance),
                       center=(0, 0, (thickness)/2), rpy=(0, 0, 0),
                       color=brush_color, collision=False, fixed=True, parent=face_name)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_col".format(face_name), link_name=tool_link,
                       dims=(tool_dim[0]+clearance_side*2, tool_dim[1]+clearance_side*2, thickness-clearance+clearance_side),
                       center=(0, 0, (thickness+clearance_side)/2), rpy=(0, 0, 0),
                       color=col_color, collision=True, fixed=True, parent=face_name)
    return brush_face

def add_kiro_indytool_up(gscene, zoff=0, tool_link="indy1_tcp", face_name="brush_face", ext_off=0.032, tool_dim=(0.08, 0.32)):
    gscene.create_safe(gtype=GEOTYPE.MESH, name="indy_tool_vis", link_name=tool_link,
                       dims=(0.1,0.1,0.1), center=(0,0,0+ext_off), rpy=(0,0,0),
                       display=True, color=(0.8,0.8,0.8,1), collision=False, fixed=True,
                       uri="package://my_mesh/meshes/stl/kiro_indytool_up_res.stl")
    gscene.create_safe(gtype=GEOTYPE.CYLINDER, name="hinge_bar_col", link_name=tool_link,
                       center=(0.08, 0, 0.10+ext_off), dims=(0.1, 0.1, 0.25), rpy=(0, np.pi / 4, 0),
                       display=True, color=(0.8, 0.8, 0.8, 0.7), collision=True, fixed=True)
    brush_face = gscene.create_safe(gtype=GEOTYPE.BOX, name=face_name, link_name=tool_link,
                       center=(0.18, 0, 0.295+zoff+ext_off), dims=(tool_dim[0]+0.02, tool_dim[1]+0.02, 0.01), rpy=(0, -np.pi, 0),
                       color=(1.0, 0.0, 0.0, 0.5),
                       collision=False, fixed=True)
    gscene.create_safe(gtype=GEOTYPE.BOX, name="{}_col".format(face_name), link_name=tool_link,
                       center=(0.18, 0, 0.25+zoff/2+ext_off), dims=(tool_dim[0]+0.02, tool_dim[1]+0.02, 0.09+zoff), rpy=(0, -np.pi, 0),
                       color=(0.8, 0.8, 0.8, 0.8),
                       collision=True, fixed=True)
    return brush_face

def add_bed(gscene, bed_center, bed_rpy, COLOR_BED_COL, add_back_wall=True,
            bed_width=0.91, margin=0.12, cover_len=1.7):
    col_width = bed_width + margin*2
    bed_vis = gscene.create_safe(GEOTYPE.MESH, "bed_vis", link_name="base_link",
                                 dims=(0.1,0.1,0.1), center=bed_center, rpy=bed_rpy,
                                 color=(0.8,0.8,0.8,1), display=True, fixed=True, collision=False,
                                 uri="package://my_mesh/meshes/stl/bed_floor_centered_m_scale.stl", scale=(1,1.1,1))
    bed_mat = gscene.create_safe(GEOTYPE.BOX, "bed_mat", link_name="base_link", 
                                 dims=(cover_len,bed_width,0.01), center=(0.0,0,0.66), rpy=(0,0,0),
                                 color=COLOR_BED_COL, fixed=True, collision=False, parent="bed_vis")

    bed_mat_col = gscene.create_safe(GEOTYPE.BOX, "bed_mat_col", link_name="base_link",
                                 dims=(1.80,col_width,0.13), center=(0.02,0,0.6), rpy=(0,0,0),
                                 color=(1, 1, 1, 0.1), fixed=True, collision=True, parent="bed_vis")

    gscene.create_safe(GEOTYPE.BOX, "bed_head", link_name="base_link", 
                                 dims=(0.3,col_width,1.30), center=(-1.03,0,0.5), rpy=(0,0,0),
                                 color=(1, 1, 1, 0.1), fixed=True, collision=True, parent="bed_vis")

    gscene.create_safe(GEOTYPE.BOX, "bed_foot", link_name="base_link", 
                                 dims=(0.3,col_width,1.30), center=(1.05,0,0.5), rpy=(0,0,0),
                                 color=(1, 1, 1, 0.1), fixed=True, collision=True, parent="bed_vis")

    gscene.create_safe(GEOTYPE.BOX, "bed_box", link_name="base_link",
                       dims=(3, col_width+0.5, 1.3), center=(0.02, 0, 0.5), rpy=(0, 0, 0),
                       color=(1, 1, 1, 0.1), fixed=True, collision=False, parent="bed_vis")

    gscene.create_safe(GEOTYPE.BOX, "room_box", link_name="base_link",
                       dims=(3, 6, 1.3), center=(0.02, 0, 0.5), rpy=(0, 0, 0),
                       color=(1, 1, 1, 0.1), fixed=True, collision=False, parent="bed_vis")

    gscene.create_safe(GEOTYPE.BOX, "bed_left_space", link_name="base_link",
                       dims=(2.5, 1, 3), center=(0.02, -col_width/2-0.4, 1), rpy=(0, 0, 0),
                       color=(1, 1, 1, 0.05), fixed=True, collision=False, parent="bed_vis")

    gscene.create_safe(GEOTYPE.BOX, "bed_right_space", link_name="base_link",
                       dims=(2.5, 1, 3), center=(0.02, col_width/2+0.4, 1), rpy=(0, 0, 0),
                       color=(1, 1, 1, 0.05), fixed=True, collision=False, parent="bed_vis")

    if add_back_wall:
        gscene.create_safe(GEOTYPE.BOX, "bed_wall", link_name="base_link",
                           dims=(0.5,7.0,3), center=(-1.27,0,1.5), rpy=(0,0,0),
                           color=(1, 1, 1, 0.1), fixed=True, collision=True, parent="bed_vis")
    return bed_mat

def move_bed(gscene, bed_center, bed_rpy):
    bed_vis = gscene.NAME_DICT["bed_vis"]
    bed_vis.set_offset_tf(center=bed_center, orientation_mat=Rot_rpy(bed_rpy))
    gscene.update_markers_all()


def add_closet(gscene, closet_center, closet_rpy, COLOR_CLOSET_COL = (0,1,0,0.3),
               margin = 0.01, margin_col = 0.06):    
    closet_vis = gscene.create_safe(GEOTYPE.MESH, "closet_vis", link_name="base_link", 
                                    dims=(0.1,0.1,0.1), center=closet_center, rpy=closet_rpy,
                                    color=(0.8,0.8,0.8,1), display=True, fixed=True, collision=False,
                                    uri="package://my_mesh/meshes/stl/top_table_centered_m_scale.stl", scale=(1,1,1))

    closet_leftup = gscene.create_safe(GEOTYPE.BOX, "closet_leftup", link_name="base_link",
                             dims=(1.3,0.255+margin*2,0.02), center=(0.30,-0.145,1.52), rpy=(0,np.pi/2,0),
                             color=COLOR_CLOSET_COL, fixed=True, collision=False, parent="closet_vis")
    closet_leftup_col = gscene.create_safe(GEOTYPE.BOX, "closet_leftup_col", link_name="base_link",
                             dims=(1.2,0.255+margin_col,0.6), center=(0,-0.145-margin_col/2,1.52), rpy=(0,np.pi/2,0),
                             color=(0, 0, 0, 0.1), fixed=True, collision=True, parent="closet_vis")

    closet_rightup = gscene.create_safe(GEOTYPE.BOX, "closet_rightup", link_name="base_link", 
                             dims=(0.58,0.32,0.025), center=(0.22-0.065,0.19,1.87), rpy=(0,np.pi/2,0),
                             color=COLOR_CLOSET_COL, fixed=True, collision=False, parent="closet_vis")
    closet_rightup_col = gscene.create_safe(GEOTYPE.BOX, "closet_rightup_col", link_name="base_link",
                             dims=(1.0,0.22+margin_col,0.465), center=(-0.01-0.065,0.12 +margin_col/2,1.55), rpy=(0,np.pi/2,0),
                             color=(0, 0, 0, 0.1), fixed=True, collision=True, parent="closet_vis")
    
    closet_down = gscene.create_safe(GEOTYPE.BOX, "closet_down", link_name="base_link",
                                 dims=(0.78,0.495+margin*2,0.02), center=(0.31,-0.025,0.5), rpy=(0,np.pi/2,0),
                                 color=COLOR_CLOSET_COL, fixed=True, collision=False, parent="closet_vis")
    closet_down_col = gscene.create_safe(GEOTYPE.BOX, "closet_down_col", link_name="base_link",
                                 dims=(0.78,0.495+margin_col*2,0.64), center=(-0.01,-0.025,0.5), rpy=(0,np.pi/2,0),
                                 color=(0, 0, 0, 0.1), fixed=True, collision=True, parent="closet_vis")
    # closet_shelf = gscene.create_safe(GEOTYPE.BOX, "closet_shelf", link_name="base_link",
    #                              dims=(0.02,0.24+margin*2,0.465), center=(-0.065,0.105,1.24), rpy=(0,np.pi/2,0),
    #                              color=(0, 0, 0, 0.1), fixed=True, collision=True, parent="closet_vis")
    closet_back = gscene.create_safe(GEOTYPE.BOX, "closet_back", link_name="base_link", 
                                 dims=(0.02,0.24,0.73), center=(-0.29,0.105,1.22), rpy=(0,0,0),
                                 color=(0, 0, 0, 0.1), fixed=True, collision=True, parent="closet_vis")
    gscene.create_safe(
        gtype=GEOTYPE.BOX, name="closet_box", link_name="base_link",
        dims=(1, 0.9, 2.3), center=(0, 0, 1.1), rpy=(0, 0, 0),
        color=(0, 0, 0, 0.1), display=True, collision=False, fixed=True, parent="closet_vis")
    return closet_leftup, closet_rightup, closet_down

def move_closet(gscene, closet_center, closet_rpy):
    closet_vis = gscene.NAME_DICT["closet_vis"]
    closet_vis.set_offset_tf(center=closet_center, orientation_mat=Rot_rpy(closet_rpy))
    gscene.update_markers_all()

# add back_wall geometry
def add_backwall(gscene):
    closet_vis = gscene.NAME_DICT["closet_vis"]
    gscene.create_safe(GEOTYPE.BOX, "back_wall", link_name="base_link",
                   dims=(0.2,7.,7), center=(-0.3,0,0), rpy=(0,0,0),
                   color=(1, 1, 1, 0.2), fixed=True, collision=True, parent="closet_vis")

class SwitchState(Enum):
    NONE = 0
    BASE_MOVED = 1
    SWEEP_APPROACH = 2
    SWEEP_RETRACT = 3
    SWEEPING = 4

from pkg.planning.mode_switcher import ModeSwitcherTemplate

class ModeSwitcherKMB(ModeSwitcherTemplate):
    def __init__(self, pscene, push_dist=0.05):
        self.pscene = pscene
        self.gscene = pscene.gscene
        self.crob = pscene.combined_robot
        mobile_name = \
        [rconfig.get_indexed_name() for rconfig in self.crob.robots_on_scene if rconfig.type == rconfig.type.kmb][0]
        self.kmb = self.crob.robot_dict[mobile_name]
        self.mobile_link = self.crob.get_robot_tip_dict()[mobile_name]
        self.push_dist = push_dist
        self.Q_before_push = None

    def switch_in(self, snode_pre, snode_new):
        switch_state = SwitchState.NONE
        snode_pre_cp = snode_pre.copy(self.pscene)
        snode_pre_cp.traj = None
        #         ppline.play_schedule([snode_pre_cp, snode_new])

        from_state = snode_pre.state
        to_state = snode_new.state
        subjects, ok = self.pscene.get_changing_subjects(from_state, to_state)
        if from_state.node[0] == 1 and to_state.node[0] == 2:
            switch_state = SwitchState.SWEEP_APPROACH
        elif from_state.node[0] == 2 and to_state.node[
            0] == 2:  # joint motion: quit sweep and homing - retract before motion
            if self.Q_before_push is not None:
                print("[MODE] RETRACT BEFORE HOMING")
                switch_state = SwitchState.SWEEP_RETRACT
                self.kmb.joint_move_make_sure(self.Q_before_push)
                self.Q_before_push = None
        return switch_state

    def switch_out(self, switch_state, snode_new):
        if switch_state == SwitchState.SWEEP_APPROACH:  # move forward
            if self.push_dist > 1e-6:
                print("[MODE] PUSH FOWARD")
                Qcur = self.kmb.get_qcur()
                Tbm = self.gscene.get_tf(self.mobile_link, Qcur)
                Tbm2 = np.matmul(Tbm, SE3(np.identity(3), [self.push_dist, 0, 0]))
                Qpush = list(Tbm2[:2, 3]) + [Rot2axis(Tbm2[:3, :3], 3), 0, 0, 0]
                self.kmb.joint_move_make_sure(Qpush)
                self.Q_before_push = Qcur