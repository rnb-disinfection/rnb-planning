import os
os.chdir(os.path.join(os.environ["RNB_PLANNING_DIR"], 'src'))

from pkg.controller.combined_robot import *
from pkg.project_config import *

crob = CombinedRobot(robots_on_scene=[
    RobotConfig(0, RobotType.panda, None,
                INDY_IP)]
              , connection_list=[False])
from pkg.geometry.builder.scene_builder import SceneBuilder
s_builder = SceneBuilder(None)
# s_builder.reset_reference_coord(ref_name="floor")
# xyz_rpy_robots = s_builder.detect_items(level_mask=[DetectionLevel.ROBOT])
xyz_rpy_robots = {"panda0": ((0,0,0), (0,0,0))}
crob.update_robot_pos_dict(xyz_rpy_robots=xyz_rpy_robots)
gscene = s_builder.create_gscene(crob)
from pkg.planning.scene import PlanningScene
pscene = PlanningScene(gscene, combined_robot=crob)

from pkg.geometry.geometry import *
from pkg.planning.constraint.constraint_actor import Gripper2Tool, PlacePlane

floor = gscene.create_safe(GEOTYPE.BOX, "floor", "base_link", (3,3,0.01), (0,0,0),
                           rpy=(0,0,0), color=(0.8,0.8,0.8,0.5), display=True, fixed=True, collision=True)
wall = gscene.create_safe(GEOTYPE.BOX, "wall", "base_link", (3,3,0.01), (-0.3,0,0),
                           rpy=(0,np.pi/2,0), color=(0.8,0.8,0.8,0.5), display=True, fixed=True, collision=True)

gtems = s_builder.add_robot_geometries(color=(0,1,0,0.5), display=True, collision=True)

gscene.create_safe(gtype=GEOTYPE.SPHERE, name="grip0", link_name="panda0_hand",
                 dims=(0.01,)*3, center=(0,0,0.112), rpy=(-np.pi/2,0,0), color=(1,0,0,1), display=True, collision=False, fixed=True)
grip0 = pscene.create_binder(bname="grip0", gname="grip0", _type=Gripper2Tool, point=(0,0,0), rpy=(0,0,0))
gscene.show_pose(crob.home_pose)

from pkg.utils.code_scraps import *

from pkg.planning.constraint.constraint_actor import KnobFramer, HingeFramer
from pkg.planning.constraint.constraint_subject import KnobFrame, HingeFrame


##
# @class KnobTask
# @brief sweep action points in alphabetical order
# @remark   state_param: boolean vector of which each element represents if each waypoint is covered or not
#           node_item: number of covered waypoints
class KnobTask(SweepTask):
    def __init__(self, oname, geometry, lever, action_points_dict, sub_binders_dict=None, tol=1e-3):
        self.lever = lever
        self.T0 = geometry.Toff
        self.link0 = geometry.link_name
        SweepTask.__init__(self, oname, geometry, action_points_dict,
                           sub_binders_dict=sub_binders_dict, tol=tol, one_direction=False)

    ##
    # @brief set object binding state and update location
    # @param binding BindingTransform
    # @param state_param list of done-mask
    def set_state(self, binding, state_param=None):
        self.binding = deepcopy(binding)
        self.state_param = np.zeros(len(self.action_points_order), dtype=np.bool)
        if self.binding.chain.handle_name is not None:
            if self.binding.chain.handle_name in self.action_points_order:
                self.state_param[:self.action_points_order.index(self.binding.chain.handle_name) + 1] = True
            else:
                raise (RuntimeError("Undefined handle {}".format(self.binding.chain.handle_name)))
        if binding.chain.handle_name is None:
            handle = self.action_points_dict[self.action_points_order[0]]
        else:
            handle = self.action_points_dict[binding.chain.handle_name]
        actor = self.lever
        if all(self.state_param):
            self.geometry.set_offset_tf(binding.T_lao[:3, 3], binding.T_lao[:3, :3])
            self.geometry.set_link(binding.actor_link)
        else:
            self.geometry.set_offset_tf(self.T0[:3, 3], self.T0[:3, :3])
            self.geometry.set_link(self.link0)

        self.update_sub_points()

    ##
    # @brief make constraints. by default, empty list.
    # @remark constraint is applied when using same binding
    # @param binding_from previous binding
    # @param binding_to next binding
    def make_constraints(self, binding_from, binding_to, tol=None):
        if binding_from is not None and binding_from.actor_name == binding_to.actor_name:
            return "Constraint not implemented yet. Use MoveitPlanner.incremental_constraint_motion=True"
        else:
            return []

    ##
    # @brief get object-level neighbor component (detach or next waypoint)
    def get_neighbor_node_component_list(self, node_tem, pscene):
        if node_tem == len(self.state_param):
            neighbor = [node_tem]
        else:  # node_tem < len(self.state_param):
            neighbor = [node_tem + 1]
        if (not self.one_direction) and node_tem > 0:
            neighbor.insert(0, node_tem - 1)
        return neighbor

def add_door(pscene, dname = "door", center = (0.5,0,0.5), rpy = (0,0,0),
             dims = (0.05, 0.4, 0.5), hinge_point = (-0.025, 0.2, 0),
             door_ang = np.pi*1/4, door_div = 1, link_name="base_link", clearance=3e-3,
             frame_depth=0.0, frame_thickness=0.01, color = (0.8,0.8,0.8,1)):
    gscene = pscene.gscene
    door = gscene.create_safe(GEOTYPE.BOX, dname, link_name=link_name,
                              center=center, rpy=rpy, dims=dims,
                              fixed=False, collision=True, color=color)
    door_frame = gscene.create_safe(GEOTYPE.BOX, dname+"_frame", link_name=link_name,
                                    center=center, rpy=Rot2rpy(np.matmul(Rot_rpy(rpy), Rot_axis(2,np.pi/2))),
                                    dims=np.asarray(dims)[[2,1,0]],
                                    fixed=True, collision=False, color=(1,1,1,0.0), display=False)
#     gscene.add_virtual_guardrail(door_frame, axis="xy", color=color, THICKNESS=frame_thickness,
#                                  HEIGHT=dims[0]/2+frame_depth, margin=frame_thickness/2+clearance)

    door_r = pscene.create_binder(bname=door.name+"_hinge", gname=door.name, _type=HingeFramer,
                                  point=hinge_point)

    hinge_points = []
    for i_div in range(door_div+1):
        ang = door_ang/door_div*i_div
        hinge_points.append(HingeFrame("h{}".format(i_div), door_frame,
                                       np.asarray(hinge_point)[[2,1,0]],
                                       Rot2rpy(
                                           np.matmul(Rot_axis(2,np.pi/2).transpose(),
                                                     Rot_axis(3, -ang)))))

    door_s = pscene.create_subject(oname=door.name, gname=door_frame.name, _type=SweepTask,
                                  action_points_dict={hp.name: hp for hp in hinge_points},
                                  one_direction=False)
    return door, door_s


def add_lever(pscene, knob,lname="lever", lever_ang=np.pi/4, knob_offset=(0.09), dims=(0.02, 0.2, 0.02),
              link_name="base_link", color=(0.8, 0.8, 0.8, 1), clearance = 1e-3):
    gscene = pscene.gscene
    Q0 = [0]*gscene.joint_num
    Tbk = knob.get_tf(Q0)
    Tbkp = np.matmul(Tbk, SE3(np.identity(3), (0, 0, knob.dims[2]/2+clearance)))
    Tbl = np.matmul(Tbkp, SE3(np.identity(3), (0, knob_offset, dims[2]/2)))
    lever = gscene.create_safe(GEOTYPE.BOX, lname, link_name=link_name,
                                    center=Tbl[:3,3], rpy=Rot2rpy(Tbl[:3,:3]), dims=dims,
                                    fixed=False, collision=True, color=color)
    lgrasp = Grasp2Point("gp", lever, (0, 0, 0), (np.pi/2, 0, -np.pi/2))
    lattach = PlaceFrame("cp", lever, (0, -knob_offset, -dims[2]/2), (0,0,0))

    lever_g = pscene.create_subject(oname=lname+"_grip", gname=lever.name, _type=CustomObject,
                                     action_points_dict={lgrasp.name: lgrasp, lattach.name: lattach})
    kpoint0 = pscene.create_binder(bname=knob.name + "_0", gname=knob.name, _type=AttachFrame,
                                   point=(0, 0, knob.dims[2]/2+clearance))
    kpoint1 = pscene.create_binder(bname=knob.name + "_1", gname=knob.name, _type=AttachFrame,
                                   point=(0, 0, knob.dims[2]/2+clearance), rpy=(0, 0, lever_ang))

    lever_plug = pscene.create_binder(bname=knob.name+"_plug", gname=lname, _type=KnobFramer,
                                   point=(0, -knob_offset, -dims[2]/2))
    rp1 = KnobFrame("r1", knob, (0, 0, knob.dims[2]/2+clearance), (0,0,0))
    rp2 = KnobFrame("r2", knob, (0, 0, knob.dims[2]/2+clearance), rpy=(0, 0, lever_ang))

    knob_s = pscene.create_subject(oname=knob.name, gname=knob.name, _type=KnobTask,
                                   action_points_dict={rp1.name: rp1, rp2.name: rp2},
                                   lever = lever_plug)
    return knob_s

door, door_s = add_door(pscene, center=(0.7,-0.2,0.5), color=(0.8,0.4,0.2,1))
knob = gscene.create_safe(GEOTYPE.BOX, 'knob', link_name="base_link",
                          center=(-door.dims[0]/2-0.01,-door.dims[1]/2+0.05,0), rpy=(0,-np.pi/2,0), dims=(0.02,0.02,0.02),
                          fixed=False, collision=True, color=(0.8,0.8,0.8,1), parent="door")

knob_s = add_lever(pscene, knob, dims=(0.02,0.1,0.02), knob_offset=0.04)

from pkg.planning.motion.moveit.moveit_planner import MoveitPlanner
mplan = MoveitPlanner(pscene, enable_dual=False)
mplan.update_gscene()

from pkg.planning.task.rrt import TaskRRT
tplan = TaskRRT(pscene)
tplan.prepare()

from pkg.planning.pipeline import PlanningPipeline
ppline = PlanningPipeline(pscene)
ppline.set_motion_planner(mplan)
ppline.set_task_planner(tplan)

from pkg.ui.ui_broker import *
# start UI
ui_broker = UIBroker.instance()
ui_broker.initialize(ppline, s_builder)
ui_broker.start_server()

ui_broker.set_tables()

from pkg.planning.filtering.grasp_filter import GraspChecker
from pkg.planning.filtering.reach_filter import ReachChecker
mplan.motion_filters = [GraspChecker(pscene), ReachChecker(pscene)]
mplan.incremental_constraint_motion = True

gscene.show_pose(crob.home_pose)
initial_state = pscene.initialize_state(crob.home_pose)
print(pscene.subject_name_list)
print(initial_state.node)

goal_nodes = [(2, 2,'grip0')]
ppline.search(initial_state, goal_nodes, max_solution_count=1,
              verbose=True, display=False, dt_vis=0.01,
              timeout=0.5, timeout_loop=300,
              multiprocess=False)