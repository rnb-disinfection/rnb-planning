from ..project_config import *
from ..controller.combined_robot import *
from ..geometry.builder.scene_builder import SceneBuilder
from ..geometry.geometry import *
from ..utils.utils import *
from ..planning.scene import PlanningScene
from ..planning.constraint.constraint_actor import Gripper2Tool, PlacePlane, SweepTool
from ..planning.motion.moveit.moveit_planner import MoveitPlanner

meta_data = {}

def set_meta_data(key, dat):
    global meta_data
    meta_data[key] = dat

def get_meta_data(key=None):
    if key is None:
        return meta_data
    else:
        return meta_data[key]

##
# @brief get robot-specific parameters for single-robot tests
def get_single_robot_params(ROBOT_TYPE):
    if ROBOT_TYPE in [RobotType.indy7, RobotType.indy7gripper]:
        ROBOT_NAME = "indy0"
        TOOL_LINK = "indy0_tcp"
        TOOL_XYZ = (0, 0, 0.14)
        TOOL_RPY = (-np.pi / 2, 0, 0)
        HOME_POSE = (0, 0, 0, 0, 0, 0)
        GRIP_DEPTH = 0.05

    elif ROBOT_TYPE == RobotType.panda:
        ROBOT_NAME = "panda0"
        TOOL_LINK = "panda0_hand"
        TOOL_XYZ = (0, 0, 0.112)
        TOOL_RPY = (-np.pi / 2, 0, 0)
        HOME_POSE = (0, -0.3, 0, -0.5, 0, 2.5, np.pi/2)
        GRIP_DEPTH = 0.03
    return ROBOT_NAME, TOOL_LINK, TOOL_XYZ, TOOL_RPY, HOME_POSE, GRIP_DEPTH


##
# @brief prepare single-robot planning scene
def prepare_single_robot_scene(ROBOT_TYPE, ROBOT_NAME, TOOL_LINK, TOOL_XYZ, TOOL_RPY,
                               VISUALIZE=True, base_xyz_rpy=((0, 0, 0), (0, 0, 0))):
    crob = CombinedRobot(robots_on_scene=[
        RobotConfig(0, ROBOT_TYPE, base_xyz_rpy,
                    INDY_IP)]
        , connection_list=[False])

    s_builder = SceneBuilder(None)
    gscene = s_builder.create_gscene(crob, start_rviz=VISUALIZE)

    gtems_robot = s_builder.add_robot_geometries(color=(0, 1, 0, 0.5), display=False, collision=True)

    pscene = PlanningScene(gscene, combined_robot=crob)

    gscene.create_safe(gtype=GEOTYPE.SPHERE, name="grip0", link_name=TOOL_LINK,
                       dims=(0.01,) * 3, center=TOOL_XYZ, rpy=TOOL_RPY, color=(1, 0, 0, 1), display=True,
                       collision=False,
                       fixed=True)
    gripper = pscene.create_binder(bname="grip0", gname="grip0", _type=Gripper2Tool, point=(0, 0, 0), rpy=(0, 0, 0))

    if ROBOT_TYPE == RobotType.panda:
        gripper.redundancy = {'w': (-np.pi / 6, np.pi / 6)}

    return s_builder, pscene


def create_data_dirs(dat_root, rtype, dat_dir=None):
    if dat_dir is None:
        dat_dir = get_now()

    DATA_PATH = os.path.join(os.environ['RNB_PLANNING_DIR'], "data")
    try_mkdir(DATA_PATH)

    TEST_DATA_PATH = os.path.join(DATA_PATH, dat_root)
    try_mkdir(TEST_DATA_PATH)

    ROBOT_DATA_ROOT = os.path.join(TEST_DATA_PATH, rtype)
    try_mkdir(ROBOT_DATA_ROOT)

    DATASET_PATH = os.path.join(ROBOT_DATA_ROOT, dat_dir)
    try_mkdir(DATASET_PATH)

    return DATASET_PATH


from pkg.planning.constraint.constraint_subject import CustomObject, Grasp2Point, PlacePoint, SweepPoint, SweepTask
from pkg.planning.filtering.lattice_model.scene_building import *

def load_gtem_args(gscene, gtem_args):
    gtem_remove = []
    for gtem in gscene:
        if ((gtem.link_name == "base_link" or not gtem.fixed)
                and gtem.parent is None):
            gtem_remove.append(gtem)
    for gtem in gtem_remove:
        gscene.remove(gtem)

    gid_list = np.arange(len(gtem_args)).tolist()
    for gidx in gid_list:
        args = gtem_args[gidx]
        if args['parent'] is not None:
            if args['parent'] not in gscene.NAME_DICT:
                gid_list.append(gidx)
                continue
        gscene.create_safe(**args)

def load_saved_scene(pscene, file_path, VISUALIZE=True):
    gscene = pscene.gscene
    saved_data = load_pickle(file_path)
    gtem_args = saved_data['gtem_args']
    obj_args = saved_data['obj_args']

    load_gtem_args(gscene, gtem_args)

    pscene.create_binder(bname="wp", gname="wp", _type=PlacePlane, point=None)
    pscene.create_binder(bname="gp", gname="gp", _type=PlacePlane, point=None)

    obj_set_list = []
    for obj_name in sorted(obj_args.keys()):
        obj, obj_arg = DummyObject(), obj_args[obj_name]
        obj.name = obj_name
        obj.GRIP_DEPTH = obj_arg["GRIP_DEPTH"]
        obj.DIM = obj_arg["DIM"]
        obj.geometry = gscene.NAME_DICT[obj_arg["gname"]]
        obj.CLEARANCE = obj_arg["CLEARANCE"]
        obj_pscene, handles = add_object(pscene, obj)
        obj_set_list.append((obj, obj_pscene, handles))

    if VISUALIZE:
        gscene.set_rviz()

    initial_state = pscene.initialize_state(pscene.combined_robot.home_pose)
    pscene.set_object_state(initial_state)
    from_state = initial_state.copy(pscene)
    gscene.update()
    return initial_state