from moveit_py import MoveitCompactPlanner, ObjectType, ObjectMPC
from ..interface import PlannerInterface
from ...utils.joint_utils import get_min_distance_map
from ...geometry.geometry import GEOTYPE

def gtype_to_otype(gtype):
    if gtype==GEOTYPE.BOX:
        return ObjectType.BOX
    elif gtype==GEOTYPE.SPHERE:
        return ObjectType.SPHERE
    elif gtype==GEOTYPE.SEGMENT:
        return ObjectType.CYLINDER

class MoveitPlanner(PlannerInterface):
    NAME = "Moveit"

    def __init__(self, urdf_path, srdf_path, robot_names):
        self.planner = MoveitCompactPlanner(urdf_path, srdf_path, robot_names)

    def update_gtems(self):
        self.ghnd.update()
        obj_list = []
        for gtem in self.ghnd:
            if gtem.collision:
                obj_list.append(ObjectMPC(
                    gtem.name, gtype_to_otype(gtem.gtype),gtem.link_name,True
                ))
        self.planner.set_scene([])

    @abstractmethod
    def plan_transition(self, from_state, to_state, binding_list, **kwargs):
        pass

    def init_online_plan(self, from_state, to_state, binding_list, T_step, control_freq, playback_rate=0.5, **kwargs):
        raise(RuntimeError("online operation not implemented with moveit"))

    def step_online_plan(self, i_q, pos, wp_action=False):
        raise(RuntimeError("online operation not implemented with moveit"))

    def update_online(self, obsPos_dict):
        raise(RuntimeError("online operation not implemented with moveit"))

    def update_target_joint(self, idx_cur, traj, joint_cur):
        raise(RuntimeError("online operation not implemented with moveit"))