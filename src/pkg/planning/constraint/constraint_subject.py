from __future__ import print_function

from .constraint_common import *
from ...constants import DIR_RPY_DICT, DIR_VEC_DICT

from itertools import combinations
from abc import *
from copy import deepcopy
__metaclass__ = type


##
# @class DirectedPoint
# @brief ActionPoint with z-axis constraint
class DirectedPoint(ActionPoint):
    ##
    # @brief update redundancy based on current informations: point, dims
    def update_redundancy(self):
        if self.point is not None:
            self.redundancy = {"w":(-np.pi,np.pi)}
        else:
            dims =self.geometry.dims
            self.redundancy = {"x":(-dims[0]/2,dims[0]/2),
                               "y":(-dims[1]/2,dims[1]/2),
                               "z":(dims[2]/2,dims[2]/2),
                               "w":(-np.pi,np.pi)}

##
# @class PitchPoint
# @brief ActionPoint with z-axis constraint
class PitchPoint(ActionPoint):
    def update_redundancy(self):
        if self.point is not None:
            self.redundancy = {"v":(-np.pi,np.pi)}
        else:
            dims =self.geometry.dims
            self.redundancy ={"x":(-dims[0]/2,dims[0]/2),
                              "y":(-dims[1]/2,dims[1]/2),
                              "z":(dims[2]/2,dims[2]/2),
                              "v":(-np.pi,np.pi)}

##
# @class RollPoint
# @brief ActionPoint with z-axis constraint
class RollPoint(ActionPoint):
    def update_redundancy(self):
        if self.point is not None:
            self.redundancy = {"u":(-np.pi,np.pi)}
        else:
            dims =self.geometry.dims
            self.redundancy = {"x":(-dims[0]/2,dims[0]/2),
                               "y":(-dims[1]/2,dims[1]/2),
                               "z":(dims[2]/2,dims[2]/2),
                               "u":(-np.pi,np.pi)}


##
# @class FramedPoint
# @brief ActionPoint with full orientation constraint
class FramedPoint(ActionPoint):
    def update_redundancy(self):
        if self.point is not None:
            self.redundancy = {}
        else:
            dims =self.geometry.dims
            self.redundancy = {"x":(-dims[0]/2,dims[0]/2),
                               "y":(-dims[1]/2,dims[1]/2),
                               "z":(dims[2]/2,dims[2]/2)}


################################# USABLE CLASS #########################################


##
# @class PlacePoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.PlacePlane
class PlacePoint(DirectedPoint):
    ctype=ConstraintType.Place


##
# @class FramePoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.FramedTool
class FramePoint(FramedPoint):
    ctype=ConstraintType.Frame


##
# @class VacuumPoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.VacuumTool
class VacuumPoint(DirectedPoint):
    ctype=ConstraintType.Vacuum


##
# @class Grasp2Point
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.Gripper2Tool
class Grasp2Point(DirectedPoint):
    ctype=ConstraintType.Grasp2
    def update_redundancy(self):
        if self.point is not None:
            self.redundancy = {"w":(0.0,0.0)}
        else:
            dims =self.geometry.dims
            self.redundancy = {"x":(-dims[0]/2,dims[0]/2),
                               "y":(-dims[1]/2,dims[1]/2),
                               "z":(dims[2]/2,dims[2]/2),
                               "w":(0.0,0.0)}

##
# @class RollGrasp2Point
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.Gripper2Tool
class RollGrasp2Point(RollPoint):
    ctype=ConstraintType.Grasp2
    def update_redundancy(self):
        if self.point is not None:
            self.redundancy = {"u":(-np.pi,np.pi)}
        else:
            dims =self.geometry.dims
            self.redundancy = {"x":(-dims[0]/2,dims[0]/2),
                               "y":(-dims[1]/2,dims[1]/2),
                               "u":(-np.pi,np.pi)}


##
# @class SweepPoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.SweepTool
class SweepPoint(DirectedPoint):
    ctype=ConstraintType.Sweep

##
# @class SweepPoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.SweepTool
class SweepFrame(FramePoint):
    ctype=ConstraintType.Sweep

##
# @class KnobFrame
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.SweepTool
class KnobFrame(FramePoint):
    ctype=ConstraintType.Knob

##
# @class KnobFrame
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.SweepTool
class HingeFrame(FramePoint):
    ctype=ConstraintType.Hinge
##
# @class WayPoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.WayAgent
class WayPoint(DirectedPoint):
    ctype=ConstraintType.Waypoint

##
# @class WayFrame
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.WayFramer
class WayFrame(FramePoint):
    ctype=ConstraintType.Waypoint


##
# @class FixturePoint
# @brief ActionPoint for rnb-planning.src.pkg.planning.constraint.constraint_actor.FixtureSlot
class FixturePoint(FramedPoint):
    ctype=ConstraintType.Fixture


##
# @class    SubjectType
class SubjectType(Enum):
    OBJECT = 0
    TASK = 1


##
# @class    Subject
# @brief    Base class for state subject definitions
# @remark   Subject consist of parent geometry and action points.
#           action_points are reference points for action, which are bound to actors.
#           binding represents current binding state with an actor.
class Subject:
    ## @brief SubjectType
    stype = None
    def __init__(self):
        ## @brief name of object
        self.oname = None
        ## @brief geometry instance for the object (rnb-planning.src.pkg.geometry.geometry.GeometryItem)
        self.geometry = None
        ## @brief dictionary of action points {point name: rnb-planning.src.pkg.planning.constraint.constraint_common.ActionPoint}
        self.action_points_dict = {}
        ## @brief dictionary of sub-binder points {point name: rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor}
        self.sub_binders_dict = {}
        ## @brief object's BindingTransform
        self.binding = BindingTransform(self, None, None, None)
        ## @brief State variables except for object binding
        self.state_param = None
        ## @brief keyword arguments for save&load
        self.kwargs = {}
        raise(NotImplementedError("AbstractObject is abstract class"))

    ##
    # @brief (prototype) set state param
    # @param binding BindingTransform
    # @param state_param
    @abstractmethod
    def set_state(self, binding, state_param):
        pass

    ##
    # @brief (prototype) get updated object-level state_param component
    @abstractmethod
    def get_updated_state_param(self, btf, state_param):
        pass

    ##
    # @brief (prototype) get available bindings from current binding state
    # @param from_binding current binding (subject name, handle name, actor name, actor root geometry name)
    # @param to_node_item desired node item
    # @param actor_dict
    #           dictionary of binder {binder_name: rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor}
    # @param Q_dict dictionary of joint values {joint_name: value}
    # @return list of available bindings [(handle name, actor name, actor root geometry name), ...]
    @abstractmethod
    def get_available_bindings(self, from_binding, to_node_item, actor_dict, Q_dict):
        pass

    ##
    # @brief (prototype) get object-level node component
    @classmethod
    @abstractmethod
    def get_node_component(cls, btf, state_param):
        pass

    ##
    # @brief (prototype) get object-level node component
    @abstractmethod
    def get_neighbor_node_component_list(self, node_tem, pscene):
        return []

    ##
    # @brief (prototype) get all object-level node component
    @abstractmethod
    def get_all_node_components(self, pscene):
        pass

    ##
    # @brief update geometries of sub-action points. You must call this after changing location of subject geometry
    def update_sub_points(self):
        gfams = self.geometry.get_family()
        for ap in self.action_points_dict.values():     # you should update action points here
            assert ap.geometry.name in gfams, "The geometries of action points should be in the family of subject geometry {} - {}".format(self.oname, ap.name)
            ap.update_handle()
        for bp in self.sub_binders_dict.values():     # you should update action points here
            assert bp.geometry.name in gfams, "The geometries of action points should be in the family of subject geometry {} - {}".format(self.oname, bp.name)
            bp.update_handle()

    ##
    # @brief make constraints. by default, empty list.
    # @remark whether to apply constarint or not is decided with previous and next bindings
    # @param binding_from previous binding
    # @param binding_to next binding
    def make_constraints(self, binding_from, binding_to, tol=None):
        return []

    ##
    # @brief get conflicting handles to build efficient search tree
    # @param handle name
    def get_conflicting_points(self, hname):
        return [hname]

    ##
    # @brief get initial binding - for object, usually no binding is initial state
    # @param actor_dict dictionary of binder {binder_name: rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor}
    # @param Q_dict dictionary of joint values {joint_name: value}
    # @return binding (subject name, handle name, actor name, actor root geometry name)
    def get_initial_chain(self, actor_dict, Q_dict):
        margin_max = -1e10
        max_point = None
        max_binder = None
        binder_gname = None
        binder_T_dict = {bname: binder.get_tf_handle(Q_dict) for bname, binder in actor_dict.items()}
        self_family = self.geometry.get_family()
        ## find best binding between object and binders
        for kpt, handle in self.action_points_dict.items():
            handle_T = handle.get_tf_handle(Q_dict)

            for bname, binder in actor_dict.items():
                if binder.check_available(Q_dict):
                    binder_T = binder_T_dict[bname]
                    if binder.geometry.name in self_family or not binder.check_pair(handle):
                        continue
                    binder_redundancy = binder.get_redundancy()
                    handle_redundancy = handle.get_redundancy()
                    margins = get_binding_margins(handle_T, binder_T, handle_redundancy, binder_redundancy,
                                                  rot_scale=1e-2)
                    margin_min = np.min(margins)
                    if margin_min > margin_max:
                        margin_max = margin_min
                        max_point = handle.name
                        max_binder = bname
                        binder_gname = actor_dict[bname].geometry.name
        return BindingChain(self.oname, max_point, max_binder, binder_gname)

    ##
    # @brief get state param
    # @return item for state_param
    def get_state_param(self):
        return deepcopy(self.state_param)

    def get_args(self):
        return {"name": self.oname,
                "gname": self.geometry.name,
                "type": self.__class__.__name__,
                "action_points_dict": {
                    apname: ap.get_args()
                    for apname, ap in self.action_points_dict.items()
                },
                "sub_binders": sorted(self.sub_binders_dict.keys()),
                "kwargs": self.kwargs
                }


##
# @class AbstractTask
# @brief Base class for task definition
class AbstractTask(Subject):
    stype = SubjectType.TASK
    def __init__(self):
        clearance = []
        raise(NotImplementedError("AbstractTask is abstract class"))


##
# @class WaypointTask
# @brief sequential waypoint reaching
class WaypointTask(AbstractTask):

    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param action_points_dict pre-defined action points as dictionary
    # @param one_direction set this value to False to enable bi-directional waypoint transfer
    def __init__(self, oname, geometry, action_points_dict, sub_binders_dict=None, tol=1e-3, one_direction=True):
        self.oname = oname
        self.geometry = geometry
        self.action_points_dict = action_points_dict
        self.action_points_order = sorted(self.action_points_dict.keys())
        self.action_point_len = len(self.action_points_order)
        self.sub_binders_dict = {} if sub_binders_dict is None else sub_binders_dict
        self.state_param = np.zeros(self.action_point_len, dtype=np.bool)
        self.binding = BindingTransform(self, None, None, None)
        self.tol = tol
        self.one_direction = one_direction
        self.kwargs = {"tol": tol}

    ##
    # @brief set object binding state and update location
    # @param binding BindingTransform
    # @param state_param list of done-mask
    def set_state(self, binding, state_param=None):
        self.binding = deepcopy(binding)
        if self.one_direction and state_param is not None:
            self.state_param = np.copy(state_param)
        else:
            self.state_param = np.zeros(self.action_point_len, dtype=np.bool)
        if self.binding.chain.handle_name is not None:
            if self.binding.chain.handle_name in self.action_points_order:
                self.state_param[:self.action_points_order.index(self.binding.chain.handle_name)+1] = True
            else:
                raise(RuntimeError("Undefined handle {}".format(self.binding.chain.handle_name)))
        self.update_sub_points()

    ##
    # @brief get object-level state_param component
    def get_updated_state_param(self, btf, state_param):
        if self.one_direction and state_param is not None:
            state_param = np.copy(state_param)
        else:
            state_param = np.zeros(self.action_point_len, dtype=np.bool)
        if btf.chain.handle_name is not None:
            state_param[:self.action_points_order.index(btf.chain.handle_name) + 1] = True
        return state_param

    ##
    # @brief get available bindings from current binding state
    # @param from_binding current binding (subject name, handle name, actor name, actor root geometry name)
    # @param to_node_item desired node item
    # @param actor_dict
    #           dictionary of binder {binder_name: rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor}
    # @param Q_dict dictionary of joint values {joint_name: value}
    # @return list of available bindings [(handle name, actor name, actor root geometry name), ...]
    def get_available_bindings(self, from_binding, to_node_item, actor_dict, Q_dict):
        ap_dict = self.action_points_dict
        apk_exclude = self.get_conflicting_points(from_binding[1])
        bd_exclude = from_binding[-2]
        if to_node_item>0:
            apk = self.action_points_order[to_node_item - 1]
        else:
            apk = self.action_points_order[0]
        ap_list = [ap_dict[apk]] if apk not in apk_exclude else []
        ctypes = [ap.ctype for ap in ap_list]
        bd_list = [actor for actor in actor_dict.values() if
                   actor.ctype in ctypes]  # bd_exclude is ignored as previous binding is re-used in sweep
        for bd in bd_list:
            self.geometry.gscene.link_control_map[bd.geometry.link_name]

        available_bindings = []
        for bd in bd_list:
            for ap in ap_list:
                if bd.check_pair(ap):
                    available_bindings.append((ap.name, bd.name, bd.geometry.name))
        if not available_bindings:
            print("=================================")
            print("=================================")
            print("=================================")
            print("Not available:{}-{}".format(self.oname, to_node_item))
            print("np_exclude:{}".format(apk_exclude))
            print("bd_exclude:{}".format(bd_exclude))
            print("=================================")
            print("=================================")
            print("=================================")
        return available_bindings

    ##
    # @brief get initial binding - for task, usually no binding is initial state
    # @param actor_dict dictionary of binder {binder_name: rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor}
    # @param Q_dict dictionary of joint values {joint_name: value}
    # @return binding (subject name, handle name, actor name, actor root geometry name)
    def get_initial_chain(self, actor_dict, Q_dict):
        return BindingChain(self.oname, None, None, None)

    ##
    # @brief get object-level node component (finished waypoint count)
    @classmethod
    def get_node_component(cls, btf, state_param):
        if btf.chain.handle_name is not None:
            return int(np.sum(state_param))
        else:
            return 0

    ##
    # @brief get all object-level node component
    def get_all_node_components(self, pscene):
        return list(range(len(self.state_param) + 1))

    ##
    # @brief get object-level neighbor component (detach or next waypoint)
    def get_neighbor_node_component_list(self, node_tem, pscene):
        neighbor = []
        if (not self.one_direction) and node_tem > 1:
            neighbor.append(node_tem - 1)
        if node_tem < len(self.state_param):
            neighbor.append(node_tem + 1)
        return neighbor

    ##
    # @brief use this function to create explicit node-edge checking rule for TaskRRT
    @classmethod
    def make_unstopable_rule(cls, pscene, snames=None):
        if snames is None:
            snames = [sname for sname, stype
                      in zip(pscene.subject_name_list, pscene.subject_type_list)
                      if isinstance(stype, cls)]
        subjects = [pscene.subject_dict[sname] for sname in snames]
        idc = [pscene.subject_name_list.index(sname) for sname in snames]
        def check_edge(pscene, node, leaf):
            ok = True
            for idx, subject in zip(idc, subjects):
                if node[idx] > 0 and node[idx] < subject.action_point_len:
                    ok = ok and node[idx]+1 == leaf[idx]
            return ok
        return check_edge


##
# @class SweepTask
# @brief sweep action points in alphabetical order
# @remark   state_param: boolean vector of which each element represents if each waypoint is covered or not
#           node_item: number of covered waypoints
class SweepTask(WaypointTask):

    ##
    # @brief make constraints. by default, empty list.
    # @remark constraint is applied when using same binding
    # @param binding_from previous binding
    # @param binding_to next binding
    def make_constraints(self, binding_from, binding_to, tol=None):
        if binding_from is not None and binding_from.actor_name == binding_to.actor_name:
            return [MotionConstraint([self.geometry], True, True, tol=tol if tol is not None else self.tol)]
        else:
            return []


##
# @class SweepLineTask
# @brief sweep action points in alphabetical order
# @remark   state_param: boolean vector of which each element represents if each waypoint is covered or not
#           node_item: number of covered waypoints
class SweepLineTask(SweepTask):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param action_points_dict pre-defined action points as dictionary
    def __init__(self, oname, geometry, action_points_dict, sub_binders_dict=None,
                 geometry_vertical=None, tol=1e-3, clearance=None, **kwargs):
        if isinstance(geometry_vertical, str):
            geometry_vertical = geometry.gscene.NAME_DICT[geometry_vertical]
        clearance_names = []
        if isinstance(clearance, list):
            clearance_gtem = []
            for gtem in clearance:
                if isinstance(gtem, str):
                    gtem = geometry.gscene.NAME_DICT[gtem]
                clearance_gtem.append(gtem)
                clearance_names.append(gtem.name)
            clearance = clearance_gtem
        SweepTask.__init__(self, oname, geometry, action_points_dict, sub_binders_dict=sub_binders_dict, tol=tol, **kwargs)
        self.kwargs = {"geometry_vertical":
                           geometry_vertical.name if geometry_vertical is not None else geometry_vertical,
                       "tol": tol, "clearance": clearance_names}

        self.fix_direction = True
        if clearance is None:
            self.clearance = []
        else:
            self.clearance = clearance
        for gtem in self.clearance:
            if gtem.collision:
                print("[WARNING] clearance geometries for SWeepLineTask should be set collision-free by default"
                      " - setting it False")
                gtem.collision = False
        if geometry_vertical is not None:
            self.geometry_vertical = geometry_vertical
            self.Rot_vertical = np.matmul(geometry.orientation_mat.transpose(), geometry_vertical.orientation_mat)
            self.center_dir = self.Rot_vertical[np.argmax(np.diag(self.Rot_vertical)), :]
        else:
            # centers in local coordinate in self.geometry
            wp_centers = [np.matmul(SE3_inv(self.geometry.Toff), sp.Toff_lh)[:3,3]
                          for sp in self.action_points_dict.values()]
            assert len(wp_centers) == 2, "We only consider 2-waypoint line sweep"
            center_dist = np.linalg.norm(wp_centers[1]-wp_centers[0])
            if center_dist==0:
                center_dist = 1
                self.center_dir = np.array([1,0,0])
            else:
                self.center_dir = (wp_centers[1]-wp_centers[0])/center_dist
            self.Rot_vertical = Rotation.from_rotvec(self.center_dir * np.pi / 2).as_dcm()
            self.geometry_vertical = geometry.gscene.create_safe(GEOTYPE.PLANE,
                                                                 "_".join([oname]+self.action_points_order),
                                                                 link_name=self.geometry.link_name,
                                                                 dims=(center_dist*1.5,center_dist*1.5,1e-6),
                                                                 center=np.mean(wp_centers, axis=0),
                                                                 rpy=Rot2rpy(self.Rot_vertical),
                                                                 color=(0.8,0.2,0.2,0.2), display=False,
                                                                 fixed=self.geometry.fixed, collision=False,
                                                                 parent=self.geometry.name)

    ##
    # @brief make constraints. by default, empty list.
    # @remark constraint is applied when using same binding
    # @param binding_from previous binding
    # @param binding_to next binding
    def make_constraints(self, binding_from, binding_to, tol=None):
        if binding_from is not None and binding_from.actor_name == binding_to.actor_name:
            tol = tol if tol is not None else self.tol
            if self.fix_direction:
                return [MotionConstraint([self.geometry], True, True, tol=tol),
                        MotionConstraint([self.geometry_vertical], True, False, tol=tol),
                        MotionConstraint([self.geometry_vertical], True, False,
                                         T_tool_offset=SE3(np.identity(3), self.center_dir*tol*10), tol=tol)]
            else:
                return [MotionConstraint([self.geometry], True, True, tol=tol),
                        MotionConstraint([self.geometry_vertical], True, False, tol=tol)]
        else:
            return []


##
# @class HingeTask
# @brief sweep action points in alphabetical order
# @remark   state_param: boolean vector of which each element represents if each waypoint is covered or not
#           node_item: number of covered waypoints
class HingeTask(AbstractTask):
    # @param binding_pairs list of tuple (handle, attach frame) [(handle, binder), ...]
    def __init__(self, oname, geometry, binding_pairs, sub_binders_dict=None, tol=1e-3):
        self.T0 = np.copy(geometry.Toff)
        self.link0 = deepcopy(geometry.link_name)
        self.oname = oname
        self.geometry = geometry
        self.binding_pairs = binding_pairs
        self.action_points_dict = {ap.name: ap for ap, bp in binding_pairs}
        self.action_points_order = [ap.name for ap, bp in binding_pairs]
        self.action_point_len = len(self.action_points_order)
        self.sub_binders_dict = {} if sub_binders_dict is None else sub_binders_dict
        for ap, bp in binding_pairs:
            self.sub_binders_dict[bp.name] = bp
        self.state_param = self.action_points_order[0]
        self.binding = BindingTransform(self, None, None, None)
        self.tol = tol
        self.kwargs = {"tol": tol}

    ##
    # @brief (prototype) set state param
    # @param binding BindingTransform
    # @param state_param
    def set_state(self, binding, state_param):
        self.binding = deepcopy(binding)
        self.state_param = deepcopy(self.state_param)
        for ap, bp in self.binding_pairs:
            bp.available = False
        chain = self.binding.chain
        if chain.handle_name in self.action_points_order:
            i_bd = self.action_points_order.index(chain.handle_name)
            self.binding_pairs[i_bd][1].available = True
        else:
            self.binding_pairs[0][1].available = True
        self.update_sub_points()

    ##
    # @brief (prototype) get updated object-level state_param component
    def get_updated_state_param(self, btf, state_param):
        chain = btf.chain
        if chain.handle_name in self.action_points_order:
            return chain.handle_name
        else:
            return deepcopy(state_param)

    ##
    # @brief (prototype) get available bindings from current binding state
    # @param from_binding current binding (subject name, handle name, actor name, actor root geometry name)
    # @param to_node_item desired node item
    # @param actor_dict
    #           dictionary of binder {binder_name: rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor}
    # @param Q_dict dictionary of joint values {joint_name: value}
    # @return list of available bindings [(handle name, actor name, actor root geometry name), ...]
    def get_available_bindings(self, from_binding, to_node_item, actor_dict, Q_dict):
        if to_node_item in self.action_points_order:
            ap_list = [self.action_points_dict[to_node_item]]
        else:
            print("ERROR: Unknown node item {} for {}".format(to_node_item, self.oname))

        ctypes = [ap.ctype for ap in ap_list]
        bd_list = [actor for actor in actor_dict.values() if
                   actor.ctype in ctypes]  # bd_exclude is ignored as previous binding is re-used in sweep
        for bd in bd_list:
            self.geometry.gscene.link_control_map[bd.geometry.link_name]

        available_bindings = []
        for bd in bd_list:
            for ap in ap_list:
                if bd.check_pair(ap):
                    available_bindings.append((ap.name, bd.name, bd.geometry.name))
        if not available_bindings:
            print("=================================")
            print("=================================")
            print("=================================")
            print("Not available:{}-{}".format(self.oname, to_node_item))
            print("=================================")
            print("=================================")
            print("=================================")
        return available_bindings

    ##
    # @brief (prototype) get object-level node component
    @classmethod
    def get_node_component(cls, btf, state_param):
        return state_param

    ##
    # @brief (prototype) get object-level node component
    def get_neighbor_node_component_list(self, node_tem, pscene):
        nodes = deepcopy(self.action_points_order)
        if node_tem in self.action_points_order:
            nodes.remove(node_tem)
        return nodes

    ##
    # @brief (prototype) get all object-level node component
    def get_all_node_components(self, pscene):
        return deepcopy(sorted(set(self.action_points_order)))

    ##
    # @brief make constraints. by default, empty list.
    # @remark constraint is applied when using same binding
    # @param binding_from previous binding
    # @param binding_to next binding
    def make_constraints(self, binding_from, binding_to, tol=None):
        return "Constraint not implemented yet. Use MoveitPlanner.incremental_constraint_motion=True"


##
# @class KnobTask
# @brief sweep action points in alphabetical order
# @remark   state_param: boolean vector of which each element represents if each waypoint is covered or not
#           node_item: number of covered waypoints
class KnobTask(HingeTask):
    def __init__(self, oname, geometry, binding_pairs, knob_plug, sub_binders_dict=None, tol=1e-3):
        self.knob_plug = knob_plug
        HingeTask.__init__(self, oname, geometry, binding_pairs, sub_binders_dict=sub_binders_dict, tol=tol)
    ##
    # @brief (prototype) set state param
    # @param binding BindingTransform
    # @param state_param
    def set_state(self, binding, state_param):
        if self.knob_plug is not None:
            if binding.chain.handle_name == self.action_points_order[-1]:
                self.knob_plug.available=True
            else:
                self.knob_plug.available=False
        HingeTask.set_state(self, binding, state_param)


##
# @class AbstractObject
# @brief Base class for objects with defined action points (handles)
# @remark get_conflicting_points and register_binders should be implemented with child classes
class AbstractObject(Subject):
    stype = SubjectType.OBJECT

    ##
    # @brief get available bindings from current binding state
    # @param from_binding current binding chain (subject name, handle name, actor name, actor root geometry name)
    # @param to_node_item desired node item
    # @param actor_dict
    #           dictionary of binder {binder_name: rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor}
    # @param Q_dict dictionary of joint values {joint_name: value}
    # @return list of available bindings [(handle name, actor name, actor root geometry name), ...]
    def get_available_bindings(self, from_binding, to_node_item, actor_dict, Q_dict):
        ap_dict = self.action_points_dict
        apk_exclude = self.get_conflicting_points(from_binding[1])
        bd_exclude = from_binding[2]
        apk_list = ap_dict.keys()
        ap_list = [ap_dict[apk] for apk in apk_list if apk not in apk_exclude]
        bd_list = [binder for bname, binder in actor_dict.items()
                   if (binder.geometry.name == to_node_item
                       and binder.check_available(Q_dict)
                       and bname != bd_exclude)]

        available_bindings = []
        for bd in bd_list:
            for ap in ap_list:
                if bd.check_pair(ap):
                    available_bindings.append((ap.name, bd.name, bd.geometry.name))
        if not available_bindings:
            print("=================================")
            print("=================================")
            print("=================================")
            print("Not available:{}-{}".format(self.oname,to_node_item))
            print("ap_exclude:{}".format(apk_exclude))
            print("bd_exclude:{}".format(bd_exclude))
            print("ap_list:{}".format(ap_list))
            print("bd_list:{}".format(bd_list))
            print("=================================")
            print("=================================")
            print("=================================")
        return available_bindings

    ##
    # @brief set object binding state and update location
    # @param binding BindingTransform
    # @param state_param (link name, offset transformation in 4x4 matrix)
    def set_state(self, binding, state_param=None):
        assert binding.chain.subject_name == self.oname, "wrong binding given {} <- {}".format(
            self.oname, binding.chain.subject_name)
        self.binding = deepcopy(binding)
        self.state_param = deepcopy(state_param)
        self.geometry.set_offset_tf(binding.T_lao[:3, 3], binding.T_lao[:3, :3])
        self.geometry.set_link(binding.actor_link)
        self.update_sub_points()

    ##
    # @brief function prototype to register pre-defined binders
    # @param planning_scene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param _type          subclass of rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    @abstractmethod
    def register_binders(self, planning_scene, _type):
        pass

    ##
    # @brief get object-level state_param update
    def get_updated_state_param(self, btf, state_param):
        return state_param

    ##
    # @brief get object-level node component (actor root geometry name)
    @classmethod
    def get_node_component(cls, btf, state_param):
        return btf.chain.actor_root_gname

    ##
    # @brief    get object-level neighbor component (other available actor root geometry name)
    #           other binding point in the scene
    # @param    node_tem    geometry name of binder currently attached
    def get_neighbor_node_component_list(self, node_tem, pscene):
        ctrl_binders, uctrl_binders = pscene.separate_active_binders([ap.ctype for ap in self.action_points_dict.values()])
        next_node_component_list = [pscene.actor_dict[bname].geometry.name for bname in ctrl_binders]
        if any([bname in ctrl_binders for bname in pscene.geometry_actor_dict[node_tem]]): # if any of currently attached binder geometry's binder is active, it's active
            next_node_component_list += [pscene.actor_dict[bname].geometry.name for bname in uctrl_binders] # thus we can add move it to uncontrolled binders
        if node_tem in next_node_component_list:
            next_node_component_list.remove(node_tem)
        return next_node_component_list

    ##
    # @brief get all object-level node component
    def get_all_node_components(self, pscene):
        return sorted(set([v.geometry.name for k,v in pscene.actor_dict.items()
                           if any(v.check_pair(ap) for ap in self.action_points_dict.values())]))


################################# USABLE CLASS #########################################


##
# @class CustomObject
# @brief Customizable object that handles can be defined by user
class CustomObject(AbstractObject):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param action_points_dict pre-defined action points as dictionary
    def __init__(self, oname, geometry, action_points_dict, sub_binders_dict=None):
        self.oname = oname
        self.geometry = geometry
        self.action_points_dict = action_points_dict
        self.sub_binders_dict = {} if sub_binders_dict is None else sub_binders_dict
        self.binding = BindingTransform(self, None, None, None)
        self.state_param = None

    ##
    # @brief do nothing
    def register_binders(self, planning_scene, _type):
        pass


##
# @class SingleHandleObject
# @brief Object with single defined handle
class SingleHandleObject(AbstractObject):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param action_point pre-defined single action point
    def __init__(self, oname, geometry, action_point=None, action_points_dict=None, sub_binders_dict=None):
        self.oname = oname
        self.geometry = geometry
        assert action_point is not None or action_points_dict is not None, "Error: Either action_point or action_points_dict should be given"
        self.action_points_dict = {action_point.name: action_point} if action_points_dict is None else action_points_dict
        self.sub_binders_dict = {} if sub_binders_dict is None else sub_binders_dict
        self.binding = BindingTransform(self, None, None, None)
        self.state_param = None

    ##
    # @brief do nothing
    def register_binders(self, planning_scene, _type):
        pass


##
# @class BoxObject
# @brief Box object with hexahedral action points
class BoxObject(AbstractObject):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param hexahedral If True, all hexahedral points are defined. Otherwise, only top and bottom points are defined
    def __init__(self, oname, geometry, hexahedral=True, CLEARANCE=1e-3,
                 GRASP_WIDTH_MIN=0.04, GRASP_WIDTH_MAX=0.06, GRASP_DEPTH_MIN=0.025, GRASP_DEPTH_MAX=0.025,
                 action_points_dict=None, sub_binders_dict=None):
        self.oname = oname
        self.geometry = geometry
        self.CLEARANCE = CLEARANCE
        self.GRASP_WIDTH_MIN = GRASP_WIDTH_MIN
        self.GRASP_WIDTH_MAX = GRASP_WIDTH_MAX
        self.GRASP_DEPTH_MIN = GRASP_DEPTH_MIN
        self.GRASP_DEPTH_MAX = GRASP_DEPTH_MAX
        self.action_points_dict = {}
        self.conflict_dict = {}
        self.sub_binders_dict = {} if sub_binders_dict is None else sub_binders_dict
        self.hexahedral = hexahedral
        if action_points_dict is None:
            self.add_place_points(self.geometry, CLEARANCE=CLEARANCE)
            self.add_grip_points(self.geometry, GRASP_WIDTH_MIN=GRASP_WIDTH_MIN, GRASP_WIDTH_MAX=GRASP_WIDTH_MAX,
                                   GRASP_DEPTH_MIN=GRASP_DEPTH_MIN, GRASP_DEPTH_MAX=GRASP_DEPTH_MAX)
        else:
            self.action_points_dict = action_points_dict

        self.set_conflict_dict()
        self.binding = BindingTransform(self, None, None, None)
        self.state_param = None
        self.kwargs = dict(hexahedral=hexahedral, CLEARANCE=CLEARANCE,
                           GRASP_WIDTH_MIN=GRASP_WIDTH_MIN, GRASP_WIDTH_MAX=GRASP_WIDTH_MAX,
                           GRASP_DEPTH_MIN=GRASP_DEPTH_MIN, GRASP_DEPTH_MAX=GRASP_DEPTH_MAX)

    ##
    # @brief add action points to given box geometry
    def add_place_points(self, gbox, CLEARANCE=1e-3):
        Xhalf, Yhalf, Zhalf = np.divide(gbox.dims,2)+CLEARANCE
        self.action_points_dict.update({
            "top_p": PlacePoint("top_p", gbox, [0,0,Zhalf], [np.pi,0,0]),
            "bottom_p": PlacePoint("bottom_p", gbox, [0,0,-Zhalf], [0,0,0])
        })
        if self.hexahedral:
            self.action_points_dict.update({
                "right_p": PlacePoint("right_p", gbox, [Xhalf,0,0], [0,-np.pi/2,0]),
                "left_p": PlacePoint("left_p", gbox, [-Xhalf,0,0], [0,np.pi/2,0]),
                "front_p": PlacePoint("front_p", gbox, [0,-Yhalf,0], [-np.pi/2,0,0]),
                "back_p": PlacePoint("back_p", gbox, [0,Yhalf,0], [np.pi/2,0,0])
            })

    ##
    # @brief add 2-finger grasping points to given box geometry
    def add_grip_points(self, gbox,
                          GRASP_WIDTH_MIN=0.04, GRASP_WIDTH_MAX=0.06,
                          GRASP_DEPTH_MIN=0.025, GRASP_DEPTH_MAX=0.025):
        dims = gbox.dims
        dims_hf = np.divide(gbox.dims, 2)
        for k_dir, rpy in DIR_RPY_DICT.items():
            R = Rot_rpy(rpy)
            for i_r in range(4):
                ggname = "{}_{}_{}_g".format(gbox.name, k_dir, i_r)
                Rapproach = np.matmul(R, Rot_axis(3,
                                                  i_r * np.pi / 2))  # z: approaching vector forward, y: pinching axis
                Rgrip = np.matmul(Rapproach,
                                  Rot_axis(1, -np.pi / 2))  # y: approaching vector backward, z: pinching axis
                rpy_grip = Rot2rpy(Rgrip)
                redundant_axis, approach_vec, pinch_axis = Rgrip[:, 0], Rgrip[:, 1], Rgrip[:, 2]
                grasp_width = np.abs(np.dot(pinch_axis, dims))
                if not (GRASP_WIDTH_MIN <= grasp_width <= GRASP_WIDTH_MAX):
                    continue
                offset_ref = np.abs(np.dot(approach_vec, dims_hf))
                offset_max = max(offset_ref - GRASP_DEPTH_MIN, 0)
                offset_min = max(offset_ref - GRASP_DEPTH_MAX, 0)
                offset_range = offset_max - offset_min
                redundant_len = max(np.abs(np.dot(redundant_axis, dims)) - GRASP_DEPTH_MIN * 2, 0)
                gcenter = np.matmul(Rgrip, [[0], [(offset_max + offset_min) / 2], [0]])[:, 0]
                if offset_range < 1e-3 and redundant_len < 1e-3:
                    gpoint = Grasp2Point(ggname, gbox, gcenter, rpy_grip)
                else:
                    ggtem = gbox.gscene.create_safe(GEOTYPE.BOX, ggname, link_name=gbox.link_name,
                                                    dims=(redundant_len, offset_range, 0), center=gcenter, rpy=rpy_grip,
                                                    color=(0.0, 0.8, 0.0, 0.5), display=gbox.display, fixed=gbox.fixed,
                                                    collision=False, parent=gbox.name)
                    gpoint = Grasp2Point(ggname, ggtem, None, (0, 0, 0))
                self.action_points_dict.update({ggname: gpoint})

    ##
    # @brief reset conflicting handle dictionary
    def set_conflict_dict(self):
        ap_names = sorted(self.action_points_dict.keys())
        dir_keys = sorted(OPPOSITE_DICT.keys())
        self.conflict_dict = {}
        for hname in ap_names:
            hnames_conflict = []
            for dkey in dir_keys:
                if dkey in hname:
                    hnames_conflict += [hname_ for hname_ in ap_names if dkey in hname_]
            self.conflict_dict[hname] = hnames_conflict

    ##
    # @brief get conflicting handles in hexahedral geometry
    # @param handle name
    def get_conflicting_points(self, hname):
        return self.conflict_dict[hname]

    ##
    # @brief register hexahedral binders
    # @param planning_scene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param _type          subclass of rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    def register_binders(self, planning_scene, _type, geometry=None):
        gscene = planning_scene.gscene
        if geometry is None:
            geometry = self.geometry
        gname = geometry.name
        dims = geometry.dims
        for k in DIR_RPY_DICT.keys():
            if not self.hexahedral and k not in ["top", "bottom"]:
                continue
            rpy = DIR_RPY_DICT[k]
            point = tuple(-np.multiply(DIR_VEC_DICT[k], dims)/2)
            bname = "{}_{}".format(gname, k)
            R = Rot_rpy(rpy)
            dims_new = np.abs(np.matmul(R.transpose(), dims))
            dims_new[2] = 1e-6
            gscene.create_safe(GEOTYPE.BOX, bname,
                               link_name=geometry.link_name,
                               dims=dims_new, center=point, rpy=rpy,
                               display=False, collision=False, fixed=geometry.fixed, parent=gname)
            self.sub_binders_dict[bname] = planning_scene.create_binder(bname=bname, gname=bname, _type=_type,
                                                                        point=None)





##
# @class CylinderObject
# @brief Cylinder object
class CylinderObject(AbstractObject):
    ##
    # @param oname object's name
    # @param geometry parent geometry
    # @param hexahedral If True, all hexahedral points are defined. Otherwise, only top and bottom points are defined
    def __init__(self, oname, geometry, CLEARANCE=1e-3,
                 GRASP_WIDTH_MIN=0.04, GRASP_WIDTH_MAX=0.06, GRASP_DEPTH_MIN=0.025, GRASP_DEPTH_MAX=0.025,
                 action_points_dict=None, sub_binders_dict=None):
        self.oname = oname
        self.geometry = geometry
        self.CLEARANCE = CLEARANCE
        self.GRASP_WIDTH_MIN = GRASP_WIDTH_MIN
        self.GRASP_WIDTH_MAX = GRASP_WIDTH_MAX
        self.GRASP_DEPTH_MIN = GRASP_DEPTH_MIN
        self.GRASP_DEPTH_MAX = GRASP_DEPTH_MAX
        self.action_points_dict = {}
        self.conflict_dict = {}
        self.sub_binders_dict = {} if sub_binders_dict is None else sub_binders_dict
        if action_points_dict is None:
            self.add_place_points(self.geometry, CLEARANCE=CLEARANCE)
            self.add_grip_points(self.geometry, GRASP_WIDTH_MIN=GRASP_WIDTH_MIN, GRASP_WIDTH_MAX=GRASP_WIDTH_MAX,
                                   GRASP_DEPTH_MIN=GRASP_DEPTH_MIN, GRASP_DEPTH_MAX=GRASP_DEPTH_MAX)
        else:
            self.action_points_dict = action_points_dict

        self.set_conflict_dict()
        self.binding = BindingTransform(self, None, None, None)
        self.kwargs = dict(CLEARANCE=CLEARANCE,
                           GRASP_WIDTH_MIN=GRASP_WIDTH_MIN, GRASP_WIDTH_MAX=GRASP_WIDTH_MAX,
                           GRASP_DEPTH_MIN=GRASP_DEPTH_MIN, GRASP_DEPTH_MAX=GRASP_DEPTH_MAX)

    ##
    # @brief add action points to given box geometry
    def add_place_points(self, gbox, CLEARANCE=1e-3, bot_only=True):
        Xhalf, Yhalf, Zhalf = np.divide(gbox.dims,2)+CLEARANCE
        self.action_points_dict.update({
            "bottom_p": PlacePoint("bottom_p", gbox, [0,0,-Zhalf], [0,0,0])
        })
        if not bot_only:
            self.action_points_dict.update({
                "top_p": PlacePoint("top_p", gbox, [0,0,Zhalf], [np.pi,0,0])
            })

    ##
    # @brief add 2-finger grasping points to given box geometry
    def add_grip_points(self, gbox,
                          GRASP_WIDTH_MIN=0.04, GRASP_WIDTH_MAX=0.06,
                          GRASP_DEPTH_MIN=0.025, GRASP_DEPTH_MAX=0.025):
        dims = gbox.dims
        dims_hf = np.divide(gbox.dims, 2)
        ggname = "{}_{}_g".format(gbox.name, "side")
        Rapproach = Rot_rpy([0,-np.pi/2,0])
        Rgrip = np.matmul(Rapproach,
                          Rot_axis(1, -np.pi / 2))  # y: approaching vector backward, z: pinching axis
        rpy_grip = Rot2rpy(Rgrip)
        redundant_axis, approach_vec, pinch_axis = Rgrip[:, 0], Rgrip[:, 1], Rgrip[:, 2]
        grasp_width = np.abs(np.dot(pinch_axis, dims))

        if GRASP_WIDTH_MAX <= grasp_width:
            print("[WARNING] Cylinder too big for the gripper")
            return
        if grasp_width <= GRASP_WIDTH_MIN:
            print("[WARNING] Cylinder too small for the gripper")
            return
        redundant_len = max(np.abs(np.dot(redundant_axis, dims)) - GRASP_DEPTH_MIN * 2, 0)
        gcenter = np.matmul(Rgrip, [[0], [0], [0]])[:, 0]
        if redundant_len < 1e-3:
            gpoint = RollGrasp2Point(ggname, gbox, gcenter, rpy_grip)
        else:
            ggtem = gbox.gscene.create_safe(GEOTYPE.BOX, ggname, link_name=gbox.link_name,
                                            dims=(redundant_len, 0, 0), center=gcenter, rpy=rpy_grip,
                                            color=(0.0, 0.8, 0.0, 0.5), display=gbox.display, fixed=gbox.fixed,
                                            collision=False, parent=gbox.name)
            gpoint = RollGrasp2Point(ggname, ggtem, None, (0, 0, 0))
        self.action_points_dict.update({ggname: gpoint})

    ##
    # @brief reset conflicting handle dictionary
    def set_conflict_dict(self):
        ap_names = sorted(self.action_points_dict.keys())
        dir_keys = sorted(OPPOSITE_DICT.keys())
        self.conflict_dict = {}
        for hname in ap_names:
            hnames_conflict = []
            for dkey in dir_keys:
                if dkey in hname:
                    hnames_conflict += [hname_ for hname_ in ap_names if dkey in hname_]
            self.conflict_dict[hname] = hnames_conflict

    ##
    # @brief get conflicting handles in hexahedral geometry
    # @param handle name
    def get_conflicting_points(self, hname):
        return self.conflict_dict[hname]

    ##
    # @brief register hexahedral binders
    # @param planning_scene rnb-planning.src.pkg.planning.scene.PlanningScene
    # @param _type          subclass of rnb-planning.src.pkg.planning.constraint.constraint_actor.Actor
    def register_binders(self, planning_scene, _type, geometry=None):
        gscene = planning_scene.gscene
        if geometry is None:
            geometry = self.geometry
        gname = geometry.name
        dims = geometry.dims
        for k in DIR_RPY_DICT.keys():
            if not self.hexahedral and k not in ["top", "bottom"]:
                continue
            rpy = DIR_RPY_DICT[k]
            point = tuple(-np.multiply(DIR_VEC_DICT[k], dims)/2)
            bname = "{}_{}".format(gname, k)
            R = Rot_rpy(rpy)
            dims_new = np.abs(np.matmul(R.transpose(), dims))
            dims_new[2] = 1e-6
            gscene.create_safe(GEOTYPE.BOX, bname,
                               link_name=geometry.link_name,
                               dims=dims_new, center=point, rpy=rpy,
                               display=False, collision=False, fixed=geometry.fixed, parent=gname)
            self.sub_binders_dict[bname] = planning_scene.create_binder(bname=bname, gname=bname, _type=_type,
                                                                        point=point)


##
# @brief convert constraint type to handle type
def ctype_to_htype(cstr):
    if cstr == ConstraintType.Grasp2.name:
        return Grasp2Point
    elif cstr == ConstraintType.Frame.name:
        return FramePoint
    elif cstr == ConstraintType.Place.name:
        return PlacePoint
    elif cstr == ConstraintType.Vacuum.name:
        return VacuumPoint


##
# @brief convert object type string to object type
def otype_to_class(ostr):
    if ostr == 'BoxObject':
        return BoxObject
    if ostr == 'SingleHandleObject':
        return SingleHandleObject
    if ostr == 'CustomObject':
        return CustomObject
