from .stereo import *
from .geometry import *
from . constants import *


def detect_environment(aruco_map, dictionary, robot_tuples, env_dict, camT_dict={"cam0":np.identity(4)}, ref_name='floor'):
    xyz_rpy_robots = {}
    xyz_rvec_cams = {}
    env_gen_dict = {}
    while True:
        try:
            objectPose_dict, corner_dict, color_image, rs_image, rs_corner_dict = get_object_pose_dict_stereo(aruco_map,
                                                                                                              dictionary)

            for rtuple in robot_tuples:
                rname = rtuple[0]
                Tbr = get_T_rel(ref_name, rname, objectPose_dict)
                xyz_rpy_robots[rname] = T2xyzrpy(Tbr)

            T_bc = SE3_inv(objectPose_dict[ref_name])
            for cname, camT in camT_dict.items():
                xyz_rvec_cams[cname] = T2xyzrvec(np.matmul(T_bc, camT))

            for ename, ginfo in env_dict.items():
                env_gen_dict[ename] = (ginfo, T2xyzrvec(get_T_rel(ref_name, ename, objectPose_dict)))
            break
        except KeyError as e:
            print(e)
            break
        except Exception as e:
            print(e)
            pass
    return xyz_rpy_robots, xyz_rvec_cams, env_gen_dict, objectPose_dict, corner_dict, color_image

def add_objects_gen(graph, obj_gen_dict, color=(0.6,0.6,0.6,1), collision=True, link_name="world"):
    graph.add_geometry_items(link_name, [ogen[0](*ogen[1], name=oname, link_name=link_name, urdf_content=graph.urdf_content,
             color=color, collision=collision) for oname, ogen in obj_gen_dict.items()], fixed=True)

def add_cam_poles(graph, xyz_rvec_cams, color=(0.6,0.6,0.6,1), link_name="world"):
    graph.add_geometry_items(link_name,
                             [GeoBox(np.subtract(xyzrvec[0], [0,0,xyzrvec[0][2]/2+0.1]),
                                     (0.1,0.1,xyzrvec[0][2]),
                                     name="pole_{}".format(cname),
                                     link_name=link_name, urdf_content=graph.urdf_content,
                                     color=color, collision=True)
                              for cname, xyzrvec in xyz_rvec_cams.items()], fixed=True)

def detect_objects(movable_generators, aruco_map, dictionary):
    aruco_map_mv = {k: v for k, v in aruco_map.items() if k in movable_generators}
    objectPose_dict_mv, corner_dict_mv, color_image, rs_image, rs_corner_dict = get_object_pose_dict_stereo(
        aruco_map_mv, dictionary)
    return objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv

def register_objects(graph, objectPose_dict_mv, object_generators, binder_dict, object_dict, ref_tuple, link_name="world"):
    objectPose_dict_mv.update({ref_tuple[0]: ref_tuple[1]})
    T_mv_dict = {mname: get_T_rel(ref_tuple[0], mname, objectPose_dict_mv) for mname in object_generators if
                 mname in objectPose_dict_mv}
    xyz_rvec_mv_dict = {mname: T2xyzrvec(Tv) for mname, Tv in T_mv_dict.items()}
    for mname, mgen in object_generators.items():
        if mname in xyz_rvec_mv_dict:
            xyz_rvec = xyz_rvec_mv_dict[mname]
            graph.add_geometry_items(link_name,
                                     [mgen(*xyz_rvec, name=mname,
                                           link_name=link_name, urdf_content=graph.urdf_content, color=(0.3, 0.3, 0.8, 1),
                                           collision=True)],
                                     fixed=False)

    for bname, bkwargs in binder_dict.items():
        graph.register_binder(name=bname, **bkwargs)

    put_point_dict = {}
    for mtem, xyz_rvec in xyz_rvec_mv_dict.items():
        if mtem in object_dict:
            put_point_dict[mtem] = get_put_dir(Robj=Rotation.from_rotvec(xyz_rvec[1]).as_dcm(),
                                               dir_vec_dict=DIR_VEC_DICT) + "_p"
            graph.register_object(mtem, binding=(put_point_dict[mtem], "floor"), **object_dict[mtem])
            print("put_point_{}: {}".format(mtem, put_point_dict[mtem]))
    return T_mv_dict, put_point_dict
