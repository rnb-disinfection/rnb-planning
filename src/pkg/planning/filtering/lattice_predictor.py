import os
import sys
sys.path.append(os.path.join(os.environ["RNB_PLANNING_DIR"], "lib/latticizer"))
import latticizer as ltc
GJK_PATH_DEFAULT = os.path.join(os.environ["RNB_PLANNING_DIR"], "lib/openGJK/lib/openGJKlib.so")

from ...geometry.geometry import *

def make_point3_ltc(x, y, z):
    return ltc.Point3(x, y, z)

def get_point_list_ltc(point_rows_np):
    pl = ltc.PointList()
    for v in point_rows_np:
        pl.append(ltc.Point3(*v))
    return pl

##
# @class    Latticizer
# @brief    Latticize a geometry scene
class Latticizer:
    def __init__(self, WDH=(3, 3, 2), L_CELL=0.05, OFFSET_ZERO=(1.5, 1.5, 0.5), gjk_path=GJK_PATH_DEFAULT):
        self.WDH = WDH
        self.L_CELL = L_CELL
        self.OFFSET_ZERO = OFFSET_ZERO
        self.Nwdh = tuple(np.divide(WDH, L_CELL).astype(np.int))
        self.ltc_bp = ltc.Latticizer(gjk_path)
        self.ltc_bp.set_centers(*(self.Nwdh + (L_CELL,) + OFFSET_ZERO))
        self.ltc_bp.set_cell_vertices()
        ##
        # @brief colliding cell indexes
        self.coll_idx_dict = {}
        ##
        # @brief geometry scene, containing cell references
        self.gscene_ref = None

    ##
    # @brief clear colliding indexes
    # @param names
    def clear(self, names=None, names_except=[]):
        if names is None and len(names_except)==0:
            self.coll_idx_dict = {}
        elif names is None:
            names = self.coll_idx_dict.keys()

        for name in [nm for nm in names if nm not in names_except]:
            del self.coll_idx_dict[name]

    def get_centers(self):
        Nw, Nd, Nh = self.Nwdh
        self.centers = np.zeros(self.Nwdh + (3,))
        for iw in range(Nw):
            self.centers[iw, :, :, 0] = (iw + 0.5) * self.L_CELL
        for id in range(Nd):
            self.centers[:, id, :, 1] = (id + 0.5) * self.L_CELL
        for ih in range(Nh):
            self.centers[:, :, ih, 2] = (ih + 0.5) * self.L_CELL
        return self.centers - self.OFFSET_ZERO

    def get_cell_vertices(self):
        ZERO_BOX = DEFAULT_VERT_DICT[GEOTYPE.BOX] * self.L_CELL
        self.vertice_list = []
        for center in self.centers:
            self.vertice_list.append(ZERO_BOX + center)
        return self.vertice_list

    ##
    # @brief prepare geometric scene with cell references
    def prepare_reference_cells(self, gscene):
        self.gscene_ref = GeometryScene(gscene.urdf_content, gscene.urdf_path, gscene.joint_names, gscene.link_names,
                                        rviz=False)
        self.centers = self.get_centers().reshape((-1, 3))
        self.cell_refs = []
        for icell, center in enumerate(self.centers):
            self.cell_refs.append(
                self.gscene_ref.create_safe(GEOTYPE.BOX, str(icell), "base_link", dims=(self.L_CELL,) * 3,
                                            center=center, rpy=(0, 0, 0), color=(1, 1, 1, 0.2),
                                            display=True, collision=False, fixed=True))

    ##
    # @brief update colliding cell indexes with geomerty list
    def convert(self, gtem_list, Qdict):
        for gtem in gtem_list:
            if not gtem.collision:
                continue
            Tgtem = gtem.get_tf(Qdict)
            Rcoeffs = tuple(Tgtem[:3, :3].flatten())
            Pcoeffs = tuple(Tgtem[:3, 3])
            verts, radii = gtem.get_vertice_radius()
            verts_gjk = get_point_list_ltc(verts)
            dims_bp = make_point3_ltc(*gtem.dims)
            if gtem.gtype == GEOTYPE.BOX:
                coll_idxes_bp = self.ltc_bp.get_colliding_cells_box(*(Rcoeffs + Pcoeffs + (verts_gjk, dims_bp, radii)))
            else:
                coll_idxes_bp = self.ltc_bp.get_colliding_cells(*(Rcoeffs + Pcoeffs + (verts_gjk, dims_bp, radii)))
            num_colls = len(coll_idxes_bp)
            coll_idxes = np.zeros(num_colls, dtype=np.int)
            for i in range(num_colls):
                coll_idxes[i] = coll_idxes_bp[i]
            self.coll_idx_dict[gtem.name] = coll_idxes

    ##
    # @brief convert flattened index to 3D grid indices
    def index_to_grid(self, idxes):
        return np.unravel_index(idxes, self.Nwdh)