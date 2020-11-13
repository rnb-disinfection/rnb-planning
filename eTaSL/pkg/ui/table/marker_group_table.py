from .table_interface import *
from ...constraint.constraint_object import otype_to_class
from ...utils.joint_utils import joint_list2dict
from ...sensor.marker import ObjectMarker, MarkerSet, TargetType
from ...geometry.geometry import GEOTYPE

class MarkerGroupTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'TType', 'GType', 'Dims', 'Color', 'Soft', 'K_col']
    HILIGHT_KEY = 'marker_group'

    def get_items(self):
        return sorted(self.graph.aruco_map.values(), key=lambda x:x.name)

    def get_items_dict(self):
        return self.graph.aruco_map

    def serialize(self, gtem):
        return [gtem.name, gtem.ttype.name, gtem.gtype.name if gtem.gtype is not None else "None",
                round_it_str(gtem.dims) if gtem.dims is not None else "None",
                round_it_str(gtem.color), str(gtem.soft), str(gtem.K_col)]

    def highlight_item(self, gtem, color=None):
        self.graph.highlight_geometry(self.HILIGHT_KEY, gtem.name, color=color)
        for i, atem in zip(range(len(gtem)), gtem):
            self.graph.add_aruco_axis(self.HILIGHT_KEY, atem, axis_name=gtem.name+"_"+str(i))

    def add_item(self, value):
        name = value[IDENTIFY_COL]
        self.graph.aruco_map[name] = MarkerSet(name,
                                               ttype=getattr(TargetType,value["TType"]),
                                               gtype=getattr(GEOTYPE, value['GType']) if value['GType'] != "None" else None,
                                               dims=str_num_it(value["Dims"]) if value["Dims"] != "None" else None,
                                               color=str_num_it(value["Color"]),
                                               soft=value["Soft"]=="True",
                                               K_col=float(value["K_col"]) if value["K_col"] != "None" else None
                                               )

    def delete_item(self, active_row):
        del self.graph.aruco_map[active_row]

    def update_item(self, atem, active_col, value):
        [IDENTIFY_COL, 'TType', 'GType', 'Dims', 'Color', 'Soft', 'K_col']
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            name_old = IDENTIFY_COL
            name_new = value
            if name_new in self.graph.aruco_map:
                res, msg = False, "marker name already defined ({})".format(name_new)
            else:
                gtem = self.graph.aruco_map[name_old]
                gtem.name = name_new
                self.graph.aruco_map[name_new] = gtem
                for atem in gtem:
                    atem.oname = name_new
        elif active_col == 'TType':
            try:
                atem.ttype = getattr(TargetType,value["TType"])
            except:
                res,msg = False, "wrong target type"
        elif active_col == 'GType':
            atem.gtype = getattr(GEOTYPE, value['GType']) if value['GType'] != "None" else None
        elif active_col == 'Dims':
            atem.dims = str_num_it(value["Dims"]) if value["Dims"] != "None" else None
        elif active_col == 'Color':
            atem.color = str_num_it(value["Color"])
        elif active_col == 'Soft':
            atem.soft = value["Soft"]=="True"
        elif active_col == 'K_col':
            atem.K_col = float(value["K_col"]) if value["K_col"] != "None" else None
        return res, msg

    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.CUSTOM:
            print("No function on apply marker groups")
        else:
            TableInterface.button(self, button, *args, **kwargs)