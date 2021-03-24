from .table_interface import *
from ...geometry.builder.scene_builder import *

class MarkerGroupTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'DLevel', 'GType', 'Dims', 'Color', 'Soft', 'K_col']
    HILIGHT_KEY = 'marker_group'
    CUSTOM_BUTTONS = ["MarkObj", "MarkEnv"]

    def get_items(self):
        return sorted(self.detector.aruco_map.values(), key=lambda x:x.name)

    def get_items_dict(self):
        return self.detector.aruco_map

    def serialize(self, gtem):
        return [gtem.name, gtem.dlevel.name, gtem.gtype.name if gtem.gtype is not None else "None",
                round_it_str(gtem.dims) if gtem.dims is not None else "None",
                round_it_str(gtem.color), str(gtem.soft), str(gtem.K_col)]

    def highlight_item(self, gtem, color=None):
        self.planning_pipeline.pscene.gscene.highlight_geometry(self.HILIGHT_KEY, gtem.name, color=color)
        for i, atem in zip(range(len(gtem)), gtem):
            self.detector.add_item_axis(self.planning_pipeline.pscene.gscene, self.HILIGHT_KEY, atem,
                                        axis_name=atem.oname+"%d"%i)

    def add_item(self, value):
        name = value[IDENTIFY_COL]
        self.detector.aruco_map[name] = MarkerSet(name,
                                               dlevel=getattr(DetectionLevel,value["DLevel"]),
                                               gtype=getattr(GEOTYPE, value['GType']) if value['GType'] != "None" else None,
                                               dims=str_num_it(value["Dims"]) if value["Dims"] != "None" else None,
                                               color=str_num_it(value["Color"]),
                                               soft=value["Soft"].lower() in ["true", "t"],
                                               K_col=float(value["K_col"]) if value["K_col"] != "None" else None
                                               )

    def delete_item(self, active_row):
        del self.detector.aruco_map[active_row]

    def update_item(self, atem, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            name_old = IDENTIFY_COL
            name_new = value
            if name_new in self.detector.aruco_map:
                res, msg = False, "marker name already defined ({})".format(name_new)
            else:
                gtem = self.detector.aruco_map[name_old]
                gtem.name = name_new
                self.detector.aruco_map[name_new] = gtem
                for atem in gtem:
                    atem.oname = name_new
        elif active_col == 'DLevel':
            try:
                atem.dlevel = getattr(DetectionLevel,value["DLevel"])
            except:
                res,msg = False, "wrong target type"
        elif active_col == 'GType':
            atem.gtype = getattr(GEOTYPE, value['GType']) if value['GType'] != "None" else None
        elif active_col == 'Dims':
            atem.dims = str_num_it(value["Dims"]) if value["Dims"] != "None" else None
        elif active_col == 'Color':
            atem.color = str_num_it(value["Color"])
        elif active_col == 'Soft':
            atem.soft = value["Soft"].lower() in ["true", "t"]
        elif active_col == 'K_col':
            atem.K_col = float(value["K_col"]) if value["K_col"] != "None" else None
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            if args[0]:
                s_builder = self.s_builder
                pscene = self.planning_pipeline.pscene
                s_builder.detect_and_register(level_mask=[DetectionLevel.MOVABLE], visualize=True)
                pscene.gscene.set_rviz()
            elif args[1]:
                s_builder = self.s_builder
                pscene = self.planning_pipeline.pscene
                s_builder.detect_and_register(level_mask=[DetectionLevel.ENVIRONMENT], visualize=True)
                pscene.gscene.set_rviz()
            else:
                print("Unknown button")
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")