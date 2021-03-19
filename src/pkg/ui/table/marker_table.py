from .table_interface import *
from ...detector.aruco.detector import ObjectMarker
from ...controller.combined_robot import RobotSpecs

class MarkerTable(TableInterface):
    HEADS = ['Object', IDENTIFY_COL, 'Size', 'Point', 'Direction']
    HILIGHT_KEY = 'marker'

    def get_items(self):
        aruco_list = []
        for k, v in self.detector.aruco_map.items():
            aruco_list+=v
        return sorted(aruco_list, key=lambda x:x.idx)

    def get_items_dict(self):
        aruco_list = self.get_items()
        return {atem.idx: atem for atem in aruco_list}

    def serialize(self, atem):
        return [atem.oname, atem.idx, atem.size, round_it_str(atem.point), round_it_str(atem.direction)]

    def highlight_item(self, atem, color=None):
        self.planning_pipeline.pscene.gscene.highlight_geometry(self.HILIGHT_KEY, atem.oname, color=color)
        self.planning_pipeline.detector.add_item_axis(self.planning_pipeline.pscene.gscene, self.HILIGHT_KEY, atem)

    def add_item(self, value):
        oname = value["Object"]
        if oname in self.detector.aruco_map:
            self.detector.aruco_map[oname].append(
                ObjectMarker(oname, int(value[IDENTIFY_COL]),
                             float(value['Size']), str_num_it(value['Point']), str_num_it(value['Direction']))
            )
        else:
            print("{} is not a registered marker group".format(oname))

    def delete_item(self, active_row):
        aruco_dict = self.get_items_dict()
        atem = aruco_dict[active_row]
        oname = atem.oname
        idx_atem =self.detector.aruco_map[oname].index(atem)
        del self.detector.aruco_map[oname][idx_atem]
        if not self.detector.aruco_map[oname]:
            del self.detector.aruco_map[oname]

    def update_item(self, atem, active_col, value):
        res, msg = True, ""
        if active_col == 'Object':
            oname_old = atem.oname
            oname_new = str(value)
            if oname_new not in self.detector.aruco_map:
                res, msg = False, "marker name not defined ({})".format(oname_new)
            else:
                atem.oname = oname_new
                self.detector.aruco_map[oname_new].append(atem)
                del self.detector.aruco_map[oname_old][atem]
        elif active_col == IDENTIFY_COL:
            atem.idx = int(value)
        elif active_col == "Size":
            atem.size = float(value)
        elif active_col == "Point":
            atem.set_offset(str_num_it(value), atem.direction)
        elif active_col == "Direction":
            atem.set_offset(atem.point, str_num_it(value))
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            print("No function on apply marker")
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")