from .table_interface import *
from ...sensor.marker import ObjectMarker, MarkerSet, TargetType
from ...geometry.geometry import GEOTYPE
from ...environment_builder import *

class MarkerGroupTable(TableInterface):
    HEADS = [IDENTIFY_COL, 'TType', 'GType', 'Dims', 'Color', 'Soft', 'K_col']
    HILIGHT_KEY = 'marker_group'
    CUSTOM_BUTTONS = ["MarkObj", "MarkEnv"]

    def get_items(self):
        return sorted(self.graph.cam.aruco_map.values(), key=lambda x:x.name)

    def get_items_dict(self):
        return self.graph.cam.aruco_map

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
        self.graph.cam.aruco_map[name] = MarkerSet(name,
                                               ttype=getattr(TargetType,value["TType"]),
                                               gtype=getattr(GEOTYPE, value['GType']) if value['GType'] != "None" else None,
                                               dims=str_num_it(value["Dims"]) if value["Dims"] != "None" else None,
                                               color=str_num_it(value["Color"]),
                                               soft=value["Soft"].lower() in ["true", "t"],
                                               K_col=float(value["K_col"]) if value["K_col"] != "None" else None
                                               )

    def delete_item(self, active_row):
        del self.graph.cam.aruco_map[active_row]

    def update_item(self, atem, active_col, value):
        res, msg = True, ""
        if active_col == IDENTIFY_COL:
            name_old = IDENTIFY_COL
            name_new = value
            if name_new in self.graph.cam.aruco_map:
                res, msg = False, "marker name already defined ({})".format(name_new)
            else:
                gtem = self.graph.cam.aruco_map[name_old]
                gtem.name = name_new
                self.graph.cam.aruco_map[name_new] = gtem
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
            atem.soft = value["Soft"].lower() in ["true", "t"]
        elif active_col == 'K_col':
            atem.K_col = float(value["K_col"]) if value["K_col"] != "None" else None
        return res, msg

    def button(self, button, *args, **kwargs):
        print("button clicked")
        if button == TAB_BUTTON.CUSTOM:
            if args[0]:
                graph = self.graph
                BINDER_DICT = {k:dict(_type=v.__class__, object_name=v.object.name, point=v.point, rpy=v.rpy_point) \
                               for k, v in graph.binder_dict.items() \
                               if (not v.controlled) and \
                               (v.object.name in graph.cam.aruco_map and graph.cam.aruco_map[v.object.name].ttype == TargetType.MOVABLE)}
                OBJECT_DICT = {k:dict(_type=v.__class__) for k,v in graph.object_dict.items()}
                objectPose_dict_mv, corner_dict_mv, color_image, aruco_map_mv = detect_objects(graph.cam.aruco_map, graph.cam.dictionary)
                put_point_dict = graph.register_object_gen(objectPose_dict_mv, BINDER_DICT, OBJECT_DICT, link_name="base_link")
                update_geometries(graph.ghnd, objectPose_dict_mv.keys(), objectPose_dict_mv, graph.cam.ref_tuple[1])
                graph.set_rviz(graph.joints.position)

                screen_size = (1080,1920)
                kn_image_out = draw_objects(color_image, graph.cam.aruco_map, objectPose_dict_mv, corner_dict_mv, *graph.cam.kn_config, axis_len=0.1)
                kn_image_out_res = cv2.resize(kn_image_out, (screen_size[1], screen_size[0]))
                cv2.imshow("test", kn_image_out_res)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            elif args[1]:
                graph = self.graph
                cam = graph.cam
                env_gen_dict, objectPose_dict, corner_dict, color_image, rs_objectPose_dict, rs_corner_dict, rs_image = cam.detect_environment(graph.ghnd)
                add_objects_gen(graph, env_gen_dict)
                graph.set_rviz()

                screen_size = (1080,1920)
                kn_image_out = draw_objects(color_image, cam.aruco_map, objectPose_dict, corner_dict, *cam.kn_config, axis_len=0.1)
                rs_image_out = draw_objects(rs_image, graph.cam.aruco_map, {k:v for k,v in rs_objectPose_dict.items() if k != 'wall'}, rs_corner_dict, *cam.rs_config, axis_len=0.1)
                kn_image_out_res = cv2.resize(kn_image_out, (rs_image_out.shape[1], rs_image_out.shape[0]))
                image_out = np.concatenate([kn_image_out_res, rs_image_out], axis=1)
                ratio = np.min(np.array(screen_size,dtype=np.float)/np.array(image_out.shape[:2],dtype=np.float))
                image_out_res = cv2.resize(image_out, (int(image_out.shape[1]*ratio), int(image_out.shape[0]*ratio)))
                cv2.imshow("test", image_out_res)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            else:
                print("Unknown button")
        else:
            TableInterface.button(self, button, *args, **kwargs)
        print("button action done")