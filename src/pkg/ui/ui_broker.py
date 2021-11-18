from ..geometry.geometry import *

import dash_launcher
from .dash_launcher import TabInfo, TableInfo
from .table.geometry_table import *
from .table.object_table import *
from .table.handle_table import *
from .table.binder_table import *
from .table.marker_table import *
from .table.marker_group_table import *
from .table.camera_table import  *
from .table.robot_table import  *
from .table.task_planner_table import *
from .table.motion_planner_table import *
from .table.plan_condition_table import  *
from .table.plan_list_table import *

class UIBroker(Singleton):

    def __init__(self):
        self.__server_on = False
        pass

    def initialize(self, planning_pipeline, s_builder):
        self.planning_pipeline = planning_pipeline
        self.s_builder = s_builder
        self.tab_list = [
            TabInfo("Instances", [TableInfo("Geometry", '550px', interface=GeometryTable(planning_pipeline, s_builder)),
                                 TableInfo("Object", '250px', interface=ObjectTable(planning_pipeline, s_builder))]),
            TabInfo("Binding", [TableInfo("Handle", '550px', interface=HandleTable(planning_pipeline, s_builder)),
                                TableInfo("Binder", '250px', interface=BinderTable(planning_pipeline, s_builder))]),
            TabInfo("Mark", [TableInfo("Marker", '550px', interface=MarkerTable(planning_pipeline, s_builder)),
                             TableInfo("MarkerGroup", '250px', interface=MarkerGroupTable(planning_pipeline, s_builder))]),
            TabInfo("Planning", [TableInfo("PlanConditions", '200px', row_selectable='single',
                                           interface=PlanConditionTable(planning_pipeline, s_builder)),
                                 TableInfo("PlanList", '420px', row_selectable='single',
                                           interface=PlanListTable(planning_pipeline, s_builder))]),
            TabInfo("Setting", [TableInfo("Robot", '200px', interface=RobotTable(planning_pipeline, s_builder)),
                                TableInfo("MotionPlanner", '200px', row_selectable='single',
                                          interface=MotionPlanTable(planning_pipeline, s_builder)),
                                TableInfo("TaskPlanner", '200px', row_selectable='single',
                                          interface=TaskPlanTable(planning_pipeline, s_builder))
                                ])
        ]
        self.table_dict = {}
        for tab in self.tab_list:
            for table in tab.table_info_array:
                self.table_dict[table.table_name] = table.interface
                table.custom_buttons = table.interface.CUSTOM_BUTTONS

    def start_server(self, ip_host='0.0.0.0'):
        if not self.__server_on:
            dash_launcher.set_tabs(self.tab_list)
            dash_launcher.run_server(on_background=True, debug=False, host=ip_host)
            self.__server_on = True

    def set_tables(self):

        dash_launcher.set_tables(
            table_loader_dict={k: v.get_table for k,v in self.table_dict.items()},
            table_selector_dict={k: v.select for k,v in self.table_dict.items()},
            table_updater_dict={k: v.update for k,v in self.table_dict.items()},
            table_button_dict={k: v.button for k,v in self.table_dict.items()},
        )
