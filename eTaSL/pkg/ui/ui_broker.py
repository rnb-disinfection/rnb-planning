from ..geometry.geometry import *

import dash_launcher
from .dash_launcher import TabInfo, TableInfo
from .table.geometry_table import *
from .table.object_table import *
from .table.handle_table import *
from .table.binder_table import *

class UIBroker:

    def __init__(self, graph):
        self.graph = graph
        self.tab_list = [
            TabInfo("Instances", [TableInfo("Geometry", '570px', interface=GeometryTable(graph)),
                                 TableInfo("Object", '270px', interface=ObjectTable(graph))]),
            TabInfo("Binding", [TableInfo("Handle", '570px', interface=HandleTable(graph)),
                                TableInfo("Binder", '270px', interface=BinderTable(graph))])
        ]

    def start_server(self):
        dash_launcher.set_tabs(self.tab_list)
        dash_launcher.run_server(on_background=True, debug=False)

    def set_tables(self):
        self.table_dict = {}
        for tab in self.tab_list:
            for table in tab.table_info_array:
                self.table_dict[table.table_name] = table.interface

        dash_launcher.set_tables(
            table_loader_dict={k: v.get_table for k,v in self.table_dict.items()},
            table_selector_dict={k: v.select for k,v in self.table_dict.items()},
            table_updater_dict={k: v.update for k,v in self.table_dict.items()},
            table_button_dict={k: v.button for k,v in self.table_dict.items()},
        )


    def button(self, button, *args, **kwargs):
        if button == TAB_BUTTON.APPLY:
            self.graph.set_rviz()
            if hasattr(self.graph, "planner"):
                self.graph.planner.update_gtems()
        else:
            TableInterface.button(self, button, *args, **kwargs)
