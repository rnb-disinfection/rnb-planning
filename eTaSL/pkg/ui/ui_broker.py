from ..geometry.geometry import *

import dash_launcher
from .dash_launcher import TabInfo, TableInfo
from .table.geometry_table import *
from .table.object_table import *
from .table.handle_table import *
from .table.binder_table import *
from .table.marker_table import *
from .table.marker_group_table import *

class UIBroker:

    def __init__(self, graph):
        self.graph = graph
        self.tab_list = [
            TabInfo("Instances", [TableInfo("Geometry", '550px', interface=GeometryTable(graph)),
                                 TableInfo("Object", '250px', interface=ObjectTable(graph))]),
            TabInfo("Binding", [TableInfo("Handle", '550px', interface=HandleTable(graph)),
                                TableInfo("Binder", '250px', interface=BinderTable(graph))]),
            TabInfo("Mark", [TableInfo("Marker", '550px', interface=MarkerTable(graph)),
                             TableInfo("MarkerGroup", '250px', interface=MarkerGroupTable(graph))])
        ]
        self.table_dict = {}
        for tab in self.tab_list:
            for table in tab.table_info_array:
                self.table_dict[table.table_name] = table.interface
                table.custom_buttons = table.interface.CUSTOM_BUTTONS

    def start_server(self):
        dash_launcher.set_tabs(self.tab_list)
        dash_launcher.run_server(on_background=True, debug=False)

    def set_tables(self):

        dash_launcher.set_tables(
            table_loader_dict={k: v.get_table for k,v in self.table_dict.items()},
            table_selector_dict={k: v.select for k,v in self.table_dict.items()},
            table_updater_dict={k: v.update for k,v in self.table_dict.items()},
            table_button_dict={k: v.button for k,v in self.table_dict.items()},
        )
