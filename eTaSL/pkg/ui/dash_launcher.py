# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.

import dash
import dash_core_components as dcc
import dash_html_components as html
import dash_table
from dash.dependencies import Input, Output, State
from uuid import uuid1
from collections import defaultdict
import visdcc
import dash_split_pane
from enum import Enum
import numpy as np

class TAB_BUTTON(Enum):
    CUSTOM = 0
    SAVE = 1
    LOAD = 2

IDENTIFY_COL = 'Name'
COLUMNS_SMALL_FONT = [IDENTIFY_COL, 'Dims', 'Center', 'Rpy', 'Point', 'Position', 'Direction', 'Color', 'CameraMatrix', 'Distortion']
__ID_DICT = defaultdict(lambda: uuid1().int)

def table_updater_default(*args, **kwargs):
    print("table_updater_default-input: {}, {}".format(args, kwargs))
    return True, ""

class TabInfo:
    def __init__(self, tab_name, table_info_array):
        self.tab_name, self.table_info_array = tab_name, table_info_array
        self.tab_id = get_tab_id(self.tab_name)

class TableInfo:
    def __init__(self, table_name, table_height, table_loader=lambda:([IDENTIFY_COL],[]),
                 table_selector=table_updater_default, table_updater=table_updater_default,
                 table_button = table_updater_default, custom_buttons=[], row_selectable='multi', selected_rows=[], interface=None):
        self.table_name, self.table_height, \
        self.table_loader, self.table_selector, self.table_updater, self.table_button = \
            table_name, table_height, table_loader, table_selector,table_updater, table_button
        self.custom_buttons = custom_buttons
        self.interface = interface
        self.row_selectable = row_selectable
        self.selected_rows = selected_rows

def get_tab_id(tab_name):
    return 'tab-'+tab_name.lower()

__tab_list = []
__tab_defaults = [0, 1]
__table_dict = {}

def set_tabs(tab_list, tab_defaults=[0,1]):
    global __tab_list, __tab_defaults, __table_dict
    __tab_list = tab_list
    __tab_defaults = tab_defaults
    __table_dict = {}
    for tab in __tab_list:
        for table in tab.table_info_array:
            __table_dict[table.table_name] = table

def set_tables(table_loader_dict, table_selector_dict, table_updater_dict, table_button_dict):
    for table_name, loader in table_loader_dict.items():
        __table_dict[table_name].table_loader = loader
    for table_name, selecter_dict in table_selector_dict.items():
        __table_dict[table_name].table_selector = selecter_dict
    for table_name, updater_dict in table_updater_dict.items():
        __table_dict[table_name].table_updater = updater_dict
    for table_name, button_dict in table_button_dict.items():
        __table_dict[table_name].table_button = button_dict

def get_tabs():
    return [
        dcc.Tab(label=tinfo.tab_name, value=tinfo.tab_id,
                className='custom-tab', selected_className='custom-tab--selected')
        for tinfo in __tab_list
    ]

external_stylesheets = []

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)
app.title = "TAMP_RNB"

def generate_table(columns, items, table_id, row_selectable='multi', selected_rows=[], height="100%"):
    data = [{k:v for k,v in zip(columns, item)} for item in items]
    print("generate table - {}".format(table_id))
    for dtem in data:
        dtem['id'] = __ID_DICT[dtem[IDENTIFY_COL]]
    return [
        visdcc.Run_js(table_id+'-javascript-refresh'),
        dcc.ConfirmDialog(
            id=table_id+'-alert-not-changeable',
            message='You changed non-changeable value! Refresh the page!',
        ),
        dash_table.DataTable(
            id=table_id+'-table-row-ids',
            columns=[{'name': col, 'id': col, 'deletable': False} for col in columns],
            data=data,
            editable=True,
            filter_action="native",
            sort_action="native",
            sort_mode='multi',
            row_selectable=row_selectable,
            row_deletable=True,
            selected_rows=selected_rows,
            style_table={
                'height': height,
                'overflowY': 'scroll',
                'fontSize': '14px'
            },
            style_header={'backgroundColor': 'rgb(27, 30, 36)',
                          'color': 'white',
                          'fontWeight': 'bold'},
            style_data_conditional=[
                                       {
                                           'if': {'column_id': c},
                                           'fontSize': '12px'
                                       } for c in COLUMNS_SMALL_FONT
                                   ]+[{
                'if': {'row_index': 'even'},
                'backgroundColor': '#F5F5F5'
            }],
            style_cell={
                'boxShadow': '0 0',
                'border': '0px solid #ffffff',
                'borderBottom': '1px solid lightgray',
                'textAlign': 'center'}
        ),
        html.Div(id=table_id+'-table-row-ids-container')
    ]

def set_layout():
    app.layout = html.Div(className="full-screen", children=[
        dash_split_pane.DashSplitPane(
            children=[
                html.Div(className="column-common", children=[
                    dcc.Tabs(id='tabs-left', value=__tab_list[__tab_defaults[0]].tab_id,
                             parent_className='custom-tabs',
                             className='custom-tabs-container',
                             children=get_tabs()),
                    html.Div(className='custom-content-container', id='tabs-left-content')]
                         )
                ,
                html.Div(className="column-common", children=[
                    dcc.Tabs(id='tabs-right', value=__tab_list[__tab_defaults[1]].tab_id,
                             parent_className='custom-tabs',
                             className='custom-tabs-container',
                             children=get_tabs()),
                    html.Div(className='custom-content-container', id='tabs-right-content')]
                         )
            ],
            id="splitter",
            split="vertical",
            size="63%",
            resizerStyle={},
        )
        # html.Div(className="column-left", children=[
        #     dcc.Tabs(id='tabs-left', value=__tab_list[__tab_defaults[0]].tab_id,
        #              parent_className='custom-tabs',
        #              className='custom-tabs-container',
        #              children=get_tabs()),
        #     html.Div(className='custom-content-container', id='tabs-left-content')]
        #          ),
        # html.Div(className="column-mid", children=[]),
        # html.Div(className="column-right", children=[
        #     dcc.Tabs(id='tabs-right', value=__tab_list[__tab_defaults[1]].tab_id,
        #              parent_className='custom-tabs',
        #              className='custom-tabs-container',
        #              children=get_tabs()),
        #     html.Div(className='custom-content-container', id='tabs-right-content')]
        #          )
    ])
    for tab_info in __tab_list:
        for table_info in tab_info.table_info_array:
            register_callback(table_info.table_name)

@app.callback(Output('tabs-left-content', 'children'),
              [Input('tabs-left', 'value')])
def render_content_left(tab):
    return __render_content_func(tab)

@app.callback(Output('tabs-right-content', 'children'),
              [Input('tabs-right', 'value')])
def render_content_right(tab):
    return __render_content_func(tab)

def register_callback(table_id):
    @app.callback(
        [Output(table_id+'-table-row-ids-container', 'children'),
         Output(table_id+'-alert-not-changeable', 'displayed')],
        [Input(table_id+'-table-row-ids', 'derived_virtual_row_ids'),
         Input(table_id+'-table-row-ids', 'selected_row_ids'),
         Input(table_id+'-table-row-ids', 'active_cell'),
         Input(table_id+'-table-row-ids', 'data'),
         Input(table_id+'-table-row-ids', 'data_previous'),
         Input(table_id+'-table-row-ids', 'filter_query')])
    def update_graphs(row_ids, selected_row_ids, active_cell, data, data_previous, filter_query):
        # When the table is first rendered, `derived_virtual_data` and
        # `derived_virtual_selected_rows` will be `None`. This is due to an
        # idiosyncrasy in Dash (unsupplied properties are always None and Dash
        # calls the dependent callbacks when the component is first rendered).
        # So, if `rows` is `None`, then the component was just rendered
        # and its value will be the same as the component's dataframe.
        # Instead of setting `None` in here, you could also set
        # `derived_virtual_data=df.to_rows('dict')` when you initialize
        # the component.
        ### delete ###
        res = True
        id_del = None
        if filter_query == update_graphs.__filter_query_prev:
            if data_previous is not None and \
                    len(data)<len(data_previous):
                ids_prev = set([dat[IDENTIFY_COL] for dat in data_previous])
                ids_cur = set([dat[IDENTIFY_COL] for dat in data])
                ids_del = ids_prev - ids_cur
                if ids_del:
                    id_del = list(ids_del)[0]
                    if id_del != update_graphs.__id_del_prev:
                        res, msg = __table_dict[table_id].table_updater(id_del, IDENTIFY_COL, 0, delete=True)
                        update_graphs.__id_del_prev = id_del
                        if not res:
                            print(msg)
                    else:
                        id_del = None
        data_ids = [dtem['id'] for dtem in data]
        if id_del is None:
            ### update prev cell ###
            if update_graphs.__cell_prev:
                row_id = update_graphs.__cell_prev['row_id']
                col_id = update_graphs.__cell_prev['column_id']
                if row_ids is not None and row_id and col_id:
                    if (data and data_previous) and data[-1]['id']!=update_graphs.__add_prev and \
                            (len(data) == len(data_previous)) and \
                            "" in data_previous[-1].values() and \
                            "" not in data[-1].values(): # add
                        res, msg = __table_dict[table_id].table_updater(-1, -1,
                                                                        {str(k):str(v) for k,v in data[-1].items()},
                                                                        add=True)
                        update_graphs.__add_prev = data[-1]['id']
                        __ID_DICT[data[-1][IDENTIFY_COL]] = update_graphs.__add_prev
                    else: # update
                        if row_id in data_ids:
                            row = data_ids.index(row_id)
                            if data_previous:
                                row_prev = [dat['id'] for dat in data_previous].index(row_id)
                                __name = data_previous[row_prev][IDENTIFY_COL]
                            else:
                                __name = data[row][IDENTIFY_COL]
                            __value = str(data[row][col_id])
                            res, msg = __table_dict[table_id].table_updater(__name, col_id, __value)
                    if not res:
                        print(msg)
                update_graphs.__cell_prev = None

            ### render selecteds ###

            selected_row_list = map(lambda sid: str(data[data_ids.index(sid)][IDENTIFY_COL]),
                                    selected_row_ids or [])
            if active_cell is None:
                active_row, active_col = None, None
            else:
                row = data_ids.index(active_cell['row_id'])
                active_row, active_col = data[row][IDENTIFY_COL],  active_cell['column_id']

            __table_dict[table_id].table_selector(selected_row_list, active_row, active_col)
        update_graphs.__cell_prev = active_cell
        update_graphs.__filter_query_prev = filter_query
        return html.Div(""), not res
    update_graphs.__cell_prev = None
    update_graphs.__filter_query_prev = None
    update_graphs.__id_del_prev = None
    update_graphs.__add_prev = None

    @app.callback(
        Output(table_id+'-table-row-ids', 'data'),
        [Input(table_id+'-add-row-button', 'n_clicks')],
        [State(table_id+'-table-row-ids', 'data'),
         State(table_id+'-table-row-ids', 'columns')])
    def add_row(n_clicks, rows, columns):
        if n_clicks > 0:
            rows.append({c['id']: '' if c['id']!='id' else uuid1().int for c in columns+[{'id':'id'}]})
        return rows

    @app.callback(
        Output(table_id+'-button-receiver', 'children'),
        [Input(table_id+'-{}-button'.format(cbn.lower()), 'n_clicks') for cbn in __table_dict[table_id].custom_buttons]+ \
        [Input(table_id + '-save-button', 'n_clicks')],
        [State(table_id + '-filename', 'value'),
         State(table_id + '-table-row-ids', 'data')])
    def button_event(*args):

        a_clicks = args[:-3]
        s_clicks, filename, data = args[-3:]
        a_clicks_sub = np.maximum(np.subtract(a_clicks, add_row.__a_clicks_prev), 0)
        if np.sum(a_clicks_sub)>0:
            add_row.__a_clicks_prev = a_clicks
            __table_dict[table_id].table_button(TAB_BUTTON.CUSTOM, *a_clicks_sub)
        elif s_clicks-add_row.__s_clicks_prev:
            add_row.__s_clicks_prev = s_clicks
            if filename:
                __table_dict[table_id].table_button(TAB_BUTTON.SAVE, filename=filename, data=data)
                print("saved on :", filename)
            else:
                print("filname is empty")
        return ""
    add_row.__a_clicks_prev = 0
    add_row.__s_clicks_prev = 0

    @app.callback(Output(table_id+'-javascript-refresh', 'run'),
                  [Input(table_id+'-alert-not-changeable', 'submit_n_clicks'),
                   Input(table_id + '-save-receiver', 'children')])
    def refresh_page(submit_n_clicks, children):
        if submit_n_clicks or children>0:
            return "location.reload()"
        else:
            return ""


    @app.callback([Output(table_id + '-save-receiver', 'children'),
                   Output(table_id + '-filename', 'value')
                   ],
                  [Input(table_id + '-load-button', 'contents')],
                  [State(table_id + '-load-button', 'filename'),
                   State(table_id + '-table-row-ids', 'data'),
                   State(table_id + '-save-receiver', 'children')])
    def load_file(contents, filename, data, children):
        if filename is not None:
            print("load: {}".format(filename))
            __table_dict[table_id].table_button(TAB_BUTTON.LOAD, filename, data)
            return 1, filename
        return 0, filename

def __render_content_func(tab):
    for tabinfo in __tab_list:
        if tab == tabinfo.tab_id:
            table_list = []
            for tableinfo in tabinfo.table_info_array:
                table_list += [
                    html.Div(id=tableinfo.table_name + "-button-receiver", children=[], style={'display':'none'}),
                    html.Div(id=tableinfo.table_name + "-save-receiver", children=0, style={'display':'none'}),
                    html.Div(className="row-header", children=[
                        tableinfo.table_name,
                        html.Button(
                            'Load', className="button-bb"),
                        html.Button(
                            'Save', className="button-bb", id=tableinfo.table_name + '-save-button', n_clicks=0)] + \
                        [html.Button(
                            cbn, className="button-bb", id=tableinfo.table_name + '-{}-button'.format(cbn.lower()),
                            n_clicks=0) for cbn in tableinfo.custom_buttons] + \
                        [html.Button(
                            'Add Row', className="button-bb", id=tableinfo.table_name + '-add-row-button', n_clicks=0),
                        dcc.Input(type="text", placeholder="*."+tableinfo.table_name.lower(), className="text-filename", id=tableinfo.table_name + '-filename'),
                        html.Div('File:', style={"font-size":"14px", "font-family":"Roboto", "float":"right", 'margin-top':'3px', 'margin-right':'3px'}),
                        dcc.Upload(className="button-bb", children='Load', id=tableinfo.table_name + '-load-button',
                                   style={'margin-top': '-35px', 'align': 'top'} ),
                    ]),
                    html.Div(className="row-content", children=generate_table(*tableinfo.table_loader(),
                                                                          table_id=tableinfo.table_name,
                                                                          height=tableinfo.table_height,
                                                                          row_selectable=tableinfo.row_selectable,
                                                                          selected_rows=tableinfo.selected_rows)),
                ]
            return html.Div(className="custom-tab-content", children=table_list)

from threading import Thread

def run_server(on_background=False, **kwargs):
    set_layout()
    if on_background:
        __thread = Thread(target=app.run_server, kwargs=kwargs)
        __thread.start()
    else:
        app.run_server(**kwargs)

if __name__ == '__main__':
    set_tabs([TabInfo("Geometry", [TableInfo("Geometry", '850px',
                                             lambda:([IDENTIFY_COL, 'GType', 'Link', 'Dims', 'Center', 'Rpy', 'Disp', 'Coll', 'Fix', 'Soft', 'Online'],
                                                     [['indy0_link0_Cylinder_0', 'SEGMENT', 'indy0_link0', '0.100,0.100,0.200', '-0.100,0.000,0.030', '-0.000,1.571,0.000', 'True', 'True', 'True', 'False', 'False'], ['indy0_link1_Cylinder_0', 'SEGMENT', 'indy0_link1', '0.200,0.200,0.300', '0.000,0.000,0.072', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['indy0_link2_Cylinder_0', 'SEGMENT', 'indy0_link2', '0.146,0.146,0.450', '-0.225,0.000,0.090', '1.571,-0.000,-1.571', 'True', 'True', 'True', 'False', 'False'], ['indy0_link2_Cylinder_1', 'SEGMENT', 'indy0_link2', '0.170,0.170,0.200', '0.000,0.000,-0.010', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['indy0_link3_Cylinder_0', 'SEGMENT', 'indy0_link3', '0.130,0.130,0.200', '0.000,0.000,0.030', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['indy0_link4_Cylinder_0', 'SEGMENT', 'indy0_link4', '0.120,0.120,0.350', '0.000,0.000,-0.092', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['indy0_link5_Cylinder_0', 'SEGMENT', 'indy0_link5', '0.110,0.110,0.183', '-0.000,0.000,-0.022', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['indy0_link6_Cylinder_0', 'SEGMENT', 'indy0_link6', '0.120,0.120,0.250', '0.000,0.000,-0.038', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['indy0_tcp_Cylinder_0', 'SEGMENT', 'indy0_tcp', '0.032,0.032,0.080', '0.005,0.044,0.100', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['indy0_tcp_Cylinder_1', 'SEGMENT', 'indy0_tcp', '0.032,0.032,0.080', '-0.005,0.044,0.100', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['indy0_tcp_Cylinder_2', 'SEGMENT', 'indy0_tcp', '0.032,0.032,0.080', '0.005,-0.044,0.100', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['indy0_tcp_Cylinder_3', 'SEGMENT', 'indy0_tcp', '0.032,0.032,0.080', '-0.005,-0.044,0.100', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['panda1_link0_Cylinder_0', 'SEGMENT', 'panda1_link0', '0.200,0.200,0.102', '-0.030,0.000,0.030', '3.142,-1.373,3.142', 'True', 'True', 'True', 'False', 'False'], ['panda1_link0_Cylinder_1', 'SEGMENT', 'panda1_link0', '0.100,0.100,0.300', '-0.050,0.000,0.030', '1.571,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['panda1_link1_Cylinder_0', 'SEGMENT', 'panda1_link1', '0.220,0.220,0.132', '0.000,-0.028,-0.075', '-2.712,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['panda1_link2_Cylinder_0', 'SEGMENT', 'panda1_link2', '0.180,0.180,0.175', '0.000,-0.080,0.035', '1.983,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['panda1_link3_Cylinder_0', 'SEGMENT', 'panda1_link3', '0.180,0.180,0.120', '0.040,0.027,-0.035', '2.485,-0.736,-1.693', 'True', 'True', 'True', 'False', 'False'], ['panda1_link4_Cylinder_0', 'SEGMENT', 'panda1_link4', '0.160,0.160,0.170', '-0.056,0.058,0.027', '-2.006,-0.715,1.059', 'True', 'True', 'True', 'False', 'False'], ['panda1_link5_Cylinder_0', 'SEGMENT', 'panda1_link5', '0.160,0.160,0.238', '0.000,0.032,-0.115', '2.874,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['panda1_link6_Cylinder_0', 'SEGMENT', 'panda1_link6', '0.180,0.180,0.090', '0.035,0.000,0.000', '-0.000,-1.571,0.000', 'True', 'True', 'True', 'False', 'False'], ['panda1_hand_Cylinder_0', 'SEGMENT', 'panda1_hand', '0.120,0.120,0.140', '0.000,0.000,0.020', '1.571,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['panda1_leftfinger_Cylinder_0', 'SEGMENT', 'panda1_leftfinger', '0.036,0.036,0.033', '0.000,0.013,0.035', '-2.733,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['panda1_rightfinger_Cylinder_0', 'SEGMENT', 'panda1_rightfinger', '0.036,0.036,0.033', '0.000,-0.013,0.035', '-2.733,-0.000,-3.142', 'True', 'True', 'True', 'False', 'False'], ['pole_cam1', 'SEGMENT', 'world', '0.150,0.150,0.709', '0.702,-0.389,0.355', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['pole_cam0', 'SEGMENT', 'world', '0.150,0.150,0.645', '0.051,-0.510,0.322', '0.000,-0.000,0.000', 'True', 'True', 'True', 'False', 'False'], ['wall', 'BOX', 'world', '3.000,3.000,0.010', '0.073,0.555,0.233', '1.577,0.063,-0.002', 'True', 'True', 'True', 'False', 'False'], ['floor', 'BOX', 'world', '1.520,0.720,0.016', '0.000,0.000,0.000', '0.000,-0.000,-0.000', 'True', 'True', 'True', 'False', 'False'], ['grip1', 'SPHERE', 'panda1_hand', '0.000,0.000,0.000', '0.000,0.000,0.112', '0.000,-0.000,0.000', 'False', 'False', 'True', 'False', 'False'], ['grip0', 'SPHERE', 'indy0_tcp', '0.000,0.000,0.000', '0.000,0.000,0.140', '0.000,-0.000,0.000', 'False', 'False', 'True', 'False', 'False'], ['box1', 'BOX', 'world', '0.050,0.050,0.050', '0.307,-0.180,0.031', '1.168,-1.560,2.933', 'True', 'True', 'False', 'False', 'False'], ['goal', 'BOX', 'world', '0.100,0.100,0.010', '0.031,0.263,0.012', '0.001,-0.003,1.394', 'True', 'True', 'False', 'False', 'False'], ['box3', 'SPHERE', 'world', '0.150,0.150,0.150', '0.613,-0.119,0.026', '0.000,-0.000,0.000', 'True', 'True', 'True', 'True', 'True'], ['box2', 'BOX', 'world', '0.050,0.050,0.050', '-0.213,-0.212,0.030', '-2.475,1.553,0.370', 'True', 'True', 'False', 'False', 'False'], ['pointer_box1_top_p', 'SPHERE', 'world', '0.000,0.000,0.000', '0.322,-0.159,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_bottom_p', 'SPHERE', 'world', '0.000,0.000,0.000', '0.293,-0.200,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_top_g', 'SPHERE', 'world', '0.000,0.000,0.000', '0.307,-0.180,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_bottom_g', 'SPHERE', 'world', '0.000,0.000,0.000', '0.307,-0.180,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box1_top_f', 'SPHERE', 'world', '0.000,0.000,0.000', '0.322,-0.159,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box1_bottom_f', 'SPHERE', 'world', '0.000,0.000,0.000', '0.293,-0.200,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_right_p', 'SPHERE', 'world', '0.000,0.000,0.000', '0.307,-0.180,0.056', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_left_p', 'SPHERE', 'world', '0.000,0.000,0.000', '0.307,-0.180,0.006', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_front_p', 'SPHERE', 'world', '0.000,0.000,0.000', '0.287,-0.165,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_back_p', 'SPHERE', 'world', '0.000,0.000,0.000', '0.328,-0.194,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_right_g', 'SPHERE', 'world', '0.000,0.000,0.000', '0.307,-0.180,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_left_g', 'SPHERE', 'world', '0.000,0.000,0.000', '0.307,-0.180,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_front_g', 'SPHERE', 'world', '0.000,0.000,0.000', '0.307,-0.180,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box1_back_g', 'SPHERE', 'world', '0.000,0.000,0.000', '0.307,-0.180,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box1_right_f', 'SPHERE', 'world', '0.000,0.000,0.000', '0.307,-0.180,0.056', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box1_left_f', 'SPHERE', 'world', '0.000,0.000,0.000', '0.307,-0.180,0.006', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box1_front_f', 'SPHERE', 'world', '0.000,0.000,0.000', '0.287,-0.165,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box1_back_f', 'SPHERE', 'world', '0.000,0.000,0.000', '0.328,-0.194,0.031', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_top_p', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.237,-0.204,0.029', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_bottom_p', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.189,-0.219,0.030', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_top_g', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.213,-0.212,0.030', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_bottom_g', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.213,-0.212,0.030', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box2_top_f', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.237,-0.204,0.029', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box2_bottom_f', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.189,-0.219,0.030', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_right_p', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.213,-0.211,0.005', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_left_p', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.213,-0.212,0.055', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_front_p', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.206,-0.188,0.030', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_back_p', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.220,-0.235,0.029', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_right_g', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.213,-0.212,0.030', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_left_g', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.213,-0.212,0.030', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_front_g', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.213,-0.212,0.030', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['pointer_box2_back_g', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.213,-0.212,0.030', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box2_right_f', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.213,-0.211,0.005', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box2_left_f', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.213,-0.212,0.055', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box2_front_f', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.206,-0.188,0.030', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False'], ['framer_box2_back_f', 'SPHERE', 'world', '0.000,0.000,0.000', '-0.220,-0.235,0.029', '0.000,-0.000,0.000', 'False', 'False', 'False', 'False', 'False']]),
                                             table_updater_default)]),
              TabInfo("Binding", [TableInfo("Handle", '570px',
                                            lambda: (['Object', 'OType', 'Handle', 'CType', IDENTIFY_COL, 'Point', 'Direction'],
                                                     [['box1', 'GeometryItem', 'right_f', 'Frame', 'framer_box1_right_f', '0.025,0.000,0.000', '0.000,-1.571,0.000'], ['box1', 'GeometryItem', 'right_g', 'Grasp2', 'pointer_box1_right_g', '0.000,0.000,0.000', '-1.000,0.000,0.000'], ['box1', 'GeometryItem', 'back_f', 'Frame', 'framer_box1_back_f', '0.000,0.025,0.000', '1.571,0.000,0.000'], ['box1', 'GeometryItem', 'left_f', 'Frame', 'framer_box1_left_f', '-0.025,0.000,0.000', '0.000,1.571,0.000'], ['box1', 'GeometryItem', 'left_g', 'Grasp2', 'pointer_box1_left_g', '0.000,0.000,0.000', '1.000,0.000,0.000'], ['box1', 'GeometryItem', 'bottom_p', 'Place', 'pointer_box1_bottom_p', '0.000,0.000,-0.025', '0.000,0.000,1.000'], ['box1', 'GeometryItem', 'back_g', 'Grasp2', 'pointer_box1_back_g', '0.000,0.000,0.000', '0.000,-1.000,0.000'], ['box1', 'GeometryItem', 'top_g', 'Grasp2', 'pointer_box1_top_g', '0.000,0.000,0.000', '0.000,0.000,-1.000'], ['box1', 'GeometryItem', 'top_f', 'Frame', 'framer_box1_top_f', '0.000,0.000,0.025', '3.142,0.000,0.000'], ['box1', 'GeometryItem', 'right_p', 'Place', 'pointer_box1_right_p', '0.025,0.000,0.000', '-1.000,0.000,0.000'], ['box1', 'GeometryItem', 'front_f', 'Frame', 'framer_box1_front_f', '0.000,-0.025,0.000', '-1.571,0.000,0.000'], ['box1', 'GeometryItem', 'front_g', 'Grasp2', 'pointer_box1_front_g', '0.000,0.000,0.000', '0.000,1.000,0.000'], ['box1', 'GeometryItem', 'top_p', 'Place', 'pointer_box1_top_p', '0.000,0.000,0.025', '0.000,0.000,-1.000'], ['box1', 'GeometryItem', 'bottom_f', 'Frame', 'framer_box1_bottom_f', '0.000,0.000,-0.025', '0.000,0.000,0.000'], ['box1', 'GeometryItem', 'bottom_g', 'Grasp2', 'pointer_box1_bottom_g', '0.000,0.000,0.000', '0.000,0.000,1.000'], ['box1', 'GeometryItem', 'front_p', 'Place', 'pointer_box1_front_p', '0.000,-0.025,0.000', '0.000,1.000,0.000'], ['box1', 'GeometryItem', 'back_p', 'Place', 'pointer_box1_back_p', '0.000,0.025,0.000', '0.000,-1.000,0.000'], ['box1', 'GeometryItem', 'left_p', 'Place', 'pointer_box1_left_p', '-0.025,0.000,0.000', '1.000,0.000,0.000'], ['box2', 'GeometryItem', 'right_f', 'Frame', 'framer_box2_right_f', '0.025,0.000,0.000', '0.000,-1.571,0.000'], ['box2', 'GeometryItem', 'right_g', 'Grasp2', 'pointer_box2_right_g', '0.000,0.000,0.000', '-1.000,0.000,0.000'], ['box2', 'GeometryItem', 'back_f', 'Frame', 'framer_box2_back_f', '0.000,0.025,0.000', '1.571,0.000,0.000'], ['box2', 'GeometryItem', 'left_f', 'Frame', 'framer_box2_left_f', '-0.025,0.000,0.000', '0.000,1.571,0.000'], ['box2', 'GeometryItem', 'left_g', 'Grasp2', 'pointer_box2_left_g', '0.000,0.000,0.000', '1.000,0.000,0.000'], ['box2', 'GeometryItem', 'bottom_p', 'Place', 'pointer_box2_bottom_p', '0.000,0.000,-0.025', '0.000,0.000,1.000'], ['box2', 'GeometryItem', 'back_g', 'Grasp2', 'pointer_box2_back_g', '0.000,0.000,0.000', '0.000,-1.000,0.000'], ['box2', 'GeometryItem', 'top_g', 'Grasp2', 'pointer_box2_top_g', '0.000,0.000,0.000', '0.000,0.000,-1.000'], ['box2', 'GeometryItem', 'top_f', 'Frame', 'framer_box2_top_f', '0.000,0.000,0.025', '3.142,0.000,0.000'], ['box2', 'GeometryItem', 'right_p', 'Place', 'pointer_box2_right_p', '0.025,0.000,0.000', '-1.000,0.000,0.000'], ['box2', 'GeometryItem', 'front_f', 'Frame', 'framer_box2_front_f', '0.000,-0.025,0.000', '-1.571,0.000,0.000'], ['box2', 'GeometryItem', 'front_g', 'Grasp2', 'pointer_box2_front_g', '0.000,0.000,0.000', '0.000,1.000,0.000'], ['box2', 'GeometryItem', 'top_p', 'Place', 'pointer_box2_top_p', '0.000,0.000,0.025', '0.000,0.000,-1.000'], ['box2', 'GeometryItem', 'bottom_f', 'Frame', 'framer_box2_bottom_f', '0.000,0.000,-0.025', '0.000,0.000,0.000'], ['box2', 'GeometryItem', 'bottom_g', 'Grasp2', 'pointer_box2_bottom_g', '0.000,0.000,0.000', '0.000,0.000,1.000'], ['box2', 'GeometryItem', 'front_p', 'Place', 'pointer_box2_front_p', '0.000,-0.025,0.000', '0.000,1.000,0.000'], ['box2', 'GeometryItem', 'back_p', 'Place', 'pointer_box2_back_p', '0.000,0.025,0.000', '0.000,-1.000,0.000'], ['box2', 'GeometryItem', 'left_p', 'Place', 'pointer_box2_left_p', '-0.025,0.000,0.000', '1.000,0.000,0.000']]),
                                            table_updater_default
                                            ),
                                  TableInfo("Binder", '250px',
                                            lambda: ([IDENTIFY_COL, 'CType', 'Geometry', 'Direction', 'Point', 'Control', 'Multi'],
                                                     [['grip1', 'Grasp2', 'grip1', '0,1,0', '', 'True', 'False'], ['grip0', 'Grasp2', 'grip0', '0,1,0', '', 'True', 'False'], ['goal_bd', 'Place', 'goal', '0,0,1', '0,0,0', 'False', 'True'], ['floor', 'Place', 'floor', '0,0,1', '', 'False', 'True']]),
                                            table_updater_default
                                            )]),
              ]
             )
    run_server(debug=True)