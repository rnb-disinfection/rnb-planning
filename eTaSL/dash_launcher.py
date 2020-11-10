# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.

import dash
import dash_core_components as dcc
import dash_html_components as html
import dash_table
from dash.dependencies import Input, Output

external_stylesheets = []

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)
app.title = "TAMP_RNB"

def generate_table(columns, items, height="100%"):
    return [
        dash_table.DataTable(
            id='datatable-row-ids',
            columns=[{'name': col, 'id': col, 'deletable': False} for col in columns],
            data=[{k:v for k,v in zip(columns, item)} for item in items],
            editable=True,
            filter_action="native",
            sort_action="native",
            sort_mode='multi',
            row_selectable='multi',
            row_deletable=True,
            selected_rows=[],
            style_table={
                'height': height,
                'overflowY': 'scroll',
                'fontSize': '16px'
            },
            style_header={'backgroundColor': 'rgb(27, 30, 36)',
                          'color': 'white',
                          'fontWeight': 'bold',
                          'textAlign': 'center'},
            style_data_conditional=[
                                       {
                                           'if': {'column_id': c},
                                           'fontSize': '12px'
                                       } for c in ['Dims', 'Center', 'Rpy', 'Point', 'Dir']
                                   ]+[{
                'if': {'row_index': 'even'},
                'backgroundColor': '#F5F5F5'
            }],
            style_cell={
                'boxShadow': '0 0',
                'border': '0px solid #ffffff',
                'borderBottom': '1px solid lightgray'}
        ),
        html.Div(id='datatable-row-ids-container')
    ]

data_headers = ["name", "Type", "Position(mm)", "Orientation(rad)"]
data_items = [["box0", "box", "0,0,0","0,0,0"],
              ["box2", "box", "0,0,0","0,0,0"],
              ["box3", "box", "0,0,0","0,0,0"]
              ]

def get_tabs():
    return [
        dcc.Tab(label='Geometry', value='tab-geometry',
                className='custom-tab', selected_className='custom-tab--selected'),
        dcc.Tab(label='Binding', value='tab-binding',
                className='custom-tab', selected_className='custom-tab--selected'),
    ]

app.layout = html.Div(className="full-screen", children=[
    html.Div(className="column-left", children=[
        dcc.Tabs(id='tabs-left', value='tab-geometry',
                 parent_className='custom-tabs',
                 className='custom-tabs-container',
                 children=get_tabs()),
        html.Div(id='tabs-left-content')]
             ),
    html.Div(className="column-mid", children=[]),
    html.Div(className="column-right", children=[
        dcc.Tabs(id='tabs-right', value='tab-binding',
                 parent_className='custom-tabs',
                 className='custom-tabs-container',
                 children=get_tabs()),
        html.Div(className='custom-content-container', id='tabs-right-content')]
             )
])



@app.callback(Output('tabs-left-content', 'children'),
              [Input('tabs-left', 'value')])
def render_content_left(tab):
    return __render_content_func(tab)

@app.callback(Output('tabs-right-content', 'children'),
              [Input('tabs-right', 'value')])
def render_content_right(tab):
    return __render_content_func(tab)

@app.callback(
    Output('datatable-row-ids-container', 'children'),
    [Input('datatable-row-ids', 'derived_virtual_row_ids'),
     Input('datatable-row-ids', 'selected_row_ids'),
     Input('datatable-row-ids', 'active_cell')])
def update_graphs(row_ids, selected_row_ids, active_cell):
    # When the table is first rendered, `derived_virtual_data` and
    # `derived_virtual_selected_rows` will be `None`. This is due to an
    # idiosyncrasy in Dash (unsupplied properties are always None and Dash
    # calls the dependent callbacks when the component is first rendered).
    # So, if `rows` is `None`, then the component was just rendered
    # and its value will be the same as the component's dataframe.
    # Instead of setting `None` in here, you could also set
    # `derived_virtual_data=df.to_rows('dict')` when you initialize
    # the component.
    selected_id_set = set(selected_row_ids or [])
    return []


def __render_content_func(tab):
    if tab == 'tab-geometry':
        return html.Div(className="custom-tab-content", children=
        [html.Div(className="row-header", children="Geometry")]
        + generate_table(*get_geometry_data(), height="850px")
                        )
    elif tab == 'tab-binding':
        return html.Div(className="custom-tab-content", children=[
            html.Div(className="row-header", children="Handle"),
            html.Div(className="row-top", children=generate_table(*get_handle_data(),
                                                                  height="570px")),
            html.Div(className="row-header", children="Binder"),
            html.Div(className="row-bottom", children=generate_table(*get_binder_data(),
                                                                     height="250px")),
        ])

__geo_loader = None
__binder_loader = None
__handle_loader = None

def set_table_data_loaders(geo_loader, handle_loader, binder_loader):
    global __geo_loader, __handle_loader, __binder_loader
    __geo_loader, __handle_loader, __binder_loader = \
        geo_loader, handle_loader, binder_loader

def get_geometry_data():
    if __geo_loader is None:
        return data_headers, data_items
    else:
        return __geo_loader()

def get_binder_data():
    if __binder_loader is None:
        return data_headers, data_items
    else:
        return __binder_loader()

def get_handle_data():
    if __handle_loader is None:
        return data_headers, data_items
    else:
        return __handle_loader()

from threading import Thread

def run_server(on_background=False, **kwargs):
    if on_background:
        __thread = Thread(target=app.run_server, kwargs=kwargs)
        __thread.start()
    else:
        app.run_server(**kwargs)

if __name__ == '__main__':
    app.run_server(debug=True)