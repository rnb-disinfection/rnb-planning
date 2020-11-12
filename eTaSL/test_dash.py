import dash
from dash.dependencies import Input, Output
import dash_html_components as html
import dash_core_components as dcc

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)
import dash_split_pane

app.layout = html.Div(
    dash_split_pane.DashSplitPane(
        children=[html.Div("LEFT PANE"), html.Div("RIGHT PANE")],
        id="splitter",
        split="vertical",
        size="60%",
    )
)

if __name__ == '__main__':
    app.run_server(debug=True)