import dash
from dash.dependencies import Input, Output
import dash_html_components as html
import dash_core_components as dcc

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)


app.layout = html.Div([
    dcc.ConfirmDialog(
        id='confirm',
        message='Danger danger! Are you sure you want to continue?',
    ),

    dcc.Dropdown(
        options=[
            {'label': i, 'value': i}
            for i in ['Safe', 'Danger!!']
        ],
        id='dropdown'
    ),
    html.Div(id='output-confirm')
])


@app.callback(Output('confirm', 'displayed'),
              [Input('dropdown', 'value')])
def display_confirm(value):
    if value == 'Danger!!':
        return True
    return False


if __name__ == '__main__':
    app.run_server(debug=True)