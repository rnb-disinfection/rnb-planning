import argparse
from flask import Flask, send_from_directory
app = Flask(__name__, static_folder="html")


parser = argparse.ArgumentParser(description='Host doxygen document - main page is <IP>:<PORT>/index.html')
parser.add_argument('--port', type=int, default=5000,
                    help='(Optional) port number to host')
args = parser.parse_args()


@app.route('/<path:path>')
def static_file(path):
    return app.send_static_file(path)


@app.after_request
def add_header(r):
    """
    Add headers to both force latest IE rendering engine or Chrome Frame,
    and also to cache the rendered page for 10 minutes.
    """
    r.headers["Accept"] = 'application/json'
    r.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    r.headers["Pragma"] = "no-cache"
    r.headers["Expires"] = "0"
    # r.headers['Cache-Control'] = 'public, max-age=0'
    return r


print("Start host server - main page is <IP>:{}/index.html".format(args.port))

app.run(host="0.0.0.0", port=args.port)