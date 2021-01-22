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

print("Start host server - main page is <IP>:{}/index.html".format(args.port))

app.run(host="0.0.0.0", port=args.port)