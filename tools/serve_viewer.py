"""Local-only static viewer. No serial data is uploaded anywhere."""
import argparse
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import mimetypes

mimetypes.add_type('text/javascript','.mjs')
parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8080);args=parser.parse_args()
directory=Path(__file__).resolve().parents[1]/'viewer'
server=ThreadingHTTPServer(('127.0.0.1',args.port),partial(SimpleHTTPRequestHandler,directory=str(directory)))
print(f'IMU Library by TT: http://localhost:{server.server_port}',flush=True)
try: server.serve_forever()
except KeyboardInterrupt: pass
finally: server.server_close()
