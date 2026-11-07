"""Local-only viewer with cache-safe assets and repeatable Windows launch."""
import argparse
import errno
import hashlib
import io
import json
import os
import socket
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
from urllib.request import urlopen
import webbrowser

DIRECTORY = Path(__file__).resolve().parents[1] / 'viewer'
CACHE_MARKER = b'ttimu-ui-20261108-4'
INFO_PATH = '/__ttimu__/info'
SERVER_VERSION = 2
ROOT_ID = hashlib.sha256(os.path.normcase(str(DIRECTORY)).encode()).hexdigest()[:16]


def asset_revision():
    digest = hashlib.sha256()
    for name in ('style.css', 'app.mjs', 'telemetry.mjs'):
        digest.update(name.encode())
        digest.update((DIRECTORY / name).read_bytes())
    return digest.hexdigest()[:16]


class ViewerServer(ThreadingHTTPServer):
    # Windows SO_REUSEADDR can bind over another live server, producing mixed
    # responses. Require exclusive ownership instead of competing for its port.
    allow_reuse_address = not hasattr(socket, 'SO_EXCLUSIVEADDRUSE')

    def server_bind(self):
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


class ViewerHandler(SimpleHTTPRequestHandler):
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, '.mjs': 'text/javascript'}

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(DIRECTORY), **kwargs)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def send_head(self):
        # A previously cached asset must not produce 304 against a different UI.
        for name in ('If-Modified-Since', 'If-None-Match'):
            if name in self.headers:
                del self.headers[name]
        path = urlsplit(self.path).path
        if path == INFO_PATH:
            data = json.dumps({'app': 'ttimu-viewer', 'server_version': SERVER_VERSION,
                               'root_id': ROOT_ID, 'asset_revision': asset_revision()}).encode()
            content_type = 'application/json'
        elif path in ('/', '/index.html', '/app.mjs'):
            name = 'app.mjs' if path == '/app.mjs' else 'index.html'
            data = (DIRECTORY / name).read_bytes().replace(CACHE_MARKER, asset_revision().encode())
            content_type = 'text/javascript' if name.endswith('.mjs') else 'text/html; charset=utf-8'
        else:
            return super().send_head()
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        return io.BytesIO(data)


def existing_viewer(port):
    try:
        with urlopen('http://127.0.0.1:%d%s' % (port, INFO_PATH), timeout=0.4) as response:
            info = json.load(response)
        return (info.get('app') == 'ttimu-viewer' and info.get('root_id') == ROOT_ID
                and info.get('server_version') == SERVER_VERSION)
    except (OSError, ValueError, AttributeError):
        return False


def find_server(preferred):
    # Reuse this checkout's modern server. Never stop an unrelated/legacy server.
    for port in range(preferred, min(preferred + 10, 65536)):
        try:
            server = ViewerServer(('127.0.0.1', port), ViewerHandler)
            return server, server.server_port
        except OSError as error:
            if error.errno not in (errno.EADDRINUSE, errno.EACCES) and getattr(error, 'winerror', None) not in (10048, 10013):
                raise
            if existing_viewer(port):
                return None, port
    server = ViewerServer(('127.0.0.1', 0), ViewerHandler)
    return server, server.server_port


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--open', action='store_true', help='Open in your default browser; Chrome/Edge required for USB.')
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error('Port must be between 0 and 65535')
    server, port = find_server(args.port)
    url = 'http://localhost:%d/?v=%s' % (port, asset_revision())
    print('IMU Library by TT: ' + url, flush=True)
    if port != args.port and args.port:
        print('Port %d is busy; opened port %d instead.' % (args.port, port), flush=True)
    if args.open:
        webbrowser.open(url)
    if server is None:
        print('Using the viewer already running for this folder.', flush=True)
        return
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
