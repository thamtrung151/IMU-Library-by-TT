"""Regression checks for cached assets and launching a viewer twice."""
import json
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
import sys
import threading
import unittest
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import serve_viewer as viewer


class SilentViewer(viewer.ViewerHandler):
    def log_message(self, *args):
        pass


class OtherApp(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(404)
        self.end_headers()

    def log_message(self, *args):
        pass


@contextmanager
def running(handler):
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


class ViewerHTTP(unittest.TestCase):
    def test_no_stale_304_and_matching_asset_revision(self):
        with running(SilentViewer) as server:
            base = 'http://127.0.0.1:%d' % server.server_port
            bodies = {}
            for path in ('/', '/style.css', '/app.mjs', '/telemetry.mjs'):
                request = Request(base + path, headers={'If-Modified-Since': 'Wed, 01 Jan 2099 00:00:00 GMT'})
                with urlopen(request, timeout=2) as response:
                    self.assertEqual(response.status, 200)
                    self.assertEqual(response.headers['Cache-Control'], 'no-store')
                    bodies[path] = response.read().decode('utf-8')
                    if path.endswith('.mjs'):
                        self.assertEqual(response.headers.get_content_type(), 'text/javascript')
            revision = viewer.asset_revision()
            self.assertIn('style.css?v=' + revision, bodies['/'])
            self.assertIn('app.mjs?v=' + revision, bodies['/'])
            self.assertIn('telemetry.mjs?v=' + revision, bodies['/app.mjs'])
            self.assertNotIn(viewer.CACHE_MARKER.decode(), bodies['/'])
            with urlopen(base + viewer.INFO_PATH) as response:
                info = json.load(response)
            self.assertEqual(info['root_id'], viewer.ROOT_ID)
            self.assertEqual(info['asset_revision'], revision)

    def test_second_launcher_reuses_only_the_same_checkout(self):
        with running(SilentViewer) as server:
            found, port = viewer.find_server(server.server_port)
            self.assertIsNone(found)
            self.assertEqual(port, server.server_port)

    def test_busy_unrelated_port_is_not_killed_or_reused(self):
        with running(OtherApp) as occupied:
            found, port = viewer.find_server(occupied.server_port)
            try:
                self.assertIsNotNone(found)
                self.assertNotEqual(port, occupied.server_port)
                self.assertEqual(occupied.fileno() >= 0, True)
            finally:
                if found:
                    found.server_close()


if __name__ == '__main__':
    unittest.main()
