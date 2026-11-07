"""Browser QA fixture: old unversioned CSS/JS intentionally stay cacheable for a year.

Versioned routes serve the real viewer. The same browser must navigate from the
old cached page to the new dashboard without clearing its cache.
"""
import argparse
import io
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from serve_viewer import ViewerHandler


class CacheSeedHandler(ViewerHandler):
    def end_headers(self):
        if self.path.split('?')[0] == '/__test__/cache-seed' or self.path in ('/style.css', '/app.mjs'):
            self.send_header('Cache-Control', 'public, max-age=31536000')
            SimpleHTTPRequestHandler.end_headers(self)
        else:
            super().end_headers()

    def send_head(self):
        path = urlsplit(self.path).path
        if path == '/__test__/cache-seed':
            data = b'<html><head><link rel="stylesheet" href="/style.css"></head><body><main>Old UI cache</main><script type="module" src="/app.mjs"></script></body></html>'
            content_type = 'text/html'
        elif self.path == '/style.css':
            data = b'body{background:rgb(14,21,31)}main{display:grid;grid-template-columns:270px 1fr}'
            content_type = 'text/css'
        elif self.path == '/app.mjs':
            data = b'window.__ttimuOldCachedModule=true;'
            content_type = 'text/javascript'
        else:
            return super().send_head()
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        return io.BytesIO(data)


if __name__ == '__main__':
    parser = argparse.ArgumentParser();parser.add_argument('--port', type=int, default=8094);args=parser.parse_args()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), CacheSeedHandler)
    print('Cache regression fixture: http://127.0.0.1:%d' % server.server_port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
