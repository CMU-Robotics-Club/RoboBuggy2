#! /usr/bin/env python3
import http.server
import argparse

PORT = 8760

# python3 asset_server.py --directory="/rb_ws/src/buggy/assets"

parser = argparse.ArgumentParser(description='Run a simple HTTP server')
parser.add_argument('--directory', type=str, help='Directory to serve')
parsed_args, _ = parser.parse_known_args()

class DirectoryHandler(http.server.SimpleHTTPRequestHandler):
  def __init__(self, *args, **kwargs):
    super().__init__(*args, directory=parsed_args.directory, **kwargs)


with http.server.HTTPServer(("", PORT), DirectoryHandler) as httpd:
    httpd.serve_forever()
