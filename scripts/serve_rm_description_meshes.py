#!/usr/bin/env python3
"""Serve rm_description assets for Foxglove mesh loading."""

import argparse
import http.server
import os
import socketserver
import subprocess
import sys


def find_rm_description_path() -> str:
    """Return the absolute path of the rm_description package."""
    try:
        result = subprocess.run(
            ["rospack", "find", "rm_description"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError("cannot resolve package 'rm_description' via rospack") from exc
    return result.stdout.strip()


class CorsHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    """HTTP handler with permissive CORS for Foxglove clients."""

    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        super().end_headers()

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self.end_headers()


class ReusableTCPServer(socketserver.TCPServer):
    """TCP server that can quickly restart on the same port."""

    allow_reuse_address = True


def main() -> int:
    """Parse arguments and start the static mesh server."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8766)
    args, _ = parser.parse_known_args()

    try:
        package_root = find_rm_description_path()
    except RuntimeError as exc:
        print(f"[foxglove_mesh_server] ERROR: {exc}", file=sys.stderr)
        return 1

    if not os.path.isdir(package_root):
        print(
            f"[foxglove_mesh_server] ERROR: package path does not exist: {package_root}",
            file=sys.stderr,
        )
        return 1

    os.chdir(package_root)
    print(
        f"[foxglove_mesh_server] Serving {package_root} at http://{args.host}:{args.port}",
        flush=True,
    )

    with ReusableTCPServer((args.host, args.port), CorsHTTPRequestHandler) as server:
        server.serve_forever()


if __name__ == "__main__":
    sys.exit(main())
