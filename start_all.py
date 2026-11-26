"""
start_all.py - Master launcher for the Pharma Rover system.
Starts all subsystems in correct dependency order and launches the unified Web Server.
Handles graceful Ctrl+C shutdown.
"""
import os
import sys
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from server.server import run_server

def main():
    parser = argparse.ArgumentParser(description="Pharma Rover Unified Web Server & Subsystems Launcher")
    parser.add_argument("--port", type=int, default=8000, help="Web server port (default: 8000)")
    parser.add_argument("--sim", action="store_true", default=None, help="Deprecated; set simulation flags per subsystem in server/config.yaml")
    args = parser.parse_args()

    print("=" * 70)
    print("  LAUNCHING PHARMA ROVER UNIFIED SERVER")
    print(f"  Web Dashboard Port: {args.port}")
    print("  Subsystem Modes:    server/config.yaml")
    print("=" * 70)

    run_server(port=args.port)

if __name__ == "__main__":
    main()
