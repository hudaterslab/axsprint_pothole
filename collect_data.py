"""Foreground entry point for PTP-synchronized camera, LiDAR and GPS collection."""

import argparse
import os
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE / "app"))
os.environ.setdefault("CONFIG_PATH", str(BASE / "config.yaml"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=0)
    args = parser.parse_args()
    from collector import run

    run(args.duration)


if __name__ == "__main__":
    main()
