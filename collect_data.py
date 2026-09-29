"""Foreground entry point for PTP-synchronized camera, LiDAR and GPS collection."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "app"))

if __name__ == "__main__":
    from collector import run

    run()
