#!/usr/bin/env python3
"""Send the detections listed in the date folders' porthole_detections.csv to the server.

For use back at the office after driving without internet, when main_live.py could only
list the detections in the CSV. Close the analysis terminal first (this needs the NPU),
then run:  python3 senddata.py

Every date folder's CSV is read and frames the server already has are skipped, so it can
be run again at any time. Each remaining frame's JPG/JSON/PCAP are rebuilt the way
main_live.py builds them: the WARMUP_FRAMES frames before it are analysed again, so the
tracker reports the same new objects as on the road, and the files are sent like a live
upload and deleted from /tmp afterwards.
"""

import collections
import csv
import fcntl
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import main_live as ml

WARMUP_FRAMES = 15  # frames analysed before each listed frame, so tracking matches the road

ml.UPLOAD_STAGING = Path("/tmp/porthole_senddata")


class Quiet:
    """main_live's CSV writer for the frames that are only analysed for tracking."""

    def add(self, key, timestamp, image_path, detections):
        pass


class Listed(Quiet):
    """main_live's CSV writer for the listed frames: remembers what each one reported."""

    def __init__(self):
        self.kinds = {}

    def add(self, key, timestamp, image_path, detections):
        self.kinds[(key, Path(image_path).name)] = "+".join(
            sorted({"pothole" if int(d.class_id) == 1 else "crack" for d in detections})
        )


class Bundles:
    """main_live's uploader for the listed frames: keeps their files until they are sent."""

    def __init__(self):
        self.ready = {}

    def accepting(self):
        return True

    def send(self, key, index, folder, manifest):
        self.ready[(key, index)] = (folder, manifest)


class Replay(ml.LiveProcessor):
    """main_live's processor; only the listed frames (targets) are written up."""

    def __init__(self):
        super().__init__(Listed(), Bundles())
        self.targets = set()

    def report(self, run, key, frame):
        if int(frame.row["frame_index"]) in self.targets:
            return super().report(run, key, frame)
        log, uploader = self.log, self.uploader
        self.log, self.uploader = Quiet(), None
        try:
            return super().report(run, key, frame)
        finally:
            self.log, self.uploader = log, uploader


def on_server(options, key):
    """File names the server already has for this run (the HTTP API answers repeats itself)."""
    if options.mode != "ssh":
        return set()
    folder = f"{options.destination.rstrip('/')}/{ml.REMOTE_FOLDER}/{key}/certifcate"
    result = subprocess.run(
        ["ssh", "-i", str(options.key), "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
         "-o", "ConnectTimeout=10", f"{options.user}@{options.host}",
         f"ls -1 {shlex.quote(folder)} 2>/dev/null || true"],
        capture_output=True, text=True, timeout=60,
    )
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or f"ssh exit {result.returncode}")
    return set(result.stdout.split())


def pcaps_for(run_path):
    """The run's PCAPs plus the last two of the run before it, as run_service gives them."""
    runs = sorted(ml.discover(ml.ROOT), key=str)
    before = [p for p in runs if str(p) < str(run_path)]
    extra = []
    if before:
        previous = ml.Run(before[-1])
        previous.refresh(with_frames=False)
        extra = list(previous.pcap_records())[-2:]
    run = ml.Run(run_path)
    run.refresh()
    pcaps = sorted(extra + list(run.pcap_records()), key=lambda p: float(p["first_timestamp"]))
    return run.frames.rows, pcaps


def rebuild(processor, key, run_path, indices):
    """Analyse the listed frames of one run again; their files end up in processor.uploader."""
    rows, pcaps = pcaps_for(run_path)
    position = {int(row["frame_index"]): i for i, row in enumerate(rows)}
    wanted = sorted(position[i] for i in indices if i in position)
    segments = []  # listed frames whose warm-up windows touch are analysed in one pass
    for p in wanted:
        if segments and p - WARMUP_FRAMES <= segments[-1][1] + 1:
            segments[-1][1] = p
        else:
            segments.append([max(0, p - WARMUP_FRAMES), p])
    processor.prune_pcaps(pcaps)
    processor.refresh_run(run_path)
    for start, end in segments:
        processor.tracker = ml.ObjectTracker()
        processor.targets = {int(rows[p]["frame_index"]) for p in wanted if start <= p <= end}
        stream = processor.infer(run_path, rows[start:end + 1])
        analysed = processor.analyse(run_path, key, stream, pcaps, lambda: False)
        try:
            for _ in analysed:
                pass
        finally:
            analysed.close()
            stream.close()


def upload(options, key, bundles):
    """Send one run's files; returns (sent, error)."""
    uploader = (
        ml.HttpUploader(options)
        if options.mode == "api"
        else ml.PersistentUploader(options, Path(ml.REMOTE_FOLDER) / key / "certifcate")
    )
    sent = 0
    try:
        for (_, index), (folder, manifest) in sorted(bundles.items()):
            uploader.upload(folder / "certifcate", manifest)
            sent += 1
            print(f"  보냄: {key} 프레임 {index}", flush=True)
    except Exception as exc:
        return sent, str(exc)
    finally:
        uploader.close()
    return sent, ""


def main():
    lock = open("/tmp/porthole_main_live.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("분석 프로그램(main_live.py)이 실행 중이에요. NPU를 같이 쓸 수 없으니 분석 창을 닫고 다시 실행하세요.")
    try:
        options = ml.upload_options()
    except ValueError as exc:
        sys.exit(f"서버 설정(.env)이 없어요: {exc}")
    tables = sorted(ml.ROOT.glob(f"*/{ml.DETECTIONS_CSV}"))
    if not tables:
        print("보낼 검출 기록(CSV)이 없어요.")
        return 0
    # The recorder may be running; it comes first, as for main_live.py.
    os.nice(10)
    if shutil.which("ionice"):
        subprocess.run(["ionice", "-c", "3", "-p", str(os.getpid())], check=False)
    total = collections.Counter()
    processor = None
    try:
        for table in tables:
            date = table.parent.name
            by_run = collections.defaultdict(dict)
            with table.open(newline="", encoding="utf-8") as stream:
                for item in csv.DictReader(stream):
                    run_name, _, image = item["image"].partition("/")
                    by_run[run_name][Path(image).name] = item["objects"]
            for run_name, listed in sorted(by_run.items()):
                key, run_path = f"{date}/{run_name}", table.parent / run_name
                total["listed"] += len(listed)
                if not (run_path / "frames/frames.jsonl").is_file():
                    print(f"{key}: 녹화 폴더가 없어 {len(listed)}건을 건너뜀")
                    total["missing"] += len(listed)
                    continue
                rows = {Path(r["path"]).name: r for r in ml.JsonlTail(run_path / "frames/frames.jsonl").read()}
                gone = [name for name in listed if name not in rows]
                if gone:
                    print(f"{key}: 녹화 기록에 없는 프레임 {len(gone)}건을 건너뜀")
                    total["missing"] += len(gone)
                have = on_server(options, key)
                todo = {
                    int(rows[name]["frame_index"]): (name, objects)
                    for name, objects in listed.items()
                    if name in rows and not all(
                        ml.detection_frame_stem(rows[name]) + ext in have for ext in (".jpg", ".json", ".pcap")
                    )
                }
                already = len(listed) - len(gone) - len(todo)
                total["already"] += already
                print(f"{key}: CSV {len(listed)}건, 서버에 이미 있음 {already}, 보낼 것 {len(todo)}", flush=True)
                if not todo:
                    continue
                if processor is None:
                    processor = Replay()
                processor.uploader.ready.clear()
                rebuild(processor, key, run_path, set(todo))
                bundles = processor.uploader.ready
                for index, (name, objects) in sorted(todo.items()):
                    found = processor.log.kinds.get((key, name))
                    if (key, index) not in bundles:
                        total["not_rebuilt"] += 1
                        print(f"  못 만듦: 프레임 {index} ({objects}) - 다시 분석했을 때 같은 검출이 나오지 않거나 PCAP이 없음")
                    elif found != objects:
                        total["differs"] += 1
                        print(f"  종류 다름: 프레임 {index} CSV {objects} / 다시 분석 {found} (다시 분석한 것으로 보냄)")
                sent, error = upload(options, key, bundles)
                total["sent"] += sent
                for folder, _ in bundles.values():
                    shutil.rmtree(folder, ignore_errors=True)
                if error:
                    total["failed"] += len(bundles) - sent
                    print(f"  전송 실패, 나머지 {len(bundles) - sent}건은 다음에 다시 실행하세요: {error}")
                    return 1
    finally:
        if processor:
            processor.close()
        shutil.rmtree(ml.UPLOAD_STAGING, ignore_errors=True)
    print(
        f"끝: CSV {total['listed']}건 중 서버에 이미 있음 {total['already']}, 보냄 {total['sent']}, "
        f"못 만듦 {total['not_rebuilt']}, 종류 다름 {total['differs']}, 녹화 폴더 없음 {total['missing']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
