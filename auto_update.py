"""live_detection 단말기 업데이트: GitHub 코드, Hugging Face 모델, Tailscale 연결.

코드는 공개 GitHub 저장소에서만 받습니다. 사내 GitLab의 live_detection은 GitLab의
저장소 미러링이 GitHub로 보내므로, 단말기에는 GitLab 주소나 업로드용 인증 정보가 없습니다.

--check는 파일/설정을 변경하지 않습니다.

모델 교체 후 실행 중인 live_pothole.py가 새 모델을 사용하려면 재실행이 필요합니다.
이 스크립트는 수집/분석 프로세스를 자동으로 재시작하지 않습니다.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import urllib.request
from datetime import datetime


WORK_DIR = Path(__file__).resolve().parent
HF_REPO_ID = "HudatersU/road_maintanance"
HF_REVISION = "main"
HF_API_URL = f"https://huggingface.co/api/models/{HF_REPO_ID}/revision/{HF_REVISION}"
HF_MODELS = ["best_seg.dxnn"]
GITHUB_URL = "https://github.com/hudaterslab/axsprint_pothole.git"
GITHUB_BRANCH = "live_detection"


def log(message):
    print(f"[{datetime.now():%Y-%m-%d %H:%M:%S}] {message}", flush=True)


def run_command(command, *, timeout=30, env=None):
    result = subprocess.run(
        command, capture_output=True, text=True, timeout=timeout, env=env,
    )
    if result.returncode:
        detail = result.stderr.strip() or result.stdout.strip() or f"exit={result.returncode}"
        raise RuntimeError(f"{command[0]}: {detail}")
    return result.stdout.strip()


def update_github(check_only=False):
    """지정한 브랜치만 fast-forward로 갱신하며 단말기 수정은 덮어쓰지 않습니다."""
    def git(*args, timeout=30):
        env = {**os.environ, "GIT_TERMINAL_PROMPT": "0"}
        if check_only:
            env["GIT_OPTIONAL_LOCKS"] = "0"
        return run_command(
            ["git", "-C", str(WORK_DIR), *args], timeout=timeout,
            env=env,
        )

    if not (WORK_DIR / ".git").exists():
        raise RuntimeError("Git 저장소 연결이 필요합니다. 기존 파일은 유지합니다.")
    if Path(git("rev-parse", "--show-toplevel")).resolve() != WORK_DIR.resolve():
        raise RuntimeError("Git 작업 폴더가 live_detection과 다릅니다.")
    if git("remote", "get-url", "origin") != GITHUB_URL:
        raise RuntimeError(f"origin이 지정된 GitHub 저장소가 아닙니다: {GITHUB_URL}")
    if git("symbolic-ref", "--quiet", "--short", "HEAD") != GITHUB_BRANCH:
        raise RuntimeError(f"현재 브랜치가 {GITHUB_BRANCH}가 아닙니다.")
    # reset --hard, clean, 자동 stash를 하지 않습니다. 수정된 코드는 먼저 원본에 반영합니다.
    if git("status", "--porcelain", "--untracked-files=no"):
        raise RuntimeError("단말기에 커밋되지 않은 코드 수정이 있어 GitHub 갱신을 중단합니다. "
                           "수정 내용은 GitLab에 올린 뒤 받으세요.")
    local_sha = git("rev-parse", "HEAD")
    ref = f"refs/heads/{GITHUB_BRANCH}"
    if check_only:
        lines = git("ls-remote", "--exit-code", "origin", ref, timeout=60).splitlines()
        matches = [line.split()[0] for line in lines if line.split()[-1] == ref]
        if len(matches) != 1:
            raise RuntimeError("GitHub 브랜치의 커밋을 확인할 수 없습니다.")
        remote_sha = matches[0]
        if local_sha == remote_sha:
            log(f"GitHub {GITHUB_BRANCH}: 최신 코드와 동일합니다 ({local_sha[:12]}).")
        else:
            log(f"GitHub {GITHUB_BRANCH}: 새 커밋 확인 ({local_sha[:12]} → {remote_sha[:12]}). "
                "확인 모드이므로 다운로드/적용하지 않습니다.")
        return
    tracking = f"refs/remotes/origin/{GITHUB_BRANCH}"
    git("fetch", "--no-tags", "origin", f"+{ref}:{tracking}", timeout=180)
    remote_sha = git("rev-parse", tracking)
    if local_sha == remote_sha:
        log(f"GitHub {GITHUB_BRANCH}: 최신 코드와 동일합니다 ({local_sha[:12]}).")
        return
    # 미러의 이력이 바뀌거나 로컬 커밋이 있으면 적용하지 않습니다.
    git("merge-base", "--is-ancestor", local_sha, remote_sha)
    git("merge", "--ff-only", "--no-edit", "--no-overwrite-ignore", remote_sha, timeout=60)
    log(f"GitHub {GITHUB_BRANCH}: 코드 갱신 완료 ({local_sha[:12]} → {remote_sha[:12]}). "
        "실행 중인 프로그램에는 재실행 후 적용됩니다.")


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download_model(url, destination, expected_sha, expected_size):
    """같은 디스크의 임시 파일에 받고, 검증에 성공한 경우에만 교체합니다."""
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=destination.parent, prefix=f".{destination.name}.",
            suffix=".part", delete=False,
        ) as output:
            temporary = Path(output.name)
            digest = hashlib.sha256()
            received = 0
            with urllib.request.urlopen(url, timeout=60) as response:
                for chunk in iter(lambda: response.read(1024 * 1024), b""):
                    received += len(chunk)
                    if received > expected_size:
                        raise RuntimeError("다운로드 파일이 예상 크기보다 큽니다.")
                    output.write(chunk)
                    digest.update(chunk)
            if received != expected_size or digest.hexdigest() != expected_sha:
                raise RuntimeError("모델 크기/SHA-256 불일치: 기존 모델을 유지합니다.")
            output.flush()
            os.fsync(output.fileno())
        temporary.chmod(0o644)
        os.replace(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def update_huggingface(check_only=False):
    with urllib.request.urlopen(f"{HF_API_URL}?blobs=true", timeout=30) as response:
        metadata = json.load(response)
    revision = metadata.get("sha", "")
    if not isinstance(revision, str) or not re.fullmatch(r"[0-9a-f]{40,64}", revision):
        raise RuntimeError("Hugging Face 응답에 유효한 커밋 해시가 없습니다.")
    files = {item["rfilename"]: item for item in metadata.get("siblings", [])}
    for name in HF_MODELS:
        info = files.get(name, {})
        lfs = info.get("lfs") or {}
        expected_sha = lfs.get("sha256", "")
        expected_size = lfs.get("size")
        if (not isinstance(expected_sha, str)
                or not re.fullmatch(r"[0-9a-f]{64}", expected_sha)
                or not isinstance(expected_size, int) or expected_size <= 0):
            raise RuntimeError(f"{name}: 다운로드 검증 정보가 없거나 잘못되었습니다.")
        destination = WORK_DIR / name
        if (destination.is_file() and destination.stat().st_size == expected_size
                and file_sha256(destination) == expected_sha):
            log(f"{name}: 최신 모델과 동일합니다.")
            continue
        if check_only:
            log(f"{name}: 다운로드 필요. 확인 모드이므로 기존 파일을 유지합니다.")
            continue
        # 조회한 커밋에 고정하여 다운로드 도중 main이 바뀌어도 일관성을 유지합니다.
        url = f"https://huggingface.co/{HF_REPO_ID}/resolve/{revision}/{name}"
        log(f"{name}: 새 모델 다운로드 시작 ({expected_size:,} bytes).")
        download_model(url, destination, expected_sha, expected_size)
        log(f"{name}: 검증 및 교체 완료. 실행 중인 분석 프로그램에는 재실행 후 적용됩니다.")


def tailscale_healthy():
    """이 단말기의 Tailscale이 접속돼 있는지 확인합니다(단말기마다 다른 IP는 보지 않음)."""
    state = json.loads(run_command(["tailscale", "status", "--json"], timeout=20))
    own = state.get("Self") or {}
    return (state.get("BackendState") == "Running"
            and own.get("Online") is True
            and bool(state.get("TailscaleIPs")))


def check_tailscale(check_only=False):
    try:
        healthy = tailscale_healthy()
    except (OSError, subprocess.SubprocessError, ValueError, RuntimeError) as exc:
        log(f"Tailscale 상태 조회 실패: {exc}")
        healthy = False
    if healthy:
        log("Tailscale 정상")
        return
    if check_only:
        raise RuntimeError("Tailscale 연결 확인 필요. 확인 모드이므로 복구 명령은 실행하지 않습니다.")
    # cron에서 비밀번호 입력 대기로 멈추지 않도록 sudo -n을 사용합니다.
    prefix = [] if os.geteuid() == 0 else ["sudo", "-n"]
    log("Tailscale 복구 시도. 일반 사용자로 예약 실행 시 해당 명령의 sudo 권한이 필요합니다.")
    run_command(prefix + ["systemctl", "start", "tailscaled"])
    run_command(prefix + ["tailscale", "up", "--timeout=30s"], timeout=45)
    if not tailscale_healthy():
        raise RuntimeError("Tailscale 복구 후에도 접속되지 않았습니다. 로그인/설정을 확인하세요.")
    log("Tailscale 복구 확인 완료")


def run_updates(check_only):
    success = True
    # 각 작업의 실패를 분리하여 네트워크 오류가 다른 점검을 막지 않습니다.
    for task in (check_tailscale, update_github, update_huggingface):
        try:
            task(check_only=check_only)
        except Exception as exc:
            success = False
            log(f"{task.__name__} 실패: {exc}")
    return 0 if success else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="읽기 전용 상태 확인")
    args = parser.parse_args()
    if args.check:
        return run_updates(True)
    # Linux 단말기에서 중복 실행으로 모델 교체가 겹치지 않게 합니다.
    import fcntl

    runtime_dir = WORK_DIR / "var"
    runtime_dir.mkdir(exist_ok=True)
    with (runtime_dir / "auto_update.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            log("다른 업데이트 작업이 실행 중이므로 건너뜁니다.")
            return 0
        return run_updates(False)


if __name__ == "__main__":
    raise SystemExit(main())
