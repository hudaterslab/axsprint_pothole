# PTP 수집기와 실시간 포트홀 탐지

현재 운영에 필요한 실행 코드와 설정만 모아 둔 폴더입니다.

```text
live_detection/
  main_live.py      실시간 포트홀 탐지 (카메라 탐지 + 라이다 깊이 측정)
  camera_calib_best_effort.json   카메라 렌즈 보정값과 카메라 위치·방향 (라이다 바로 아래 9 cm, main_live.py가 읽음)
  XT32_Angle_Correction_File.csv      라이다 채널 각도 (main_live.py가 읽음)
  best_seg.dxnn     main_live.py 모델 (저장소에 넣지 않음, ./download_models.sh로 받음)
  download_models.sh   모델 다운로드 (Hugging Face HudatersU/road_maintanance, sha256 확인)
  .env.example      main_live.py 설정 예시 (.env로 복사해 채움, .env는 저장소 제외)
  collect_data.py   카메라·라이다·GPS 수집 실행 파일
  ptp_service.py    공통 PTP 기준 시계 실행 파일
  launch_component_terminal.sh   수집 터미널 열기
  run_component_foreground.sh    수집 실행·로그·정상 종료 제어
  run_main_live.sh               main_live.py 실행, 오류로 끝나면 자동 재시작 (분석 터미널이 사용)
  checklidarcamera.py            녹화 구간 프레임에 라이다 점을 겹쳐 저장 (코드 상단 FOLDER, START_FRAME, END_FRAME 설정)
  attach_component_terminal.sh   원격 데스크톱에서 수집 터미널 연결
  resolve_recording_storage.sh   외장 SSD와 저장 경로 확인
  config.yaml      카메라, GPS, 저장 위치 등 수집 설정
  install.sh       새 단말기 설치 (아래 "새 단말기 설치")
  install_detection.sh   main_live.py 부팅 시 자동 실행 (아래 "실시간 탐지 자동 실행")
  deploy/          install.sh가 설치하는 시스템 파일 원본
  app/
    collector.py   수집 루프·GPS·JPEG 저장·재연결·10분 폴더 전환
    camera.py      하드웨어 카메라 디코딩·GPU JPEG 인코딩·PTP 프레임 큐 (별도 worker 포함)
    lidar.py       XT32 패킷 해석·진단·PCAP 저장
    ptp.py         PTP 상태 검사·센서 제어·RTP/RTCP 시각 변환
  var/             실행 중 생성되는 PTP 설정·상태·로그
  runs/            /mnt/ssd/porthole_runs 바로가기
  README.md
```

`app`은 4개 모듈이며 따로 실행할 필요가 없습니다. `collect_data.py`가 수집기를,
`ptp_service.py`가 PTP 기준 시계를 실행합니다. 카메라 worker는 `camera.py --worker`로
별도 프로세스에서 실행해 GStreamer/VAAPI와 사용자 OpenCV 라이브러리를 분리합니다.
현재 사용하지 않는 LCZ·BIN 저장, 프레임별 LiDAR 잘라 저장하기, 비PTP 시각 추정,
PyAV·OpenCV·FFmpeg 대체 수신 경로, 원본 영상 복사와 화면 표시 코드는 제거했습니다.
PTP 기능을 기존 클래스에 실행 중 덮어씌우던 연결도 없애고 수집 코드에 직접 통합했습니다.
항상 켜 두던 설정 스위치(`record_enabled`, `lidar_enabled` 등)와 코드가 읽지 않는 설정 키도 없앴습니다.
`config.yaml`에는 수집기가 실제로 읽는 값만 있습니다.
Python이 만드는 `app/__pycache__`는 실행 캐시입니다.
`var`가 없어도 시작 시 다시 만들며, 일반 사용자 수집기가 PID를 기록할 수 있도록
PTP 서비스와 실행 도우미가 디렉터리 소유자를 `hudaters`로 확인합니다.

## 사용

단말기 부팅 시 PTP 기준 시계 서비스가 시작하고, 기존 LXQt 자동 로그인 후
기존 수집 터미널이 열립니다. 수집 로그를 그 터미널에서 볼 수 있습니다.
PTP 기준 시계도 기존 수집기처럼 최초 NTP 동기화를 최대 60초 기다립니다.
NTP가 꺼져 있거나 시간 안에 응답하지 않으면 단말기 RTC 시각으로 시작합니다.
기준 시계 준비 시에만 남은 NTP 시각 차이를 먼저 보정하고,
카메라 SDK의 PTP 종료·시작 명령으로 카메라의 이전 동기화 상태를 초기화합니다.
단말기 인터넷 시간 동기화는 `chrony.service`가 담당합니다. 설정은
`/etc/chrony/chrony.conf`이며 시간 보정 속도를 5ppm으로 제한합니다.
두 LAN 포트의 하드웨어 시계는 같은 단말기 시각을 따르며, `phc2sys`의
보정 속도도 제한합니다. 포트 간 시계 차이는 하드웨어 동시 측정으로 검사합니다.
터미널에서 Ctrl+C를 누르거나 창을 닫으면 수집 파일을 마감하고 수집만 종료합니다.
PTP 기준 시계는 계속 동작합니다. 분석 프로세스 자동 실행은 꺼져 있습니다.

수집 터미널을 다시 열려면 다음을 실행합니다.

```bash
/home/hudaters/Desktop/live_detection/launch_component_terminal.sh collector
```

터미널 로그는 기존처럼 `/tmp/porthole_collector_foreground_날짜_시간_PID.log`입니다.
자동 실행 항목은 `~/.config/autostart/porthole-collector-terminal.desktop`입니다.
자동 실행 항목은 이 폴더의 `launch_component_terminal.sh collector`를 실행합니다.
터미널 실행, 수집 실행·종료 제어, 원격 터미널 연결, SSD 확인 스크립트를 모두
이 폴더로 옮겼습니다. 현재 수집 실행에는 `porthole` 폴더의 코드가 필요하지 않습니다.
이 폴더의 스크립트는 `collector`(수집)와 `analysis`(`main_live.py`) 두 가지만 실행합니다.
이전 `porthole` 폴더의 같은 이름 스크립트 4개는 예전 `ptp_pothole` 폴더를 가리키는 링크이며
이 폴더와 관계없습니다.
고정 실행 도우미 `/usr/local/sbin/ptp-pothole-collector-foreground`가
일반 사용자 수집기에 PTP 패킷 확인에 필요한 네트워크 권한을 제공합니다.
이 도우미와 systemd·sudoers·LXQt 자동 시작 등록은 운영체제의 표준 위치에 유지합니다.
원본은 `deploy/`에 있으며 `install.sh`가 설치합니다.
도우미는 PTP 서비스를 시작 요청한 다음 이 폴더의 `collect_data.py`를 실행합니다.
`porthole-collector.service`는 중복 수집 방지를 위해 비활성화했습니다.
기준 시계 상태는 `systemctl status ptp-pothole-master.service`로 확인합니다.

수집은 `/mnt/ssd/porthole_runs/날짜/시간/` 아래 `frames / lidar / gps / meta`에 저장합니다.
PCAP와 목록 파일은 `lidar/00000000.pcap`, `lidar/pcaps.jsonl`처럼 `lidar` 바로 아래에
저장합니다. `lidar_pcap` 하위 폴더는 새로 만들지 않습니다. 변경 전 run의 파일은 기존 위치를 유지합니다.
10분마다 새 run으로 전환합니다. 카메라 30fps, 라이다 20Hz 설정을 유지합니다.
전원이 갑자기 꺼져도 그때까지 디스크에 확정한 데이터는 남습니다. 사진은 디스크에 확정한 뒤 `frames.jsonl`에 올리고,
라이다는 10초 파일 단위로, GPS는 2초마다 확정합니다. 꺼진 폴더에는 마감 기록(`run_finished`)이 없으므로
다음에 수집기가 켜질 때 `status: interrupted` 마감 기록을 붙입니다. main_live는 마감 기록이 있어야
그 폴더를 끝난 것으로 보고 다음 폴더로 넘어가기 때문입니다.
카메라 JPEG은 카메라 worker 안에서 GPU(VA-API `vaapijpegenc`, 품질 `jpeg_quality`)로 만들고,
수집기는 그 JPEG을 그대로 저장합니다. CPU로 인코딩할 때보다 수집기 CPU가 절반 정도로 줄어듭니다.
GPU 인코더는 동영상 색 범위(16~235)를 그대로 담아서, CPU(OpenCV) 인코딩보다 명암이 약 12% 낮고
검은색이 조금 떠 보입니다. `config.yaml`의 `camera_gpu_jpeg: false`로 두면 이전처럼 CPU로 인코딩합니다.
라이다 수신 스레드는 패킷 tail의 날짜 변환을 초마다 한 번만 하고 카메라 화각 판정은 미리 만든 표로 찾습니다.
PTP Announce를 받는 raw 소켓에는 커널 필터를 걸어 PTP 패킷(UDP 319/320)만 받습니다. 필터가 없으면 같은 랜의
라이다 패킷 초당 5000개가 모두 이 소켓에도 들어옵니다. 두 변경으로 수집기 CPU가 한 코어의 77%에서 57%로
줄었고, 저장되는 내용은 이전과 같습니다.
라이다는 내장 LAN `enp1s0`, 카메라는 `enp2s0`에 연결합니다.
NetworkManager의 `ptp-pothole-lidar` 설정은 `enp1s0`/`48:21:0B:72:DC:48`,
`porthole-camera` 설정은 `enp2s0`/`48:21:0B:72:DC:47`에 고정되어 있습니다(첫 단말기 기준이며,
다른 단말기에서는 `install.sh`가 그 단말기의 MAC 주소로 만듭니다).
재부팅 후에도 두 장비의 IP 설정이 서로 다른 포트에 적용되도록 유지해야 합니다.

카메라 `frames.jsonl`의 `timestamp_ns`와 LiDAR PCAP header는 모두 UTC입니다.
LiDAR 패킷 내부 tail은 원본 PTP/TAI이며 기록된 UTC offset(현재 37초)을 한 번 빼야 합니다.
이미 UTC인 PCAP header에는 다시 빼지 않습니다. 기존 offset 추정값을 추가 적용하지 마세요.
PTP는 시계를 맞추며, 카메라 노출 시작과 LiDAR 회전 위상 자체를 일치시키지는 않습니다.
또 카메라는 사진을 찍고 약 한 프레임(33 ms) 뒤의 시각을 `timestamp_ns`로 붙입니다. 시계 차이가 아니라
카메라 안의 처리 시간이라 PTP로는 없어지지 않으며, `main_live.py`의 `OFFSET_SEC`(0.033초)로 맞춥니다.
LiDAR PCAP 시각은 센서가 패킷에 적은 PTP 시각 그대로입니다.
시작 직후에는 PTP 잠금과 카메라 RTCP 시각 정보가 확인될 때까지 잠시 저장을 기다립니다.
시작 시 동기화 검사는 5초 연속 통과해야 합니다. 준비 시간의 기준은
운영체제 부팅 시간이 아닌 `collect_data.py` 프로세스 시작 시각입니다.
계속 0으로 남는 경우 `meta/ptp_status.jsonl`의 `qualification`과 `sensors` 오류를 확인합니다.
`meta/ptp_status.jsonl`의 `master_clock.port_skew_ns`는 두 LAN 포트 간 시계 차이입니다.
수집기는 하드웨어 시계를 읽기만 하며, 시스템 시각을 변경할 권한은 없습니다.
`[CONNECTION] LIDAR`는 실제 UDP 수신 여부이고 `[PTP] READY/WAITING`은
시계 검증 상태입니다. PTP 오차 0.1ms 기준을 초과한 데이터를 정상으로 처리하지 않습니다.

## 새 단말기 설치

Ubuntu 22.04 Lubuntu(LXQt), 사용자 `hudaters`, 라이다 포트 `enp1s0`(`/dev/ptp0`)와
카메라 포트 `enp2s0`(`/dev/ptp1`)가 하드웨어 타임스탬프를 지원하는 NUC에서 실행합니다.
라이다, 카메라, 라벨이 `porthole`인 ext4 USB SSD를 연결한 뒤 실행하세요.

```bash
git clone -b live_detection https://github.com/hudaterslab/axsprint_pothole.git ~/Desktop/live_detection
cd ~/Desktop/live_detection
sudo ./install.sh --dry-run   # 바뀔 내용만 확인
sudo ./install.sh
sudo reboot
```

`install.sh`가 하는 일은 다음과 같습니다.

- 패키지: linuxptp·chrony·GStreamer/VAAPI 등 apt 패키지와 numpy·opencv-python(pip)
- 사용자 그룹: `dialout`(GPS), `video`·`render`(하드웨어 디코딩)
- PTP: `/dev/ptp0`·`/dev/ptp1` 읽기 권한(udev), 보정 속도를 제한한 chrony,
  `systemd-timesyncd` 중지, `ptp-pothole-master.service` 설치·시작
  (ptp4l 설정과 phc2sys 실행은 `ptp_service.py`가 합니다)
- PTP 식별자: 이 단말기 라이다 포트와 두 센서의 MAC 주소로 만든 clock identity를
  `/etc/ptp-pothole/device.json`에 기록합니다. `app/ptp.py`는 이 파일이 있으면 이 값을 씁니다.
- 라이다 PTP: clock source를 PTP로 설정하고 1588v2·UDP/IPv4·도메인 0인지 확인
- 수집기: 실행 도우미, sudoers, 라이다 수신 버퍼(sysctl), 수집 터미널 자동 실행, SDDM 자동 로그인
- 업로드 설정: `.env`가 없으면 `.env.example`로 만들고, 업로드 설정 값이 비어 있으면 알려 줍니다
- 저장: `/mnt/ssd` 자동 마운트(`/etc/fstab`)
- 네트워크: 라이다 포트 `192.168.1.100/32`(`192.168.1.201` 경로 포함), 카메라 포트 `192.168.11.2/24`
- 확인: PTP 기준 시계가 두 포트에서 MASTER인지, 두 센서가 이 단말기에 SLAVE로 동기화됐는지,
  라이다가 `Locked`인지, 저장 SSD가 잡히는지

바꾸기 전 파일은 `~/ptp_pothole_archive/install_날짜_시간/`에 백업하며, 다시 실행해도 안전합니다.
설치 후 `.env`에 업로드 설정을 채우세요. API 서버로 보내려면 `PORTHOLE_API_URL`(필요하면 `PORTHOLE_API_TOKEN`)을,
SSH 수신 서버로 보내려면 `PORTHOLE_UPLOAD_HOST`, `_USER`, `_DIR`, `_KEY`를 채웁니다(아래 "서버 전송").
값이 없으면 `main_live.py`는 분석은 계속하고 업로드만 하지 않습니다.
필요하면 `.env`에 다음 선택 항목도 추가할 수 있습니다: `PORTHOLE_UPLOAD_BW_KIB`(업로드 속도 제한 KiB/s, 기본 24576),
`DAMAGE_EXPORT_VEHICLE_TYPE`(결과에 기록할 차량 종류, 기본 현대 팰리세이드).
센서를 연결하지 않은 채 실행했다면 연결한 뒤 다시 실행하세요.
`config.yaml`의 `camera_forward_offset_deg`는 라이다와 카메라의 장착 각도이므로
장착이 첫 단말기와 다르면 직접 맞춰야 합니다.

## 실시간 탐지 자동 실행

`install.sh`는 수집기만 자동 실행으로 등록합니다. `main_live.py`도 부팅 시 자동으로
돌리려면 `install.sh` 다음에 `install_detection.sh`를 따로 실행합니다(sudo 불필요).

```bash
cd ~/Desktop/live_detection
./install_detection.sh --dry-run   # 바뀔 내용만 확인
./install_detection.sh
sudo reboot
```

- 확인: numpy·OpenCV, DEEPX 런타임(`dx_engine`, `/dev/dxrt*`, `dxrt.service`), 보정 파일,
  `.env` 업로드 설정(API 주소, 또는 SSH 키·서버 호스트 키)
- 모델: `best_seg.dxnn`이 없으면 `download_models.sh`로 Hugging Face
  (`HudatersU/road_maintanance`)에서 받아 sha256을 확인합니다. 모델만 따로 받을 때도
  `./download_models.sh`를 실행하면 됩니다. Hugging Face의 모델을 바꾸면 스크립트의 sha256도 바꿔야 합니다.
- 자동 실행: `~/.config/autostart/porthole-analysis-terminal.desktop`을 등록해 자동 로그인 후
  수집 터미널과 별도로 탐지 터미널이 열립니다. 창을 닫거나 Ctrl+C를 누르면 탐지만 종료합니다.
- 자동 재시작: `main_live.py`가 오류로 끝나면 10초 뒤 다시 시작합니다(계속 실패하면 최대 60초 간격).
  NPU 장치가 리셋되면 DEEPX 런타임이 프로그램을 직접 종료하고, DEEPX 서비스가 다시 뜨는 동안에는
  시작할 수 없기 때문입니다. 다시 시작하면 그 시점부터 녹화되는 프레임만 분석합니다.
- 끄기: `./install_detection.sh --disable`

DEEPX 런타임은 이 스크립트가 설치하지 않습니다. NPU 카드와 DEEPX 런타임(dx-runtime)을 먼저 설치하세요.
`main_live.py`는 옵션 없이 실행합니다. GPS 속도가 있으면 주행 중 라이다 점의 어긋남을 보정하고,
GPS 속도가 없으면(실내 등) 보정 없이 깊이를 측정합니다. 정지·저속에서는 보정 없이도 오차가 작습니다.
라이다는 사진의 `timestamp_ns`에서 `OFFSET_SEC`(0.033초)를 뺀, 실제로 찍은 순간에 맞춥니다.
2026-10-02 주행에서 도로 도색이 라이다 반사도와 사진에서 겹치는 시각을 찾아 잰 값으로, 속도(3~10 m/s)와
상관없이 같았습니다. 보정 전에는 9 m/s에서 라이다 점이 사진보다 약 30 cm 뒤쪽(사진 위쪽)에 찍혔습니다.
모델 검출은 신뢰도 0.4 이상만 씁니다(`CONFIDENCE_THRESHOLD`).
크랙은 모델 결과만으로 보고합니다. 포트홀은 라이다로 한 번 더 확인합니다. 모델 영역 안 라이다 점 중
주변 도로(영역에서 10~30 cm 둘레)보다 1 cm 이상 깊은 점이 3개 이상이고, 점들의 중앙값이 주변 도로보다
5 mm 넘게 솟아 있지 않아야 포트홀입니다. 라이다 점 하나는 노이즈(약 3 mm)로 5~9 mm 낮게 찍힐 수 있어서,
평평한 도색이 가장 깊은 점 하나만으로 통과하지 않게 한 것입니다(2026-10-02 주행 기준).
도색(닳은 도색 포함)은 아스팔트보다 반사가 강해서, 라이다가 도색 면과 그 바로 다음 몇 점을 최대 1.5 cm쯤
깊게 잽니다. 그래서 반사도 15 이상인 점과, 같은 라이다 줄에서 그 앞뒤 5점(도색 구역)은 2 cm 이상 깊어야
깊은 점으로 셉니다. 도색 구역 밖의 아스팔트는 1 cm부터 셉니다.
도로 평면 검사의 기준 방향(`RoadPlaneSettings.expected_normal`)은 2026-10-02 주행에서 이 차에 단 센서로
잰 도로 방향입니다. 센서 장착 각도를 바꾸면 다시 재야 합니다.
라이다 스캔 해석은 numpy 배열 계산으로 한 번에 처리하고, PCAP은 파일별 색인으로 필요한 패킷만 읽습니다.
추적은 프레임 순서대로 합니다.

`main_live.py`는 시작한 시점 이후에 녹화된 프레임만 분석합니다. 단말기가 갑자기 꺼져서 분석하지 못한
프레임이 남아 있어도, 다시 켜면 그 프레임은 건너뛰고 새로 들어오는 프레임부터 분석합니다.
단말기에는 분석 결과를 저장하지 않고, 녹화 날짜 폴더마다 검출된 이미지 목록
`/mnt/ssd/porthole_runs/<날짜>/porthole_detections.csv`만 남깁니다(검출이 없는 날은 만들지 않음).
추적 객체를 처음 확인한 프레임마다 한 줄이며, 서버로 보냈는지와 상관없이 적습니다.
날짜는 run 폴더를 따르므로, 자정을 넘긴 run의 검출도 그 run의 날짜 폴더에 적습니다.

- `time`: 촬영 시각(KST)
- `image`: 날짜 폴더 안의 이미지 경로(예: `20260930_2240/frames/00001725.jpg`)
- `objects`: `crack`, `pothole` 또는 `crack+pothole`

## 서버 전송

`main_live.py`는 추적 객체를 처음 확인한 프레임마다 같은 이름의
JPG·JSON·PCAP을 메모리(`/tmp/porthole_upload`)에 만들어 서버로 보내고, 보낸 뒤 바로 지웁니다.
이름은 `frame_<날짜>_<시각>_<밀리초>`로, CSV의 `time`과 같은 촬영 시각(KST)입니다
(예: `frame_20261002_101345_123`). 프레임 번호는 JSON의 `record_id`에 있습니다.
2026-10-02까지 보낸 파일은 `frame_<촬영 시각 ns>_<프레임 번호 8자리>` 이름입니다.
서버에는 `<PORTHOLE_UPLOAD_DIR>/porthole_live_analysis/<run>/certifcate/`에 저장됩니다.
인터넷이 끊기거나 서버가 응답하지 않아 보내지 못하면 그 프레임은 버리고(CSV에는 남음),
30초 동안은 새 검출도 CSV에만 적은 뒤 다시 보내 봅니다. 보내지 못한 프레임을 나중에 다시 보내지는 않으며,
CSV에 남은 검출은 사무실에서 `senddata.py`로 보냅니다(아래).
JSON은 「크랙 포트홀 서버 전송 명세서」(2026-09-28) 형식입니다. `record_id`(`단말기/run 폴더/프레임 번호`),
`categories`, `images`, `annotations`(bbox·segmentation·크기·깊이), `gps`, `lidar.pcap_files`만 담고,
측정값이 없으면 0 대신 null입니다. 크랙은 깊이를 재지 않으므로 `depth.median_cm`이 null입니다.
`gps.speed_mps`(2026-10-02 추가)는 그 순간 차 속도(m/s)로, 단말기가 주행 중 라이다 점을 사진에 맞출 때
쓴 값입니다. 서버의 시각화도 같은 값으로 똑같이 보정할 수 있습니다. GPS 속도가 없으면 null입니다.
JSON은 열어 보기 쉽게 줄바꿈과 2칸 들여쓰기로 저장합니다.
단말기 이름은 `.env`의 `PORTHOLE_TERMINAL_ID`이며, 비어 있으면 호스트 이름을 씁니다.
단말기마다 다른 이름을 써야 서버가 다른 단말기의 결과를 중복으로 버리지 않습니다(첫 단말기는 `hudaters`).

`.env`에 `PORTHOLE_API_URL`이 있으면 프레임마다 HTTP POST 한 번으로 보내고 SSH 설정은 쓰지 않습니다.

- 요청: `multipart/form-data`, 파일 항목 이름 `jpg`·`json`·`pcap`
- 헤더: `X-Record-Id`(JSON의 `record_id`), `PORTHOLE_API_TOKEN`이 있으면 `Authorization: Bearer <토큰>`
- 응답: 2xx는 완료이고, 409는 서버가 이미 받은 것으로 보고 완료로 처리합니다.
  400·413·415·422는 서버가 받지 않는 요청이므로 그 프레임은 버리고 다음 프레임을 보냅니다.
  그 밖의 응답, 연결 실패, 리다이렉트는 보내지 못한 것으로 보고 위와 같이 CSV에만 남깁니다.

`PORTHOLE_API_URL`이 비어 있으면 기존 SSH 수신 서버(`PORTHOLE_UPLOAD_*`)로 보냅니다.
API 서버가 정해지면 위 요청 형식(항목 이름, 헤더, 응답 코드)이 서버와 맞는지 확인하세요.
`.env`를 바꾼 뒤에는 `main_live.py`를 다시 시작해야 적용됩니다.

## 인터넷 없이 주행한 뒤 서버로 보내기

주행 중 인터넷이 없으면 검출은 날짜 폴더의 CSV에만 남습니다. 인터넷이 되는 곳(사무실)에서
분석 창을 닫고 다음을 실행하면 CSV에 적힌 검출을 서버로 보냅니다.

```bash
cd ~/Desktop/live_detection
python3 senddata.py
```

- 모든 날짜 폴더의 CSV를 읽고, 서버에 이미 있는 프레임은 건너뜁니다. 여러 번 실행해도 됩니다.
- 각 프레임을 `main_live.py`와 같은 방법으로 다시 만듭니다. 앞 15장부터 다시 분석해 추적까지 맞추므로
  주행 때와 같은 객체를, 실시간 전송과 같은 JPG·JSON·PCAP으로 같은 서버 위치에 보냅니다.
  만든 파일은 `/tmp`에서 바로 지웁니다.
- NPU를 쓰므로 분석 프로그램(`main_live.py`)이 켜져 있으면 실행되지 않습니다. 수집기는 켜져 있어도 됩니다.
- 녹화 폴더를 지운 날의 검출은 보낼 수 없습니다.

## 서버 시각화 (server/visualize.py)

서버에 올라온 검출 프레임을 사진 위에 검출 영역과 라이다 점(도로 평면 기준 높이 색)으로 그립니다.
단말기가 아니라 업로드 서버에서 실행하는 도구이며, 서버의 데이터 폴더(서버에서는
`<데이터 폴더>/code_server/visualize.py`로 두므로 그 한 단계 위 폴더, 다른 곳에서는 환경변수 `PORTHOLE_ROOT`) 아래
`model_detections/porthole_live_analysis*`를 읽어 `model_detections_lidar_images`에 그립니다.
이미 그린 프레임은 건너뜁니다. 서버에서 NumPy와 Pillow가 있는 파이썬으로 `python3 visualize.py`를 실행합니다.
보정 계산이나 `OFFSET_SEC`를 바꾼 뒤 이미 그린 이미지를 새로 그리려면, 예전 이미지 폴더를 다른 이름으로
옮겨 두고 다시 실행하거나 `OVERWRITE = True`로 한 번 실행합니다.

라이다 해석, 카메라 보정값, 도로 평면과 그 검사, 주행 중 보정은 `main_live.py`와 같습니다.
JSON의 `gps.speed_mps`와 단말기와 같은 카메라 시각 보정(`OFFSET_SEC`)으로 라이다 점을 똑같이 옮기고,
평면 검사에 걸리면 라이다 없이 그립니다. 그래서 `OFFSET_SEC`를 넣기 전(2026-10-02 이전)에 분석된 프레임도
지금 그리면 실제 촬영 순간에 맞춰 그려집니다.
`main_live.py`의 평면·보정 계산, `OFFSET_SEC`, `camera_calib_best_effort.json`을 바꾸면 이 파일도 같이 고친 뒤
서버의 `code_server/visualize.py`에 복사해 주세요.

## 시험 자료 및 변경 전 파일

시험용 수집기·검증 스크립트·검증 결과·이전 백업·미사용 보조 파일은
`/home/hudaters/ptp_pothole_archive/` 아래 정리 날짜 폴더로 옮겼습니다.
실제 systemd 서비스 설정은 `/etc/systemd/system/`에 유지합니다.
SSD의 수집 데이터는 `runs` 바로가기 대상 경로에 그대로 있습니다.

## 코드와 모델 업데이트

배포는 사내 GitLab `hudaters_lab1/pothole`의 `live_detection` 브랜치에 올리면 됩니다.

```
사내 GitLab live_detection  →(GitLab 저장소 미러링, push)→  GitHub hudaterslab/axsprint_pothole
납품 단말기  ←(auto_update.py, 받기만)←  GitHub + Hugging Face
```

- GitLab → GitHub: GitLab의 설정 > 저장소 > 저장소 미러링에 GitHub 주소를 push 방향으로 등록하고
  "보호된 브랜치만 미러링"을 켭니다. 보호된 브랜치는 `live_detection` 하나만 두어야 다른 브랜치가
  공개 GitHub로 나가지 않습니다. GitHub 토큰은 GitLab 미러링 설정에만 넣습니다.
- 단말기: `auto_update.py`가 공개 GitHub에서 코드를 받고, Hugging Face
  `HudatersU/road_maintanance`의 `best_seg.dxnn`을 확인하고, Tailscale 접속을 확인·복구합니다.
  단말기에는 GitLab 주소나 업로드용 인증 정보가 없습니다.

```bash
python3 auto_update.py --check  # 파일을 바꾸지 않고 상태 확인
python3 auto_update.py          # 코드와 모델 업데이트
```

코드는 origin이 위 GitHub 저장소이고 현재 브랜치가 `live_detection`인 Git 작업 폴더에서만
갱신합니다. 커밋되지 않은 수정, 분기된 이력, 기존 파일과 충돌이 있으면 덮어쓰지 않고 중단합니다.
모델은 크기와 SHA-256을 검증한 후 교체합니다. 코드나 모델을 바꿔도 실행 중인 수집·분석
프로세스를 자동으로 재시작하지 않으며, 다음 실행부터 적용됩니다.

운영 단말기는 `hudaters` 사용자의 crontab에 부팅 2분 뒤와 한국 시간(Asia/Seoul) 00시·12시·18시로
등록되어 있습니다. 차량 단말기는 예약 시각에 꺼져 있는 경우가 많아 켤 때마다 한 번 확인합니다.
별도 실행 도우미 없이 `auto_update.py`를 직접 실행합니다.

```cron
@reboot sleep 120 && /usr/bin/python3 /home/hudaters/Desktop/live_detection/auto_update.py >> /home/hudaters/Desktop/live_detection/update.log 2>&1
0 0,12,18 * * * /usr/bin/python3 /home/hudaters/Desktop/live_detection/auto_update.py >> /home/hudaters/Desktop/live_detection/update.log 2>&1
```

꺼져 있던 시간의 예약 작업을 나중에 몰아서 실행하지는 않습니다.
실행 결과(코드·모델을 받았는지, 실패 이유)는 단말기의 `live_detection/update.log`에 시각과 함께 남습니다.
단말기에서 코드를 직접 고치면 그 단말기는 업데이트를 멈춥니다. 고친 내용은 GitLab에 올려 주세요.
Tailscale 복구에는 `systemctl start tailscaled`와 `tailscale up --timeout=30s`의 sudo 권한(NOPASSWD)이
필요합니다.

새 단말기를 복제한 것만으로 이 예약 작업과 sudo 권한이 설치되지는 않습니다.
수동 확인과 업데이트에는 위의 `auto_update.py` 명령을 사용하세요.
