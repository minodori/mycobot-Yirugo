# AUTOMATO 로봇팔 개발 진행 기록 (Legion-5 MoveIt2 트러블슈팅 이후)

이전 문서 `legion5_moveit2_troubleshooting.md`에서 Legion-5 로컬 MoveIt2 환경
구축까지 완료한 상태를 이어받아, 씬 오브젝트 회피 테스트 완성 → 다음 단계 계획
→ D435 카메라 확보 전략까지 진행한 내용을 정리함.

---

## 1. 씬 오브젝트 충돌 회피 테스트 완성

이전 문서의 "문제 4/5"(discovery race condition, 좌표 겹침)를 해결한 이후,
실제로 장애물을 피해 우회하는 경로 계획까지 성공적으로 확인함.

### 최종 확인된 동작
- `tomato_scene_test.py` 실행 → 콜리전 오브젝트(토마토 3개 구 + 나뭇잎 1개 박스)가
  RViz Scene Objects 탭에 정상 등록됨
- `move_to_pose()` 호출 시 OMPL이 나뭇잎을 피해 옆으로 우회하는 자세로 플래닝 성공
- 스크립트가 반복 실행되며 로봇팔이 여러 목표를 오가며 움직이는 것 확인

### RViz Planning 탭과 스크립트의 관계 (중요한 개념 정리)
- RViz Planning 탭의 **Goal State 드롭다운은 RViz 자체의 대화형 쿼리 상태**만
  표시함. `pymoveit2` 스크립트가 move_group에 직접 보내는 액션 요청과는
  **완전히 별개의 경로**라서, 스크립트가 로봇을 움직여도 이 드롭다운은
  `<current>`로 그대로 남아있는 게 정상.
- 따라서 스크립트로 이미 실행한 동작에 대해 RViz에서 별도로 Plan & Execute를
  누를 필요 없음 (Goal State가 `<current>`인 채로 누르면 이동 거리 0인 빈
  플랜만 생성됨).
- 로봇을 초기 자세로 되돌리고 싶을 때는 RViz Planning 탭 → Goal State →
  `init_pos` 선택 → Plan & Execute.

### 최종 스크립트
`tomato_scene_test.py` (프로젝트에 이미 저장됨) — pymoveit2로 씬 오브젝트 등록 +
목표 pose 플래닝/실행. 다음 단계에서 하드코딩 좌표를 YOLO 실좌표로 교체할
베이스가 됨.

---

## 2. 실제 YOLO+D435 연동 시 MoveIt2 동작 개념 정리

씬 테스트가 하드코딩 좌표로 이뤄졌다면, 실제로는 아래 파이프라인을 거쳐
동일한 `move_to_pose()` 호출로 들어감:

1. **D435 캡처** — RGB + 정합된 depth 프레임 동시 획득
2. **YOLO 추론** — RGB에서 토마토 바운딩박스 검출 → 중심 픽셀 `(u, v)`
3. **2D → 3D 변환** — depth 프레임의 `(u, v)` 거리값 + 카메라 intrinsics →
   카메라 좌표계 기준 3D 점 `(x, y, z)` (`pyrealsense2`의 deproject 함수 계열)
4. **TF2 변환** — 카메라 좌표 → `g_base`(로봇 베이스) 좌표로 변환.
   Eye-in-Hand 마운트라 핸드-아이 캘리브레이션(`easy_handeye2`) 결과가 필요함
5. **PoseStamped 생성** → `moveit2.move_to_pose()`에 그대로 전달

### 지금 테스트와 실전의 차이
- **콜리전 씬**: 지금은 좌표 직접 지정(add_collision_sphere/box). 실전에서는
  D435 포인트클라우드를 옥토맵(Octomap)으로 등록해 실시간 장애물 자동 반영 필요
  (move_group 로그의 `occupancy_map_monitor`가 이 역할)
- **Look-then-move 필수**: D435 최소 뎁스(~10.5cm) vs myCobot 280 도달범위
  (280mm) 충돌 → 멀리서 찍고(look) 좌표 계산 후 접근(move)하는 2단계 전략 필요
  (Eye-in-Hand 유지 시에만 해당, Eye-to-Hand로 전환하면 해소됨)

---

## 3. 다음 단계 로드맵

1. ~~`coord_to_goal_node` 구현~~ — ✅ 완료 (5장 참고)
2. ~~YOLO + D435 실제 연동~~ — ✅ 완료 (좌표 계산까지, 6장 참고). 단,
   TF2 변환(카메라→g_base)이 없어 `coord_to_goal_node`와의 완전한 클로즈드루프는
   3단계 완료 후 가능
3. 핸드-아이 캘리브레이션(`easy_handeye2`) — camera_link → g_base 변환행렬
4. Octomap 연동 — 수동 콜리전 오브젝트 → 실제 포인트클라우드 기반 전환
5. 그리퍼 URDF 추가 검토 — 현재 `mycobot_280_moveit2`는 팔만 있는 구성이라
   수확 동작엔 그리퍼 별도 추가 필요

병행 가능: `harvest_server.py` 재구현(pymycobot 시리얼 연동), 스크립트를
팀 저장소(`ddagi_control`)로 이관하는 시점 판단.

### RPi+실물 로봇 연결이 필요한 시점

| 단계 | RPi+로봇 필요? |
| --- | --- |
| 1. `coord_to_goal_node` 구현 | ❌ 로컬 |
| 2. YOLO+D435 연동 (좌표 계산까지) | ❌ 로컬 (카메라만 있으면 됨) |
| 3. 핸드-아이 캘리브레이션 | ✅ 필요 (팔이 실제 여러 자세를 취해야 계산 가능) |
| 4. Octomap 연동 | ❌ 로컬 (D435 포인트클라우드만 있으면 됨) |
| 5. 실제 픽업 동작 | ✅ 필수 |

핵심 기준: `mock_components/GenericSystem`(FakeSystem)을 실제 pymycobot
시리얼 하드웨어 인터페이스로 교체하는 시점부터 RPi+로봇이 필요함.

### 카메라 마운트 방식 결정
- **1순위**: Eye-in-Hand (그리퍼 부착, 기존 계획 유지)
- **대안(막히면 전환)**: Eye-to-Hand (고정 카메라)
  - 장점: look-then-move 전략 불필요(항상 일정 거리 유지), Octomap 등록이
    안정적(시점이 흔들리지 않음)
  - 캘리브레이션은 이 경우도 여전히 실물 로봇 필요 (마커를 든 팔이 여러
    자세를 취해야 하는 건 동일)

---

## 4. D435 확보 제약과 우회 전략

### 문제 상황
- D435가 학원에만 있고 집(Legion-5)에는 없음. 왕복 시간 부담으로 매번 갈 수
  없는 상황
- 일반 USB 웹캠으로 대체 시: YOLO 검출(2D 바운딩박스)까지는 가능하지만
  **depth가 없어 3D 좌표 변환은 불가능** — 2단계가 절반만 진행됨

### 채택한 전략: 학원에서 rosbag(.bag) 녹화 → 집에서 재생
D435의 `pyrealsense2` 내장 레코딩 기능을 쓰면 컬러+뎁스+intrinsics **원본
그대로** `.bag` 파일에 저장되어, 집에서 재생 시 실시간 스트림과 동일하게
동작함 (화면 캡처 방식과 달리 뎁스 원본 값이 보존됨 — 이 차이가 핵심).

**가능한 것**: YOLO 추론, depth 조회, 3D 좌표 변환까지 파이프라인 코드 전체를
재생 데이터로 검증 가능
**불가능한 것**: 캘리브레이션(camera→robot base 변환)은 실제 로봇+카메라가
동시에 있어야 하므로 재생 데이터로는 못 함. 카메라 각도/거리도 녹화된 그대로
고정이라, 학원에서 찍을 때 다양한 거리/각도/겹침 케이스를 충분히 담아와야 함

### RealSense 드라이버 설치 (apt, 소스 빌드 불필요)
```bash
sudo apt install ros-jazzy-librealsense2*
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-realsense2-camera-msgs
```
파이썬에서 직접 쓸 경우(ROS2 wrapper 없이):
```bash
pip install pyrealsense2 opencv-python numpy --break-system-packages
```
소스 빌드는 apt 버전이 카메라 펌웨어와 안 맞을 때만 고려 (지금은 파이썬
뷰어가 정상 실행됐으므로 해당 없음).

### 지금 단계 판단: pyrealsense2 직접 사용 (ROS2 wrapper는 나중에)
- ROS2 wrapper(`realsense2_camera`): 토픽 발행 → `ros2 bag record`. 노드
  계층이 하나 더 껴서 설정이 많음 (align_depth 옵션 등)
- pyrealsense2 직접: `config.enable_record_to_file()` 한 줄로 바로 `.bag`
  저장. 순수 파이썬이라 단순함
- **결정**: 지금은 pyrealsense2 직접 사용. ROS2 wrapper는 실제 로봇에 붙여서
  YOLO 노드가 ROS2 토픽을 구독하는 실시간 파이프라인으로 갈 때 설치 예정

### 작성한 스크립트: `d435_viewer_record.py`
D435 RGB(좌)+Depth 컬러맵(우) 동시 표시 + `.bag` 녹화 겸용 스크립트.

**주요 설계 포인트**
- `rs.align(rs.stream.color)`로 컬러-뎁스 픽셀 1:1 정렬 — 나중에 YOLO가 컬러
  이미지에서 찾은 바운딩박스 좌표를 그대로 뎁스 프레임에 매핑하기 위한 사전
  작업. 정렬 안 해두면 좌표 매칭 단계에서 별도 처리 필요해서 미리 반영함
- 화면 표시는 시각화용 컬러맵(8비트)이지만, `.bag`에는 원본 16비트 뎁스가
  그대로 저장됨 (사람이 보는 것과 저장되는 데이터가 다름에 유의)

**사용법**
```bash
# 뷰어만
python3 d435_viewer_record.py

# 뷰어 + 녹화 (학원에서 이걸로 실행)
python3 d435_viewer_record.py --out tomato_bed.bag
```

**학원에서 할 일 체크리스트**
1. D435 연결, 위 스크립트로 뷰어 확인
2. `--out tomato_bed.bag`로 녹화 시작
3. 토마토 베드를 여러 각도/거리로 훑으며 촬영 (다양한 케이스 확보)
4. `.bag` 파일 집으로 가져오기 → `python3 d435_viewer_record.py` 대신 추후
   재생/파싱 스크립트로 좌표 변환 파이프라인 검증

---

## 5. `coord_to_goal_node` 구현 (`mycobot_280_pick` 패키지)

`ros2 pkg create`로 스캐폴딩한 `mycobot_280_pick` 패키지에 `coord_to_goal_node`를
구현. 입력은 `/target_point`(`geometry_msgs/PointStamped`) 토픽 — `frame_id`가
`g_base`와 다르면 TF2로 변환(같으면 identity), 그 후 목표보다
`APPROACH_OFFSET_X`(0.05m)만큼 로봇 쪽으로 당긴 접근점으로
`moveit2.move_to_pose()` 호출. `JOINT_NAMES`/`BASE_LINK_NAME`(`g_base`)/
`END_EFFECTOR_NAME`(`joint6_flange`)/`GROUP_NAME`(`arm_group`)은
`tomato_scene_test.py`에서 이미 확인된 값 그대로 사용.

### 겪은 문제: 목표 1회 성공 후 완전 무응답 (재현 가능한 버그)

`/target_point`를 여러 번 연속 발행하면, **처음 성공적으로 실행된 이후로
이후 메시지에 대해 콜백 자체가 더 이상 호출되지 않는** 문제 발생 (경고 로그도
전혀 안 뜸, 노드 재시작해야 복구).

**원인**: `pymoveit2`의 `plan()`(비-move_group_action 경로)과
`wait_until_executed()`가 내부적으로 `rclpy.spin_once(self._node, ...)`를
직접 호출함(`moveit2.py:527-528`, `:790`). `coord_to_goal_node`는 이미 자체
`MultiThreadedExecutor`로 이 노드를 spin 중이었는데, 콜백 안에서 또 다른 스핀
메커니즘(`spin_once`)을 호출하면 두 스핀이 내부적으로 충돌해서 executor가
이후 콜백을 디스패치 못 하게 됨. rclpy에서 잘 알려진 안티패턴 — **노드 하나는
반드시 하나의 executor만 spin해야 함**.

**해결**:
1. `MoveIt2(..., use_move_group_action=True)`로 생성 — `plan()`을 거치지 않는
   완전 콜백 기반 `MoveGroup` 액션 경로를 사용 (spin_once 호출 코드 자체를 안 탐)
2. `wait_until_executed()` 블로킹 호출 제거 → `create_timer()`로
   `moveit2.query_state() == MoveIt2State.IDLE`을 폴링하는 방식으로 완료 감지
   (같은 executor가 처리하므로 spin_once 불필요)

수정 후 성공→성공→성공, 실패(충돌)→성공 등 연속 시나리오 모두 정상 동작 확인함.

---

## 6. YOLO + D435 실카메라 연동 (`yolo_d435_detector_node`)

같은 `mycobot_280_pick` 패키지에 `yolo_d435_detector_node` 추가. D435 RGB+depth
캡처(`pyrealsense2`, `rs.align`) → YOLO(`tomato_4cls_model.pt`)로 `ripe` 클래스
검출 → bbox 중심 픽셀의 depth 조회 → `rs2_deproject_pixel_to_point()`로 카메라
좌표계 3D 점 계산 → `/target_point`(`frame_id: camera_color_optical_frame`)로
1Hz 발행.

### 환경 구축 이슈: numpy/setuptools 버전 충돌
`ultralytics`/`pyrealsense2` 설치 과정에서 `numpy`/`setuptools`가 최신으로
올라가면서 colcon 빌드와 시스템 `matplotlib`이 깨짐 → `setuptools<80`,
`numpy<2`로 고정해서 ROS2 + YOLO 스택 공존 확인.

### 발견한 문제: confidence 1등이 하필 depth 무효
검출된 여러 `ripe` 후보 중 **confidence가 가장 높은 것의 depth가 0(무효)이고,
그보다 낮은 후보들은 depth가 정상**인 경우가 실제로 발생함(반사/모서리 등
표면 특성 때문으로 추정). 기존 로직은 confidence 1등만 보고 depth 없으면
그 사이클을 통째로 버렸음.

**해결**: confidence 내림차순으로 정렬 후, depth가 유효한 것을 찾을 때까지
순서대로 시도하도록 변경 (`ripe_boxes.sort(...)` + for-loop).

---

## 7. 실물 로봇 연결 — 노트북(Legion-5) ↔ RPi 아키텍처

`demo.launch.py`는 `mock_components/GenericSystem`(FakeSystem)이라 계획/실행이
전부 시뮬레이션임. 실물 로봇은 RPi(`jetcobot@192.168.100.13`)에 USB로 연결되어
있고, 아래 구조로 시뮬레이션 움직임을 실물에 그대로 미러링함.

### 노드/토픽 구성도

```
┌─────────────────────────── 노트북 (Legion-5) ───────────────────────────┐
│                                                                         │
│  D435 (USB)                                                            │
│    │                                                                   │
│    ▼                                                                   │
│  yolo_d435_detector_node ──/target_point──▶ coord_to_goal_node          │
│  (mycobot_280_pick)      (camera_color_       (mycobot_280_pick)        │
│                            optical_frame)            │                 │
│                                                       ▼ move_to_pose()  │
│                                              MoveGroup 액션 (move_group)│
│                                                       │                 │
│                                                       ▼                 │
│                              ros2_control_node (FakeSystem, 시뮬레이션) │
│                                                       │                 │
│                                                       ▼                 │
│                                              /joint_states 발행          │
└───────────────────────────────────│─────────────────────────────────────┘
                                     │  (같은 ROS_DOMAIN_ID=10, 같은 LAN)
                                     ▼
┌─────────────────────────── RPi-5 (jetcobot@192.168.100.13) ────────────┐
│                                                                         │
│  sync_plan (mycobot_280_moveit2_control)                                │
│    /joint_states 구독 → 관절각 라디안→도(°) 변환                          │
│    → pymycobot.MyCobot280(port, baud).send_angles(data_list, 35)        │
│                                                       │                 │
│                                                       ▼                 │
│                                        실물 myCobot 280 (시리얼 제어)    │
│                                        /dev/ttyJETCOBOT, baud 1000000    │
└─────────────────────────────────────────────────────────────────────────┘
```

**요점**: 노트북 쪽 시뮬레이션 파이프라인이 계산한 `/joint_states`를, RPi의
`sync_plan`이 구독해서 그대로 실물에 전달하는 구조. `coord_to_goal_node`나
RViz Planning 탭 등 **무엇으로 시뮬레이션을 움직이든 상관없이** `/joint_states`만
보고 실물이 따라 움직임.

### 겪은 문제: `sync_plan`이 실물을 못 움직임 (포트/보드레이트 불일치)

`mycobot_280_moveit2_control`의 `sync_plan.py`는 `ls /dev/ttyUSB*`로 포트를
자동 감지하고 보드레이트를 `115200`으로 고정해뒀는데, 이 로봇의 실제 연결은
`/dev/ttyJETCOBOT`(udev 별칭) + 보드레이트 `1000000`이었음 (사용자가 별도로
쓰던 수동 teleoperation 스크립트에서 확인). 보드레이트 불일치로 시리얼
통신이 깨져서 명령이 조용히 무시됨(에러 없이 무반응).

**해결**: `sync_plan.py`의 포트/보드레이트를 `/dev/ttyJETCOBOT`,
`1000000`으로 수정 후 재빌드 → 시뮬레이션과 실물이 동일하게 움직이는 것 확인.

### 검증 완료
- `/target_point` 발행 → 시뮬레이션(RViz) + 실물 로봇 동시 이동 확인
- RViz Planning 탭 마우스 조작으로도 동일하게 실물 동기화됨 (경로 무관, `/joint_states`만 보므로)

---

## 8. Octomap 연동 (`mycobot_280_moveit2`)

D435 실시간 포인트클라우드를 `occupancy_map_monitor`에 등록해서, 수동
`add_collision_sphere/box`(`tomato_scene_test.py`) 대신 센서가 실제로 관측한
형상을 콜리전 오브젝트로 자동 반영하도록 구성함.

### 설치
```bash
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-camera-msgs
sudo apt install -y ros-jazzy-moveit-ros-perception  # PointCloudOctomapUpdater 등 실제 플러그인 구현체
```
`moveit-ros-occupancy-map-monitor`(프레임워크)만으로는 부족하고, 위 `moveit-ros-perception`이
따로 필요함 — 이게 없으면 `sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater`를
찾을 수 없다는 에러(`Declared types are` 뒤에 아무것도 안 나옴)로 실패함.

### 추가한 파일
- `config/sensors_3d.yaml` — `PointCloudOctomapUpdater` 설정
  (`point_cloud_topic: /camera/camera/depth/color/points`, `max_range: 2.0`,
  `resolution`은 여기 아니라 move_group 파라미터로 별도 지정)
- `launch/move_group.launch.py` 수정 — 라이브러리의 `generate_move_group_launch()`를
  그대로 못 바꾸니 내용을 복제해서 직접 작성. `.sensors_3d()`를 빌더에 추가하고,
  `octomap_frame: g_base`, `octomap_resolution: 0.02`, `max_range: 2.0`을
  move_group 파라미터에 수동으로 끼워넣음 (라이브러리 API에 이 값들을 넣는
  통로가 없어서 직접 구성해야 했음)
- `launch/demo_octomap.launch.py` (신규) — 기존 `demo.launch.py`(시뮬레이션
  스택) + D435(`realsense2_camera`의 `rs_launch.py`) + 카메라→`g_base` static
  transform을 한 번에 띄우는 launch 파일. 카메라 위치는 아직 실측/캘리브레이션
  전이라 `camera_x/y/z/roll/pitch/yaw` launch argument로 조정 가능하게 만듦
  (기본값은 대략적인 추정치)

### 겪은 문제들

**1. `PointCloudOctomapUpdater` 플러그인 로드 실패** — `moveit-ros-perception`
미설치가 원인 (위 설치 섹션 참고).

**2. D435 하드웨어 에러 (`VIDIOC_S_FMT failed`, `Device or resource busy`)** —
근본 원인은 **제 프로세스 정리(`pkill`/`kill`) 패턴이 매번 일부만 잡아서
`realsense2_camera_node`, `static_transform_publisher` 등이 좀비로 계속
누적**됐던 것. 한 번은 동시에 4개의 `realsense2_camera_node`가 같은 D435를
두고 충돌하고 있었음. **교훈**: `ps aux`로 프로세스를 확인할 때 패턴 문자열에
빠짐없이 다 넣기보다, `grep -E "ros2|move_group|realsense|static_transform"`처럼
넓게 훑고 **PID로 직접 kill**하는 게 안전함. 소프트웨어 정리 후에도 D435가
이상하면 USB 재연결(뽑았다 꽂기)로 디바이스 노드(`/dev/video0-7`)를 깨끗하게
재enumerate.

**3. TF 트리 충돌: "Tf has two or more unconnected trees"** — static
transform의 `--child-frame-id`를 `camera_depth_optical_frame`(또는
`camera_color_optical_frame`)으로 직접 지정한 게 원인. `realsense2_camera_node`가
`publish_tf`(기본 true)로 **자체적으로 `camera_link → camera_depth_frame →
camera_depth_optical_frame` 체인을 이미 발행**하고 있어서, 같은 자식
프레임에 제가 또 다른 부모(`g_base`)를 지정하면 두 트리가 충돌함. **해결**:
카메라 트리의 **뿌리**인 `camera_link`에 연결해야 함 (`--child-frame-id
camera_link`) — 그러면 realsense가 발행하는 내부 체인과 자연스럽게 이어짐.
외부 센서를 로봇 TF에 통합할 때 일반적으로 적용되는 원칙.

### 검증 완료
`/get_planning_scene` 서비스로 `PlanningSceneComponents.OCTOMAP`(비트값 32)을
조회해서 실제 옥트리 데이터(`id: 'OcTree'`, 수만~15만 bytes) 확인. 장애물을
옮긴 후 재조회 시 데이터 크기/내용이 바뀌는 것도 확인 — 약 1Hz
(`max_update_rate`)로 실시간에 가깝게 갱신됨.

### 한계 / 추가로 필요한 것
1. **카메라 위치가 부정확함** — 지금 static transform 값(`camera_x/y/z/roll/pitch/yaw`)은
   실측이 아니라 대략적인 추정치라서, RViz에서 Octomap이 실제 물체보다 아래로
   어긋나 보임. 이건 **정식 핸드-아이 캘리브레이션과는 다른 문제** —
   지금은 그냥 고정 카메라 1회성 위치 추정이 부정확한 것뿐이라, 줄자 실측이나
   launch argument 값을 RViz 보면서 트라이얼로 조정하면 개선됨. 로봇이 여러
   자세를 취해야 하는 진짜 캘리브레이션(3단계)은 별도 작업.
2. **Octomap은 의미(semantic) 구분이 없음** — YOLO가 찾은 토마토도 나뭇잎/배경과
   똑같이 "occupied voxel"로 잡힘. 실제 픽업 단계(5단계)에서는 목표 좌표
   주변을 접근 직전에 옥토맵에서 제외(clear)하는 로직이 `coord_to_goal_node`에
   추가로 필요함 — 안 그러면 목표 지점 자체가 장애물이라 플래닝이 거부됨.

---

## 9. SO-ARM101 임시 스탠드인으로 카메라 static transform 실측

mycobot 실물이 아직 없어서(집에는 SO-ARM101만 있음, mycobot은 월요일 학원에서
사용 가능), SO-ARM101 그리퍼 위 D435 위치를 임시로 `g_base` 기준 카메라
위치로 간주하고 `demo_octomap.launch.py`의 `camera_x/y/z/roll/pitch/yaw`
값을 실측해서 넣음.

### 실측 절차
1. SO-ARM 베이스(고정된 지점)를 `g_base`로 간주
2. 그리퍼 위 카메라까지 줄자로 x(앞뒤)/y(좌우)/z(높이) 측정, 카메라가
   보는 방향을 각도로 대략 측정해 roll/pitch/yaw로 변환(도→라디안)
3. `demo_octomap.launch.py`의 `DeclareLaunchArgument` 기본값에 반영

### 검증: 실제 포인트클라우드로 좌표 확인
카메라 정면 ~330mm 지점의 실제 점 하나를 `g_base` 기준으로 변환해서, 손으로
계산한 예상값과 대조 → 거의 정확히 일치 확인 (변환 로직 자체는 정확함).

### 발견한 문제: 원뿔(Free Voxels) 가운데가 비어 보임
`g_base`가 SO-ARM(실물)과 mycobot(RViz 시뮬레이션 모델)이라는 **서로 다른
두 로봇이 공유하는 좌표계**라서, 카메라가 보는 실제 공간이 시뮬레이션
mycobot 모델의 몸체와 우연히 겹치면 `PointCloudOctomapUpdater`의 자기 몸
필터(`shape_mask`)가 실제 관측 포인트를 걸러냄. `camera_pitch`를 올리면
(카메라 시야가 아래로 꺾이며 mycobot 모델 몸체를 덜 스치게 되어) 더 많이
보이지만, **이건 좌표가 더 정확해져서가 아니라 필터링이 덜 되는 것뿐** —
실제 카메라는 수평(pitch=0)이니 pitch=0이 정답이고, 시각화가 덜 되는 건
SO-ARM/mycobot 좌표계 공유로 인한 부작용. 실물 mycobot으로 넘어가면(카메라와
로봇이 같은 개체가 되므로) 해소될 것으로 예상.

### 중요한 개념 정리: static transform은 "로봇이 정지해 있을 때"만 유효

- **YOLO 좌표 계산(look 단계)**: 정해진 자세에서 촬영 → 좌표 계산까지 끝나면
  그 이후엔 결과가 고정값이라 로봇이 움직여도 상관없음 (`coord_to_goal_node`가
  받는 목표 좌표는 이미 계산 완료된 절대좌표). 문서 2장에 정리된
  "look-then-move" 전략과 정확히 맞물림.
- **Octomap(실시간 갱신)**: D435가 계속 스트리밍하며 실시간 갱신되는데,
  static transform은 로봇이 움직이면 더 이상 실제 카메라 위치와 안 맞음 →
  **로봇이 움직이는 동안 Octomap이 잘못된 위치에 쌓이기 시작함.**
- **결론**: 지금 하는 static transform 튜닝은 "로봇이 가만히 있는 상태"까지만
  유효한 임시방편. 로봇이 움직이는 동안에도 Octomap이 정확하려면, 관절
  각도에 따라 실시간으로 갱신되는 TF(=정식 핸드-아이 캘리브레이션, 3단계)가
  필요함 — 이게 3단계가 근본적으로 필요한 이유.

### 내일(mycobot 연결 후) 다시 해야 할 것
1. `camera_x/y/z/roll/pitch/yaw`를 mycobot 기준으로 재실측 (절차는 동일,
   로봇만 SO-ARM→mycobot으로 교체)
2. 원뿔 가운데가 비어 보이던 문제가 해소되는지 확인 (오늘 가설 검증)
3. YOLO+D435 → `/target_point` → `coord_to_goal_node` 전체 파이프라인 첫
   end-to-end 테스트
4. 여유되면 정식 핸드-아이 캘리브레이션(`easy_handeye2`) 착수

---

## 10. mycobot 실물 연결 후 카메라 위치 재실측

mycobot 그리퍼 위에 D435를 장착하고 카메라가 토마토 베드를 보도록 자세를
잡은 뒤, `camera_x/y/z/roll/pitch/yaw`를 다시 잡음.

### 순기구학(FK) 스크립트로 관절 각도 → 카메라 위치 자동 계산 시도

`scripts/compute_camera_transform.py` 작성 — `pymycobot.get_angles()`로 읽은
관절 각도 6개(도 단위)를 `mycobot_280_m5.urdf`의 조인트 원점 값으로 순기구학
계산해서 `g_base -> joint6_flange` 변환을 자동으로 뽑아주는 스크립트.
`init_pose`(전부 0도)로 검증 시 높이 0.411m — 데이터시트 예상치(~0.4m)와 일치.

실제 관측 자세(관절각 `[0.52, -90.96, -0.52, 89.2, 90.08, -0.61]`)를 넣어
계산한 값:
```
camera_x=0.2094, camera_y=-0.0171, camera_z=0.2004
camera_roll=-1.5707, camera_pitch=0.0291, camera_yaw=0.0105
```

### 실측값과 대조 — roll이 90도 어긋남을 발견

같은 자세에서 직접 줄자/눈으로 실측한 값은 `x=0.23m, y=0.06m, z=0.25m,
roll=pitch=yaw=0(거의 수평)`. **위치(x,y,z)는 FK와 몇 cm 이내로 비슷했지만,
회전(특히 roll)이 90도나 차이남.**

**원인**: FK는 `joint6_flange`(URDF 링크)의 좌표축을 그대로 카메라의
광학축이라고 가정했는데, 실제로는 **카메라가 flange에 물리적으로 마운트되며
생기는 고정 회전 오프셋**이 있음 (그리퍼 브라켓이 flange 좌표축과 정확히
평행하게 붙어있지 않음). 이 오프셋을 모르면 FK의 위치값은 대략 믿을 수
있어도 회전값은 신뢰할 수 없음.

**결론**: 지금은 FK 대신 실측값(`x=0.23, y=0.06, z=0.25, rpy=0`)을
`demo_octomap.launch.py` 기본값으로 사용. FK 스크립트는 "위치 어림값을 빨리
얻는 용도"로는 여전히 유용하지만, 회전까지 정확히 자동 계산하려면
flange→카메라 고정 오프셋을 별도로 알아내야 함 (이 오프셋 자체도 결국
캘리브레이션의 일부).

### RViz "Publish Point"로 실제 좌표 직접 확인하는 방법 확립

Occupied/Free Voxels가 화면에서 이상하게(멀고, g_base보다 아래로) 보여서
한참 의심했으나, RViz 툴바에 **Publish Point** 도구를 추가(`+` 버튼)해서
복셀에 마우스를 올리면 상태바에 정확한 좌표가 뜬다는 걸 확인함. 이걸로
직접 대조한 결과:
- 가까운 물체(손, 카메라 앞 10~20cm): `z ≈ -0.03` (g_base 근처, 정상)
- 먼 배경(1m+ 떨어진 벽/바닥 등): `z ≈ -0.21` (g_base보다 뚜렷이 아래)

**결론: 버그가 아니었음.** 책상(로봇 베이스) 뒤로 보이는 바닥/벽 같은 먼
배경이 자연스럽게 낮게 잡히는 것이었고, 관심 대상(가까운 물체)은 애초부터
정확하게 잡히고 있었음. 전체 복셀 뭉치를 한눈에 볼 때 가까운 것과 먼 것이
섞여서 "위치가 이상하다"는 착시가 생긴 것. **앞으로 Octomap 위치를 검증할
때는 Publish Point로 관심 대상 복셀만 개별적으로 찍어서 확인**하는 게
전체 뭉치를 눈으로 보고 판단하는 것보다 훨씬 정확함.

---

## 11. 카메라 정면 방향이 flange의 X축이 아니라 Z축이었음 (핵심 발견)

시뮬레이션 로봇 모델을 실제 관절 각도로 맞춰보니, "카메라가 오른쪽을 보고
있어야 하는데 Octomap 복셀은 아래쪽에 나온다"는 불일치를 발견함.

### 원인

`compute_camera_transform.py`가 처음부터 **"카메라의 정면 = flange의 로컬
X축"**이라고 가정했는데, 실제로는 **"카메라의 정면 = flange의 로컬 Z축"**
이었음. flange의 각 로컬 축이 `g_base`에서 가리키는 방향을 직접 벡터로
확인해서 발견함:
- flange 로컬 **Z축** → g_base 기준 거의 정확히 **+Y**(카메라가 실제로
  보는 방향)
- flange 로컬 **X축** → g_base 기준 거의 **+X**(제가 잘못 가정했던 "정면")

이게 이전부터 계속 나타났던 "FK로 계산한 roll이 실측/눈으로 본 것과 90도
차이난다"는 문제의 근본 원인이었음 — 축을 잘못 짚어서 생긴 체계적 오차였고,
로봇 자세마다 달라지는 "임의의 오차"가 아니었음.

### 왜 눈으로 미리 못 잡았나
"카메라가 안 기울고 안 숙여져 있다(roll/pitch≈0)"는 수평/중력 기준으로
눈으로 판단 가능하지만, "어느 수평 방향(yaw)을 보고 있는지"는 `g_base`라는
추상적 좌표계 기준으로 눈으로 판단할 방법이 없음 — 특히 팔이 크게 꺾인
자세(joint2≈-86도)에서는 더더욱. 그래서 매번 "rpy≈0"이라고 눈으로 판단한
게 yaw 부분에서 계속 틀렸던 것.

### 추가 정정: 카메라는 flange가 아니라 joint6 링크에 마운트됨

위 발견 직후, 실물을 다시 보니 **카메라가 joint6_flange(그리퍼, J6에 따라
회전하는 부분)가 아니라 joint6 링크 자체**(J6 회전과 무관한 고정 부분)에
붙어있다는 걸 확인함 — J6(마지막 관절, 손목 회전)을 돌리면 그리퍼만 돌고
카메라는 그대로임. 그래서 FK 계산에서 J6을 빼고(J1~J5만 사용) joint6까지의
변환으로 다시 계산해야 함. 이때 joint6 로컬 축과 g_base를 대조해보니
"카메라 정면 = joint6의 로컬 X축이 아니라 로컬 **Y축**"으로 확인됨 (flange
기준으로 봤을 때의 Z축과는 또 다름 — joint6→joint6_flange 사이에 고정
회전이 하나 더 있어서 축이 다시 한번 바뀜).

### 해결
`compute_camera_transform.py`를 joint6 기준으로 다시 작성 — FK는 J1~J5만
사용하고, `CAMERA_FROM_JOINT6` 고정 회전 행렬(joint6 기준 Z축으로 90도 회전과
동일: camera_X=joint6_Y, camera_Y=-joint6_X, camera_Z=joint6_Z)로 자동
보정. 이제 관절 각도(J6 포함해서 그대로 입력해도 무방, 내부적으로 무시함)만
넣으면 정확한 roll/pitch/yaw가 나옴. `demo_octomap.launch.py` 기본값도
갱신함(`roll=-0.0815, pitch=-0.001, yaw=1.5675`, 관절각
`[0.52,-86.04,-0.52,91.23,89.29,0.61]` 기준).

**교훈**: 로봇 마운트에 카메라를 달 때, (1) "어느 링크에 물리적으로
고정되어 있는지"(어떤 관절을 돌려도 같이 안 움직이는지 직접 확인)와
(2) "정면 방향이 그 링크의 어느 축인지"를 둘 다 가정하지 말고 실측/FK로
직접 확인해야 함. 둘 다 처음엔 잘못 가정해서 두 번 고쳤음.

### 추가로 발견한 것: 자기 몸 필터가 자세에 따라 가까운 영역을 통째로 지움

팔을 거의 수직으로 세운 자세(init_pose 근처)로 테스트하니, 카메라 정면
가까운 범위(벽까지 포함)가 Octomap에서 전부 사라지는 현상 발견. 원본
포인트클라우드(`/camera/camera/depth/color/points`를 RViz PointCloud2
디스플레이로 직접 확인)에는 벽이 뚜렷이 잡히는데, Octomap(자기 몸 필터를
거친 결과)에는 없음 — 두 디스플레이를 나란히 켜서 직접 대조해 확인.

**원인**: 이 자세에서는 카메라가 g_base 원점(=시뮬레이션 mycobot 몸체가
서있는 자리) 바로 옆에 있어서, 카메라가 보는 가까운 공간 전체가 시뮬레이션
로봇 몸체와 겹침 → `shape_mask` 자기 몸 필터가 실제 관측 데이터를 로봇
자기 자신으로 오인해서 지움. SO-ARM 때 발견한 것과 같은 종류의 문제 —
**임시 static transform이 실물이 아닌 시뮬레이션 로봇과 좌표계를 공유해서
생기는 부작용**이지, 자기 몸 필터 자체의 결함은 아님.

**중요**: 핸드-아이 캘리브레이션이 끝나면 이 문제는 근본적으로 해소됨 —
카메라가 실물 로봇의 실시간 관절 상태에 정확히 묶이면, 자기 몸 필터는
로봇 자기 자신만 정확히 걸러내고 벽 같은 외부 물체는 정상적으로 통과시킴.
지금은 **카메라가 로봇 몸체에서 충분히 떨어져 보이는 자세를 골라서
회피**하는 방식으로 대응함 (마지막 테스트 자세: 관절각
`[0.17,-86.83,89.91,-0.79,89.2,0.7]`).

---

## 12. 로드맵 현황 (갱신)

| 단계 | 상태 | RPi+로봇 필요? |
| --- | --- | --- |
| 1. `coord_to_goal_node` | ✅ 완료 | ❌ |
| 2. YOLO+D435 연동 (좌표 계산) | ✅ 완료 | ❌ (카메라만) |
| 3. 핸드-아이 캘리브레이션 | ✅ 완료 (`mycobot_d435_eih.calib`, 13장) — `demo_octomap.launch.py`에 반영 완료 | ✅ 필요 |
| 4. Octomap 연동 | ✅ 완료 (캘리브레이션 기반 joint6->camera_link 실시간 TF, 자기 몸 필터 padding_scale 튜닝(0.7)까지 완료, 벽/손으로 육안 검증함) | ❌ (D435만) |
| 5. 실제 픽업 동작 | 대기 (그리퍼 URDF + 캘리브레이션 + 목표 주변 옥토맵 클리어 로직 필요) | ✅ 필수 |

mycobot 실물 연결 완료. SO-ARM101 병행 조사(`ycheng517/lerobot-ros` +
`Pavankv92/lerobot_ws`)는 여전히 미착수, 여유 생기면 진행.

---

## 13. 핸드-아이 캘리브레이션 (`easy_handeye2`) 설정

### 왜 필요한가

Octomap용 static transform(11장)은 관절 각도가 바뀔 때마다 FK로 다시
계산해서 launch 인자를 수동으로 갱신해야 하는 임시방편이었음. 정식
핸드-아이 캘리브레이션을 마치면, camera_link -> g_base 변환이 실시간
관절 상태로부터 자동으로 계산·발행되어 로봇이 움직여도 항상 정확함.
카메라가 그리퍼(정확히는 joint6 링크)에 고정된 구조라 **Eye-in-Hand**
방식 사용 (`docs` 8~9장에서부터 1순위로 정한 계획 그대로).

### 설치

- `easy_handeye2`는 ROS 배포판 apt에 없어서 소스 빌드함:
  ```
  cd ~/<workspace>/src
  git clone https://github.com/marcoesposito1988/easy_handeye2.git
  cd ..
  colcon build --packages-select easy_handeye2_msgs easy_handeye2 --symlink-install
  ```
- 마커 검출은 `aruco_opencv` 패키지 사용 (apt로 설치 가능, C++ 노드라
  소스 빌드 불필요):
  ```
  sudo apt install ros-jazzy-aruco-opencv
  ```
- 추가 런타임 의존성 (apt로 설치):
  ```
  sudo apt install python3-transforms3d
  ```
  (`rqt-gui`, `rqt-gui-py`, `python-qt-binding`은 이미 설치되어 있었음)

### 구조 결정: 어디서 뭘 실행하나

캘리브레이션은 로봇의 **실시간 엔코더 관절 각도**가 필요함 (Octomap
작업 때 쓰던 FakeSystem 시뮬레이션 자세로는 안 됨 — 그건 명령값이지
실측값이 아니라서). 그래서 `demo.launch.py` 스택 대신, 아래처럼 역할을
나눔:

- **RPi(jetcobot) 쪽** — 실제 하드웨어에 직접 연결된 것만:
  - `follow_display` 노드 (mycobot_280jn 패키지) — 실제 관절 각도를
    읽어(`get_radians()`) `/joint_states`로 발행:
    ```
    ros2 run mycobot_280jn follow_display --ros-args \
      -p port:=/dev/ttyJETCOBOT -p baud:=1000000
    ```
    실행 전 서보를 릴리즈해서 손으로 자세를 잡을 수 있게 해야 함.
  - D435 카메라 (`realsense2_camera`, image_raw/camera_info만 있으면
    됨 — pointcloud는 캘리브레이션과 무관)
- **로컬 PC 쪽** (`mycobot_280_moveit2/launch/handeye_calibration.launch.py`,
  새로 작성):
  - `robot_state_publisher` (moveit_configs_utils의 `generate_rsp_launch`
    재사용) — 네트워크로 받은 `/joint_states`를 TF로 변환 (같은
    ROS_DOMAIN_ID면 RPi가 어디 있든 topic이 그대로 보임)
  - `aruco_opencv`의 `aruco_tracker_autostart` — 마커 인식, `marker_<id>`
    프레임을 TF로 발행 (카메라가 로컬이든 RPi든 image 토픽만 보이면 됨)
  - `easy_handeye2`의 `calibrate.launch.py` include — `calibration_type=
    eye_in_hand`, `robot_base_frame=g_base`, `robot_effector_frame=joint6`
    (joint6_flange 아님 — 11장에서 실측 확인한 대로 카메라는 joint6
    자체에 고정), `tracking_base_frame=camera_color_optical_frame`,
    `tracking_marker_frame=marker_0`
  - `rviz2` (마커/로봇/TF 육안 확인용)

### 사용법

1. 마커 인쇄 (한 변 크기를 기억해둘 것 — `marker_size` 인자에 그대로 씀):
   ```
   ros2 run aruco_opencv create_marker 0
   ```
   출력에 나오는 "Marker side size"(미터)를 그대로 사용. g_base 기준
   고정된 곳(책상/스탠드)에 평평하게 부착 — 로봇이 움직여도 마커는
   움직이면 안 됨 (eye-in-hand의 전제).
2. RPi: `follow_display` 실행, 카메라 실행 (위 명령 참고)
3. 로컬 PC:
   ```
   ros2 launch mycobot_280_moveit2 handeye_calibration.launch.py \
     marker_size:=<인쇄한 값>
   ```
4. rqt "Calibrate" GUI에서: 서보 릴리즈 상태로 팔을 손으로 잡고 마커가
   항상 화면에 보이게 하면서, 회전 위주로 다양한 자세(각 축 최대한
   크게, 최소 10~15개)를 잡아가며 매번 "Take sample" 클릭 → 15~20개
   모이면 "Compute" → 결과 확인 → "Save"
5. 저장 결과는 `~/.ros2/easy_handeye2/calibrations/<name>.calib`에 저장됨
   (YAML 포맷, 확장자만 `.calib`). 이후 `demo_octomap.launch.py`의 수동
   static_transform_publisher를 `easy_handeye2`의 `publish.launch.py`
   include로 교체 예정.

### 실제 진행하며 겪은 문제 (트러블슈팅)

1. **TF 트리 충돌** — `tracking_base_frame`을 `camera_color_optical_frame`
   으로 잡으면 안 됨. `easy_handeye2`의 `calibrate.launch.py`가
   `robot_effector_frame -> tracking_base_frame`으로 임시 dummy static
   transform을 발행하는데, `camera_color_optical_frame`은 이미
   realsense2_camera 자신이 `camera_link -> camera_color_frame ->
   camera_color_optical_frame` 체인으로 발행 중이라 부모가 두 개
   생겨버림 (Octomap 작업 때 겪은 것과 같은 종류). **`tracking_base_frame`
   은 realsense 자체 트리의 뿌리인 `camera_link`로 잡아야 함** —
   `handeye_calibration.launch.py`에 이미 반영해둠.
2. **rqt Calibrate GUI가 멈춤/무응답** — `Take Sample` 몇 번 하다 보면
   Qt 이벤트 루프가 데드락 걸림 (`QSocketNotifier: Can only be used
   with threads started with QThread` 경고와 관련된 것으로 추정, CPU
   사용량도 0%로 떨어져서 진짜 데드락 확인함). GUI를 강제 종료하면 그때
   까지 모은 샘플이 다 날아감. **해결책**: GUI 대신 `ros2 service call`
   로 직접 진행 —
   ```
   ros2 service call /easy_handeye2/calibration/take_sample easy_handeye2_msgs/srv/TakeSample "{}"
   ros2 service call /easy_handeye2/calibration/get_sample_list easy_handeye2_msgs/srv/TakeSample "{}"
   ros2 service call /easy_handeye2/calibration/compute_calibration easy_handeye2_msgs/srv/ComputeCalibration "{}"
   ros2 service call /easy_handeye2/calibration/save_calibration easy_handeye2_msgs/srv/SaveCalibration "{}"
   ros2 service call /easy_handeye2/calibration/remove_sample easy_handeye2_msgs/srv/RemoveSample "{sample_index: 0}"
   ```
   (launch 파일 자체는 계속 켜둔 채로, 다른 터미널에서 이 명령들만
   호출하면 됨. `handeye_server`가 살아있기만 하면 GUI 없이도 동일하게
   동작함.)
3. **`cv2.calibrateHandEye` 없음 (`AttributeError`)** — `~/.local`에 pip로
   설치된 `opencv-python`(5.0.0, YOLO/ultralytics용으로 이전에 설치한
   것)이 시스템 `python3-opencv`(4.6.0, apt)보다 import 우선순위가 높은데,
   5.0.0에는 `calibrateHandEye`가 없음(API 개편으로 빠지거나 이름이
   바뀐 것으로 추정). **해결책**: launch 실행 시 `PYTHONNOUSERSITE=1`을
   붙여서 시스템 opencv(4.6.0)를 쓰도록 강제:
   ```
   PYTHONNOUSERSITE=1 ros2 launch mycobot_280_moveit2 handeye_calibration.launch.py marker_size:=0.05
   ```
   이 환경변수는 이 실행에만 적용되므로 다른 터미널의 YOLO 작업에는
   영향 없음.
4. **launch 프로세스 kill 시 orphan 발생 주의** — 멈춘 rqt 프로세스를
   `kill`할 때 PID를 잘못 잡으면(`ros2 launch` 부모 프로세스 등) 의도치
   않게 `handeye_server`/`dummy_publisher`까지 같이 죽고 카메라/aruco만
   orphan으로 남는 상황이 있었음. `ps aux`로 정확한 PID를 확인하고 최소
   범위로만 kill할 것.

### 캘리브레이션 전후 비교

**이전 (수동 FK, `compute_camera_transform.py`)**: g_base 기준으로 매
자세마다 재계산 필요, 카메라 원점=joint6 원점(오프셋 0)이고 회전은
joint6 기준 Z축 정확히 90도라고 가정.

**이후 (`easy_handeye2`, 결과 이름 `mycobot_d435_eih`)**: `joint6 ->
camera_link`가 로봇 자세와 무관한 고정값 하나로 나옴 (실시간 TF에
얹히므로 관절이 움직여도 항상 유효).

```yaml
translation: {x: -0.0267, y: -0.0035, z: 0.0573}   # m
rotation:    {x: -0.0079, y: -0.0157, z: 0.6198, w: 0.7846}
```

이전 가정과 비교하면:
- **위치**: 오프셋을 0으로 가정했었는데 실제로는 약 **6.3cm** 떨어져
  있었음 (카메라 렌즈와 joint6 원점 사이 실제 물리적 거리)
- **회전**: 정확히 90도라고 가정했었는데 실제로는 약 **13.5도** 차이가
  있었음

이 어긋남이 그동안 Octomap에서 물체 위치가 실제보다 살짝 아래/옆으로
보였던 원인 중 하나로 추정됨 (11장에서 관찰한 내용과 방향이 일치).

**검증 방법**:
1. (정성적, 우선 진행) `demo_octomap.launch.py`를 캘리브레이션 결과로
   교체한 뒤, 팔을 서로 다른 2~3개 자세로 바꿔가며 카메라 앞 물체 위치가
   Octomap에서 재계산 없이도 일관되게 맞는지 확인 (수동 FK 방식은 자세
   바뀔 때마다 재계산해도 여전히 오차가 있었음 — 이게 핵심 차이).
2. (정량적, 선택) `easy_handeye2`의 `evaluate.launch.py` +
   `rqt_evaluator.py` — 캘리브레이션에 안 쓴 새 자세에서 마커를 관측해
   예측 위치와 실제 위치 사이 오차를 실시간으로 보여줌. rqt 기반이라
   위 트러블슈팅 2번과 같은 프리즈가 재발할 수 있음.

### 상태

✅ 완료 — `mycobot_d435_eih.calib` 저장됨. `demo_octomap.launch.py`도
`easy_handeye2`의 `publish.launch.py`로 교체 완료, 벽/손을 놓고 실제로
Octomap Occupied Voxels가 정확히 맞는 것까지 육안으로 검증함 (여러 번의
`controller_manager` 서비스 기동 지연 문제를 겪었으나 이 launch 파일
자체의 문제는 아니었고, 정상 실행 시엔 몇 초 안에 다 뜸).

**추가 튜닝**: `sensors_3d.yaml`의 `padding_scale`을 기본값 1.0에서
**0.7로 낮춤** — 1.0에서는 자기 몸 필터가 카메라 근처의 실제 물체(벽 등)
까지 과도하게 걸러내는 경우가 있었음. 0.7로 낮추니 벽은 정확히 잡히고,
손처럼 카메라에 더 가까운 물체를 앞에 두면 그 뒤 벽 부분에 정확히
구멍(가려짐)이 뚫리는 것까지 확인함.

---

## 14. MoveIt Hand-Eye Calibration (`moveit_calibration`) 설정

### easy_handeye2와의 차이

`easy_handeye2`(13장)와는 **완전히 별도인 워크스페이스/도구**. 둘 다
Eye-in-Hand 캘리브레이션을 하지만 타겟과 워크스페이스가 다르다.

| | easy_handeye2 (13장) | moveit_calibration (이 장) |
|---|---|---|
| 워크스페이스 | `~/Projects/mycobot` | `~/Projects/moveit_calibration` (완전 별도) |
| 타겟 | ArUco 단일 마커(`marker_0`) + `aruco_opencv` | ChArUco 보드(체스판+ArUco 결합) |
| UI | rqt 플러그인 | RViz 플러그인(도킹 패널) |
| 결과 저장 | `~/.ros2/easy_handeye2/calibrations/<name>.calib` | 수동으로 원하는 경로에 yaml 저장 |

### 설치

```bash
mkdir -p ~/Projects/moveit_calibration/src
cd ~/Projects/moveit_calibration
git clone https://github.com/ros-planning/moveit_calibration.git -b ros2 src/moveit_calibration
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 실행 방법

**1. RPi(jetcobot) 쪽 — 서보 릴리즈** (손으로 자세를 잡아야 하므로)

```bash
python3 -c "from pymycobot import MyCobot280; MyCobot280('/dev/ttyJETCOBOT', 1000000).release_all_servos()"
```

**2. 로컬 PC — 두 워크스페이스 모두 source**

```bash
source /opt/ros/jazzy/setup.bash
source ~/Projects/mycobot/install/setup.bash
source ~/Projects/moveit_calibration/install/setup.bash
```

**3. 카메라 포함된 기존 launch로 기동** (`demo_octomap.launch.py`를
씀 — move_group/카메라/rviz2가 다 포함돼 있어 별도 launch 파일을
새로 만들 필요 없음)

```bash
ros2 launch mycobot_280_moveit2 demo_octomap.launch.py
```

**4. RViz에서 패널 추가**

`Panels → Add New Panel → moveit_calibration_gui` 아래
`HandEyeCalibration` 선택.

⚠️ **주의**: RViz 안에 카메라 영상을 보려고 `Camera`/`Image` 디스플레이를
따로 추가하면 GPU 렌더링(OGRE) 문제로 죽을 수 있음 (아래 트러블슈팅 참고).
카메라 화면 확인은 별도 터미널에서:
```bash
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
```

### Context 탭 설정값

| 필드 | 값 | 근거 |
|---|---|---|
| Sensor configuration | `Eye-in-Hand` | |
| Planning Group | `arm_group` | SRDF에 정의된 유일한 그룹 |
| Sensor frame | `camera_color_optical_frame` | 이미지/camera_info의 `header.frame_id` |
| Object frame | `handeye_target` | moveit_calibration이 타겟 검출 시 하드코딩된 고정 이름으로 TF 발행 (Target 탭에서 보드가 인식돼야 드롭다운에 나타남) |
| End-effector frame | `joint6` | ⚠️ `joint6_flange` 아님! 카메라는 joint6_flange(그리퍼, J6 회전 영향)가 아니라 joint6 링크 자체에 고정됨(11장 실측). 이전 시도(`moveit_hand-eye_cali.yaml`)는 `joint6_flange`로 잘못 저장했었음 |
| Robot base frame | `g_base` | |

**왜 `Sensor frame`엔 optical frame(자식 프레임)을 써도 되는가** — easy_handeye2의
`tracking_base_frame`(13장)과 헷갈리기 쉬운 부분. 둘 다 "카메라 기준
프레임"이라는 개념은 같지만, TF 트리에서 하는 **역할이 다르다**:

- **easy_handeye2 `tracking_base_frame`**: `calibrate.launch.py`의 더미
  퍼블리셔가 `robot_effector_frame → tracking_base_frame`으로 **새
  부모를 강제로 붙인다.** `camera_color_optical_frame`은 이미
  realsense가 부모(`camera_color_frame`)를 정해놨으므로, 부모가
  2개가 되어 TF 트리가 깨진다 → 뿌리 프레임(`camera_link`)을 써야 함.
- **moveit_calibration `Sensor frame`**: 타겟 인식에 성공하면
  `Sensor frame → handeye_target`(완전히 새로 생기는 자식 프레임)을
  발행한다(`handeye_target_widget.cpp:421`,
  `handeye_target_base.h:172`의 `child_frame_id = "handeye_target"`).
  `Sensor frame`은 **부모 역할**로만 쓰이므로, 자기 자신이 기존에 다른
  부모(`camera_color_frame`)를 갖고 있어도 상관없다 — 한 프레임이
  부모를 여러 개 가지면 안 되지만 자식은 여러 개 가져도 무방하기 때문.

즉 "새 TF 엣지에서 그 프레임이 부모냐 자식이냐"가 기준이지, 프레임
이름(`_optical_frame`인지 아닌지) 자체는 기준이 아니다.

### Target 탭 설정값

- **Target Type**: ChArUco board
- **Camera Image Topic**: `/camera/camera/color/image_raw`
  (camera_info는 `image_transport::subscribeCamera`가 같은 네임스페이스에서
  자동으로 같이 구독하므로 별도 지정 불필요)
- 보드 파라미터(행/열 개수, square size, marker size, dictionary)는
  실제 인쇄한 보드 치수와 정확히 일치해야 함 — 하나라도 안 맞으면
  타겟 인식 자체가 안 되고 `Object frame`이 `world`로만 나옴(=검출 실패
  신호)

### 트러블슈팅

**1. Object frame이 "world"로만 나옴** — 타겟 검출 실패 신호.
- 보드가 카메라 프레임 안에 온전히 들어와 있는지
- Target 탭 보드 파라미터가 실제 인쇄 치수와 일치하는지
- 거리/조명

**2. 모터가 안 풀림(손으로 못 움직임)** — 위 "1. RPi 쪽" 서보 릴리즈
명령 실행 필요.

**3. `demo_octomap.launch.py`는 FakeSystem(시뮬레이션)이라는 점 주의**
— 손으로 실물 팔을 움직여도 MoveIt이 읽는 "현재 자세"는 FakeSystem의
마지막 명령값일 뿐, 실물의 실제 관절각이 아님. easy_handeye2 때
`follow_display`로 실측 인코더값을 흘려보내야 했던 것과 같은 이유.
**증상**: 서보만 릴리즈(`release_all_servos()`)하고 손으로 움직이면
`/joint_states`가 안 바뀌어서, 아무리 다른 자세를 잡아도 매번 "End-effector
orientation is too similar to a prior sample" 에러가 남 (`ros2 topic
echo /joint_states`로 직접 확인함 — 값이 고정돼 있었음).
**해결**: RPi에서 `release_all_servos()` 대신 `follow_display`를 직접
실행 — 릴리즈 + 실측 각도 발행을 같이 함:
```bash
ros2 run mycobot_280jn follow_display --ros-args \
  -p port:=/dev/ttyJETCOBOT -p baud:=1000000
```
FakeSystem의 `joint_state_broadcaster`와 `/joint_states`에 발행자가
2개(FakeSystem + follow_display)가 되는 셈이지만, 실제로 돌려보니 샘플
22개 정상 기록·계산까지 문제없이 진행됨(아래 "캘리브레이션 결과" 참고).

**4. 카메라 디스플레이 추가 시 RViz가 SIGSEGV로 죽음**
(`exit code -11`, 로그에 `failed to create drawable` 반복) — OGRE
렌더링 문제. `glxinfo | grep "OpenGL renderer"`로 확인해보니 NVIDIA
RTX 5060(dGPU)이 아니라 **AMD 내장 그래픽(Radeon 780M)으로 렌더링되고
있었음** — 하이브리드 그래픽 노트북에서 PRIME 오프로드 미설정.
  - **즉시 우회**: RViz엔 카메라 디스플레이를 추가하지 말고
    `rqt_image_view`로 별도 확인 (위 3번 참고)
  - **근본 해결 시도**: NVIDIA로 강제 전환 후 재시도
    ```bash
    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
      ros2 launch mycobot_280_moveit2 demo_octomap.launch.py
    ```
    (아직 이 방법으로 재현/해결 검증은 안 됨)

### 시도 이력

**1차 시도 (`moveit_hand-eye_cali.yaml`) — 폐기**: `End-effector frame`을
`joint6_flange`로 잘못 잡고 진행한 결과라 신뢰할 수 없음.
```yaml
# EYE-IN-HAND: joint6_flange -> camera_color_optical_frame
translation: {x: -0.0753477, y: -0.225542, z: -0.122314}
rotation: {x: -0.0109544, y: 0.106479, z: -0.0432289, w: 0.993314}
```
MoveIt Calibration의 Planning Group 드롭다운은 SRDF 그룹만 나열하는데,
`arm_group`이 `joint6output_to_joint6`까지 포함해서 tip이 무조건
`joint6_flange`로 잡힘 — 중간 링크(`joint6`)를 직접 지정하는 필드가
UI에 없어서 생긴 구조적 문제.

**2차 시도 (`moveit_hand-eye_cali_2.yaml`) — 첫 번째 저장, 폐기**:
Context 탭의 `Sensor configuration`을 실수로 `Eye-to-Hand`로 선택한 채
진행 → `joint1 -> camera_color_optical_frame`(카메라가 고정되고
마커가 손끝에서 움직인다는, 실제와 반대되는 전제)으로 계산되어 무효.

**2차 시도 재작업 (`moveit_hand-eye_cali_2.yaml`, 덮어씀) — ✅ 성공**:
`Sensor configuration = Eye-in-Hand`, `Robot base frame = g_base`,
`End-effector frame = joint6`로 정정하고 ChArUco 보드 샘플 22개로 재계산.
```yaml
# EYE-IN-HAND: joint6 -> camera_color_optical_frame
translation: {x: -0.134963, y: -0.233021, z: 0.0967767}
rotation: {x: -0.687523, y: -0.0403723, z: -0.110947, w: 0.716501}
```
YAML 문법/쿼터니언 정규화(노름² ≈ 1.0000007) 모두 정상 확인.

### easy_handeye2와 교차검증 — 일치 확인 ✅

`joint6`과 `camera_color_optical_frame` 사이엔 직접 발행되는 TF가 없고,
easy_handeye2 결과(`joint6 -> camera_link`)와 realsense 자체 발행
체인(`camera_link -> camera_color_frame -> camera_color_optical_frame`)이
TF에서 자동 합성된다. `demo_octomap.launch.py`(camera + robot_state_publisher
+ easy_handeye2 `publish.launch.py` 포함) 켜둔 상태에서 직접 조회:
```bash
ros2 run tf2_ros tf2_echo joint6 camera_color_optical_frame
```
결과:
```
Translation: [-0.135, -0.233, 0.097]
Rotation (xyzw): [-0.688, -0.040, -0.111, 0.717]
```

| | tf2_echo (easy_handeye2 합성) | moveit_calibration (`_2.yaml`) | 차이 |
|---|---|---|---|
| X | -0.135 | -0.134963 | ~0.04mm |
| Y | -0.233 | -0.233021 | ~0.02mm |
| Z | 0.097 | 0.0967767 | ~0.2mm |
| qx | -0.688 | -0.687523 | 0.0005 |
| qy | -0.040 | -0.0403723 | 0.0004 |
| qz | -0.111 | -0.110947 | 0.0001 |
| qw | 0.717 | 0.716501 | 0.0005 |

차이는 `tf2_echo` 표시 반올림(소수점 3자리) 수준 — 이동 0.2mm 이하,
회전 오차 1도 미만. **ArUco 단일 마커(easy_handeye2)와 ChArUco 보드
(moveit_calibration)라는 서로 다른 방법·다른 세션·다른 로봇 위치에서
독립적으로 계산했는데 사실상 같은 값**이 나와, `joint6 ->
camera_color_optical_frame` 값의 신뢰도가 상호 검증됨. (g_base의 물리적
위치가 두 세션 간 달라도 무방한 이유: g_base는 joint6-camera 관계보다
상류에 있는 별개의 기준점이라 이 비교와 무관.)

### 상태

✅ 완료 — moveit_calibration으로도 `joint6 -> camera_color_optical_frame`
계산 성공, easy_handeye2 결과와 교차검증까지 마침. 두 도구 다 일관된
결과를 주는 것을 확인했으므로 앞으로는 easy_handeye2 쪽(`mycobot_d435_eih`,
`publish.launch.py`로 실시간 TF 유지)을 기본으로 계속 사용.

---

## 다음 세션에서 이어갈 작업 (우선순위)

1. YOLO+D435 → `/target_point` → `coord_to_goal_node` 전체 파이프라인 첫
   end-to-end 테스트 (mycobot 실물로, 이제 Octomap 충돌회피까지 살아있는
   상태로 테스트 가능)
2. 목표 주변 Octomap 클리어 로직 설계 — `coord_to_goal_node`가 목표로
   접근하기 전 그 주변 voxel을 제외하도록
3. 그리퍼 URDF 추가 검토
4. (병행 가능) SO-ARM101용 MoveIt2 구성 조사 — `Pavankv92/lerobot_ws` 확인,
   `coord_to_goal_node` 패턴을 SO-101 조인트/링크 이름으로 이식
5. NVIDIA PRIME 오프로드 강제 적용이 RViz 카메라 디스플레이 SIGSEGV를
   해결하는지 검증 (14장 트러블슈팅 4번, 아직 미검증 — 지금은
   `rqt_image_view` 우회로 작업 중)