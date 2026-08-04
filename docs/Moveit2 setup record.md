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
                                     │  (같은 ROS_DOMAIN_ID, 같은 LAN — 2026-07-24 기준
                                     │   실제 쓰는 값은 21, 아래 "실행 방법" 참고)
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

### 실행 방법

⚠️ **안전 확인 먼저**: `sync_plan`을 시작하는 순간, 실물이 **그 시점의
시뮬레이션 위치로 보간 없이 바로 이동**함. 노트북 쪽 시뮬레이션이 실물의
현재 자세(보통 look pose)와 다른 곳에 가 있으면 큰 점프가 날 수 있음 —
시작 전 RViz에서 시뮬레이션을 실물과 같은 자세(예: `look_pose` named
state, 위 SRDF 항목 참고)로 맞춰두고 시작할 것. 팔 주변 사람/물건도 확인.

1. 노트북에서 `demo_octomap.launch.py`(또는 `demo.launch.py`)가 이미 실행
   중이어야 함 — `/joint_states`가 발행되고 있어야 `sync_plan`이 받을 게
   있음.
2. 노트북과 RPi가 **같은 `ROS_DOMAIN_ID`**를 쓰는지 확인. RPi
   (`jetcobot_126b`)는 `.bashrc`에 기본값 `21`로 설정돼 있음(비대화형
   SSH 명령으로는 `.bashrc`가 안 읽혀서 값이 안 보일 수 있음 — 대화형
   쉘/실제 로그인 세션 기준). 노트북 쪽 값과 다르면 서로 안 보이므로
   맞춰줄 것.
3. RPi에 SSH 접속 후:
   ```bash
   ssh jetcobot_126b
   source /opt/ros/jazzy/setup.bash
   source ~/smh_ws/install/setup.bash
   export ROS_DOMAIN_ID=21   # 노트북과 동일한 값으로
   ros2 run mycobot_280_moveit2_control sync_plan
   ```
   원격에서 백그라운드로 띄우고 로그를 남기려면(비대화형 SSH 세션에서는
   Python 표준출력이 파일로 리다이렉트될 때 블록 버퍼링돼서 실시간으로
   안 보일 수 있음 — `PYTHONUNBUFFERED=1`을 같이 export하면 실시간으로 보임):
   ```bash
   ssh jetcobot_126b '
     source /opt/ros/jazzy/setup.bash
     source ~/smh_ws/install/setup.bash
     export ROS_DOMAIN_ID=21
     export PYTHONUNBUFFERED=1
     nohup ros2 run mycobot_280_moveit2_control sync_plan > ~/sync_plan_session.log 2>&1 < /dev/null &
     disown
     echo "started with PID $!"
   '
   # 로그 확인
   ssh jetcobot_126b 'tail -f ~/sync_plan_session.log'
   ```
4. 정상 시작되면 로그에 `port:/dev/ttyJETCOBOT, baud:1000000`이 뜨고,
   `/joint_states`가 들어올 때마다 `data_list: [...]`가 찍힘(관절
   순서는 `joint2_to_joint1 ~ joint6output_to_joint6`, 단위는 도(°)).
5. 이후 노트북에서 `coord_to_goal_node`로 `/target_point`를 발행하든,
   RViz Planning 탭에서 마커로 직접 움직이든 **그대로 실물에 동기화됨**.
6. **끝낼 때**: RPi에서 해당 프로세스를 종료
   (`pkill -f "sync_plan$"` 또는 시작할 때 받은 PID로 `kill`).

⚠️ **포트 동시 접근 금지**: `sync_plan`이 `/dev/ttyJETCOBOT`(=
`/dev/ttyUSB0`, 같은 물리 장치)을 물고 있는 동안, `get_angles()` 등으로
실물 상태를 읽는 별도 pymycobot 스크립트를 **동시에 실행하면 안 됨** —
시리얼 포트 동시 접근 충돌로 `sync_plan`이
`SerialException: device reports readiness to read but returned no
data (device disconnected or multiple access on port?)`로 크래시함
(2026-07-24 실측, 실물 손상은 없었음). 상태 확인이 필요하면 `sync_plan`을
잠시 멈추고 확인할 것.

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

### 추가로 발견한 문제: joint6(손목) 회전 방향이 실물과 반대 (2026-07-24)

RViz Planning 탭에서 인터랙티브 마커로 손목을 돌려 Plan & Execute 했을 때,
시뮬레이션에서 돌린 방향과 실물 손목이 실제로 도는 방향이 반대임을 확인함
(다른 5개 관절은 정상 — joint6output_to_joint6 하나만 증상 있음). 근본
URDF의 `joint6output_to_joint6` axis를 뒤집으면 이미 확정된 look pose
수치(`docs/look_pose.md`, `initial_positions.yaml`, SRDF `look_pose`
group_state)가 전부 의미가 바뀌므로, 실물-시뮬 경계인 `sync_plan.py`에서
이 관절 각도만 부호를 반전시켜 보냄(`sync_plan.py`의
`listener_callback()`에 `if joint == 'joint6output_to_joint6':
radians_to_angles = -radians_to_angles` 추가). RPi(`~/smh_ws`)의 실행
중인 파일에 직접 패치 후 재시작해 반영 확인(로그에 마지막 값 부호가
반전된 것 확인). 이 패치는 이전 포트/보드레이트 수정과 마찬가지로
`mycobot_ros2`(elephantrobotics 원본 clone, git으로 안 잡히는 로컬 diff)
안에 있어 별도 커밋 대상 아님 — RPi/노트북 양쪽 로컬 diff로만 존재.

### SRDF에 `look_pose` named state 추가 (2026-07-24)

RViz Planning 탭의 Select Start/Goal State 드롭다운에 `init_pose`만 있고
`look_pose`가 없어서, 마커를 매번 손으로 다시 맞춰야 했음. `firefighter.srdf`에
`initial_positions.yaml`과 동일한 라디안 값으로 `look_pose` group_state를
추가 → 드롭다운에서 바로 선택 후 Plan & Execute로 복귀 가능해짐(SRDF는
move_group 시작 시 1회만 로드되므로 스택 재시작 필요).

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

## 9~11. 카메라 마운트 실측/좌표 조사

문서가 너무 길어져서 별도 파일로 분리함 → **[`camera_mount_investigation.md`](camera_mount_investigation.md)**
(SO-ARM101 스탠드인 실측, mycobot FK 재실측, 카메라 정면 축 발견 등, 챕터
번호 9~11 그대로 유지).

---

## 12. 로드맵 현황 (갱신)

| 단계 | 상태 | RPi+로봇 필요? |
| --- | --- | --- |
| 1. `coord_to_goal_node` | ✅ 완료 | ❌ |
| 2. YOLO+D435 연동 (좌표 계산) | ✅ 완료 | ❌ (카메라만) |
| 3. 핸드-아이 캘리브레이션 | ✅ 완료 (`mycobot_d435_eih.calib`, [handeye_calibration.md](handeye_calibration.md) 13장) — `demo_octomap.launch.py`에 반영 완료 | ✅ 필요 |
| 4. Octomap 연동 | ✅ 완료 (캘리브레이션 기반 joint6->camera_link 실시간 TF, 자기 몸 필터 padding_scale 튜닝(0.7)까지 완료, 벽/손으로 육안 검증함) | ❌ (D435만) |
| 5. 실제 픽업 동작 | 대기 (그리퍼 URDF + 캘리브레이션 + 목표 주변 옥토맵 클리어 로직 필요) | ✅ 필수 |

mycobot 실물 연결 완료. SO-ARM101 병행 조사(`ycheng517/lerobot-ros` +
`Pavankv92/lerobot_ws`)는 여전히 미착수, 여유 생기면 진행.

---

## 13~15. 핸드-아이 캘리브레이션

문서가 너무 길어져서 별도 파일로 분리함 → **[`handeye_calibration.md`](handeye_calibration.md)**
(`easy_handeye2` 설정/사용법, `moveit_calibration` 교차검증, 2026-07-23
재검증 절차까지, 챕터 번호 13~15 그대로 유지).

---

## 다음 세션에서 이어갈 작업 (우선순위, 2026-07-23 갱신)

**이번 세션(2026-07-23)에 완료된 것**: 그리퍼 URDF 자체충돌 SRDF 패치(3번
항목 해결), 목표 플래닝 전 octomap 전체 클리어 로직(2번 항목 해결, 다만
근본 해결은 아니고 완화 — 아래 참고), look pose 확정(`docs/look_pose.md`).

1. **[신규, 최우선] [handeye_calibration.md](handeye_calibration.md) 15장
   핸드-아이 캘리브레이션 재검증** — 실행 여부에 따라
   그리퍼 근접거리 문제가 "노이즈"인지 "실제 근접"인지, 목표 좌표 어긋남이
   캘리브레이션 탓인지 판가름 남
2. `coord_to_goal_node`의 `approach_quat` 고정값(identity) 개선 — 지금은
   목표 자세에 따라 IK가 아예 안 풀리는 경우가 많음(`Unable to sample any
   valid states for goal tree`). 현재 자세 기준 orientation을 쓰거나
   `weight_orientation`을 낮추는 방향 검토(단, pymoveit2가 near-zero
   weight를 1.0으로 강제 보정하는 점 주의 — 완전히 자유롭게 하려면 다른
   방법 필요)
3. 그리퍼 근접거리(카메라 min-range) 노이즈 필터링 — octomap 전체 클리어는
   임시방편이라, 실행 도중 새 프레임이 다시 같은 문제를 재현시킴. 근본
   해결은 depth 근접거리 필터(realsense 쪽 threshold filter 등) 또는
   1번 재검증으로 "노이즈 아님"이 확인되면 접근 방식 자체를 재검토
4. YOLO+D435 → `/target_point` → `coord_to_goal_node` 전체 파이프라인 첫
   end-to-end 테스트 (수동 좌표 테스트 절차는
   [obstacle_avoidance_manual_test.md](obstacle_avoidance_manual_test.md) 참고)
5. (병행 가능) SO-ARM101용 MoveIt2 구성 조사 — `Pavankv92/lerobot_ws` 확인,
   `coord_to_goal_node` 패턴을 SO-101 조인트/링크 이름으로 이식
6. NVIDIA PRIME 오프로드 강제 적용이 RViz 카메라 디스플레이 SIGSEGV를
   해결하는지 검증 ([handeye_calibration.md](handeye_calibration.md) 14장
   트러블슈팅 4번, 아직 미검증 — 지금은
   `rqt_image_view` 우회로 작업 중)
7. **[운영상 주의]** `src/mycobot_ros2/`가 `.gitignore`로 전체 제외돼 있어,
   이번 세션에 그 안에서 고친 SRDF/`initial_positions.yaml`/
   `handeye_calibration.launch.py` 주석 수정이 git에 안 잡힘 — 재클론/재vendoring
   시 유실 위험(automato_ws에서 실제로 한 번 겪은 문제와 동일 종류). 별도
   patch 파일로 보관하거나 gitignore 예외 처리 검토 필요