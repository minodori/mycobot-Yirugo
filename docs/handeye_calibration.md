# 핸드-아이 캘리브레이션 (13~15장)

`Moveit2 setup record.md`가 너무 길어져서 분리한 문서. 원래 챕터 번호(13~15)는
그대로 유지 — 다른 문서/세션 기록에서 "13장"/"14장"/"15장"으로 참조하는 곳이
있으면 이 파일을 보면 됨.

---

## 13. 핸드-아이 캘리브레이션 (`easy_handeye2`) 설정

### 왜 필요한가

Octomap용 static transform(`camera_mount_investigation.md` 11장)은 관절 각도가
바뀔 때마다 FK로 다시 계산해서 launch 인자를 수동으로 갱신해야 하는
임시방편이었음. 정식 핸드-아이 캘리브레이션을 마치면, camera_link -> g_base
변환이 실시간 관절 상태로부터 자동으로 계산·발행되어 로봇이 움직여도 항상
정확함. 카메라가 그리퍼(정확히는 joint6 링크)에 고정된 구조라 **Eye-in-Hand**
방식 사용.

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
  - `follow_display` 노드 (**`mycobot_280pi` 패키지** — SBC 실체가 Jetson
    Nano가 아니라 Raspberry Pi 5로 확정됨에 따라 `mycobot_280jn`에서
    변경, 2026-07-23) — 실제 관절 각도를 읽어(`get_radians()`)
    `/joint_states`로 발행:
    ```
    ros2 run mycobot_280pi follow_display --ros-args \
      -p port:=/dev/ttyUSB0 -p baud:=1000000
    ```
    (포트도 `/dev/ttyJETCOBOT`가 아니라 실측 확인된 `/dev/ttyUSB0` —
    2026-07-23 확인. baud 1000000도 실측 확인값, 115200 아님.)
    이 노드가 시작 시 자동으로 `release_all_servos()`를 호출해서 서보를
    릴리즈하므로 별도 릴리즈 명령 불필요 — 실행하자마자 팔이 손으로
    움직일 수 있는 상태가 됨.
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
    (joint6_flange 아님 — 카메라는 joint6 자체에 고정), `tracking_base_frame=
    camera_color_optical_frame`, `tracking_marker_frame=marker_0`
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
보였던 원인 중 하나로 추정됨.

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
| End-effector frame | `joint6` | ⚠️ `joint6_flange` 아님! 카메라는 joint6_flange(그리퍼, J6 회전 영향)가 아니라 joint6 링크 자체에 고정됨. 이전 시도(`moveit_hand-eye_cali.yaml`)는 `joint6_flange`로 잘못 저장했었음 |
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

## 15. 핸드-아이 캘리브레이션 재검증 (2026-07-23)

### 왜 다시 하나

`mycobot_280_pick`의 `coord_to_goal_node`로 장애물 회피 테스트 중, RViz에
찍은 목표 좌표(장애물 voxel 오른쪽)로 팔을 보냈는데 실제로는 팔이 장애물
쪽으로 향하는 현상을 관찰함. 원인 후보 중 하나가 handeye 캘리브레이션
오차 — Octomap에 그려지는 voxel(장애물)은 `depth 포인트(카메라 프레임) →
camera_link → joint6(handeye 고정 변환) → ... → g_base`를 거쳐 배치되는데,
이 변환에 오차가 있으면 화면상 voxel 위치가 g_base 기준 **실제** 장애물
위치와 어긋난다. 반면 팔의 실제 움직임은 관절 엔코더+URDF 기구학만으로
결정되고 캘리브레이션과 무관하게 정확하므로, 화면상 "장애물 오른쪽"이
실제 물리 공간에서는 장애물 쪽일 수 있음.

**단, 참고**: 14장에서 이미 easy_handeye2(ArUco)와 moveit_calibration
(ChArUco) 두 독립적인 방법으로 `joint6 -> camera_color_optical_frame`을
교차검증해서 0.2mm/1도 미만 오차로 일치했던 이력이 있음 — 즉 계산 자체는
당시 매우 정확했다. 그 이후로 바뀔 수 있는 것은 **카메라 마운트의 물리적
위치**(이 세션 중 D435를 여러 번 재연결/포트 교체함) 정도이지, 그리퍼
URDF 추가는 `joint6` 자체의 정의를 안 바꾸므로 이 변환과 무관해야 정상.
그래서 재검증 전에 아래 "빠른 확인"부터 해보고, 실제로 어긋난 게
확인되면 전체 재캘리브레이션(15-2)으로 넘어가는 순서를 권장.

**추가 확인(2026-07-23)**: 실제로 지금 로드되는 `mycobot_d435_eih.calib`
파일(수정 시각 2026-07-20 20:48)의 값이 위 13장에 문서화된 교차검증
값과 다름 — 위치 1.3~2cm, 회전(z 성분)도 눈에 띄게 차이남:

| | 문서(13장, 교차검증됨) | 실제 파일(2026-07-23 기준 사용 중) |
|---|---|---|
| translation (m) | (-0.0267, -0.0035, 0.0573) | (-0.0141, 0.0090, 0.0786) |
| rotation (xyzw) | (-0.0079, -0.0157, 0.6198, 0.7846) | (-0.0132, 0.0253, 0.6888, 0.7244) |

즉 13/14장에서 "0.2mm/1도 미만으로 교차검증 완료"라고 기록된 캘리브레이션과
**지금 실제로 로드되는 파일이 다른 버전** — 언제/왜 덮어써졌는지 기록이
없음. 이게 오늘 관찰한 좌표 어긋남의 유력한 원인일 수 있음.

### 15-1. 빠른 확인 (재캘리브레이션 없이 오차 유무만 확인)

실물 팔이 있는 상태에서, 줄자로 잰 known position(예: 책상 모서리, 팔에서
잰 거리)이 RViz에서 클릭한 좌표/Octomap voxel 위치와 맞는지 눈대중 대조.
또는 정량적으로:

```bash
# demo_octomap.launch.py 등 처럼 easy_handeye2 publish.launch.py가 이미 떠 있는 상태에서
ros2 launch easy_handeye2 evaluate.launch.py \
  name:=mycobot_d435_eih \
  calibration_type:=eye_in_hand \
  robot_base_frame:=g_base \
  robot_effector_frame:=joint6 \
  tracking_base_frame:=camera_link \
  tracking_marker_frame:=marker_0
```

마커를 다시 카메라 앞에 두고 `rqt_evaluator` 창에서 실시간 오차(reprojection
error)를 확인. 오차가 원래 캘리브레이션 때와 비슷한 수준(수 mm/1도 내외)이면
캘리브레이션은 문제가 아니고, 그리퍼 형상/근접거리 노이즈/`coord_to_goal_node`의
좌표 처리 쪽을 먼저 의심할 것. 오차가 눈에 띄게 커졌으면 15-2로.

### 15-2. 전체 재캘리브레이션 절차

**0. 사전 정리** — 기존에 떠 있는 `demo_octomap.launch.py` 스택(카메라,
`coord_to_goal_node` 등)을 전부 종료. D435와 실물 팔 시리얼 포트를
동시에 여러 프로세스가 물면 안 됨(장치당 프로세스 1개).

```bash
# 로컬 PC
pkill -9 -f "move_group|rviz2|realsense2_camera_node|ros2_control_node|robot_state_publisher|handeye_publisher|static_transform_publisher|coord_to_goal_node"
```

**1. RPi(jetcobot_126b, `ssh jetcobot_126b`)에서 실물 관절 각도 발행**
(SBC는 Raspberry Pi 5 — 패키지는 `mycobot_280jn`이 아니라 `mycobot_280pi`,
포트는 `/dev/ttyUSB0`, baud 1000000. `smh_ws`에 이미 빌드돼 있음):

```bash
ssh jetcobot_126b
source ~/smh_ws/install/setup.bash
export ROS_DOMAIN_ID=21   # ~/.bashrc의 jetcobot2 alias와 동일한 값
ros2 run mycobot_280pi follow_display --ros-args -p port:=/dev/ttyUSB0 -p baud:=1000000
```

실행하자마자 서보가 자동으로 릴리즈됨(코드 내부에서 `release_all_servos()`
호출) — 팔이 손으로 움직이는 상태가 됨. 이 터미널은 계속 열어둘 것.

**2. ArUco 마커 준비** — 이전에 인쇄한 마커(`marker_size=0.0742`, 즉
7.42cm)가 남아있으면 재사용. 없으면:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run aruco_opencv create_marker 0
```
출력된 "Marker side size"를 기록해두고, g_base 기준 고정된 곳(책상 등)에
평평하게 부착 — 로봇이 움직여도 마커는 고정이어야 함.

**3. 로컬 PC에서 캘리브레이션 스택 실행**:

```bash
source /home/minodori/Projects/mycobot/install/setup.bash
ros2 launch mycobot_280_moveit2 handeye_calibration.launch.py marker_size:=0.0742
```
(`marker_size`는 2번에서 확인한 실측값으로. 새 마커를 인쇄했다면 그 값으로
교체.) 이 launch가 `robot_state_publisher`(RPi가 발행하는 `/joint_states`를
구독) + D435(컬러 스트림만, 로컬 PC USB) + `aruco_opencv` 마커 추적 +
`easy_handeye2` 캘리브레이션 서버 + RViz를 한 번에 띄움.

**4. 샘플 수집 — rqt GUI 대신 서비스 콜 사용** (13장에서 이미 확인된
이슈: rqt "Calibrate" GUI가 `Take Sample` 몇 번 하면 Qt 데드락으로
멈추고 그때까지 모은 샘플이 날아감). 팔을 손으로 마커가 화면에 계속
보이는 자세로 옮긴 뒤 매번:

```bash
ros2 service call /easy_handeye2/calibration/take_sample easy_handeye2_msgs/srv/TakeSample "{}"
```

회전 위주로 각 축을 최대한 다양하게 바꿔가며 최소 10~15개(권장 15~20개)
샘플 수집. 특정 샘플이 나쁘면:

```bash
ros2 service call /easy_handeye2/calibration/remove_sample easy_handeye2_msgs/srv/RemoveSample "{sample_index: <제거할 인덱스>}"
```

**5. 계산 및 저장**:

```bash
ros2 service call /easy_handeye2/calibration/compute_calibration easy_handeye2_msgs/srv/ComputeCalibration "{}"
```
결과(translation/rotation, reprojection error) 확인 후 문제없으면:

기존 결과를 덮어쓰기 전에 백업부터:
```bash
cp ~/.ros2/easy_handeye2/calibrations/mycobot_d435_eih.calib \
   ~/.ros2/easy_handeye2/calibrations/mycobot_d435_eih.calib.bak_20260723
```

그 다음 저장(같은 이름 `mycobot_d435_eih`로 저장하면 `demo_octomap.launch.py`가
인자 변경 없이 바로 새 값을 사용함):
```bash
ros2 service call /easy_handeye2/calibration/save_calibration easy_handeye2_msgs/srv/SaveCalibration "{}"
```

**6. 검증** — 이 launch를 종료하고 `demo_octomap.launch.py`를 다시 띄운 뒤,
벽/손처럼 눈에 보이는 물체를 카메라 앞에 두고 팔을 2~3개 다른 자세로
바꿔가며 Octomap voxel이 실제 물체 위치와 일관되게 맞는지 확인(자세가
바뀌어도 재계산 없이 항상 맞아야 정상). 이전 값과 새 값을 비교하고 싶으면
`.bak` 파일을 다시 `.calib`로 복사해 롤백 가능.

### 상태

⬜ 미착수 — 사용자가 직접 진행 예정(2026-07-23).
