# Legion-5 로컬 MoveIt2 환경 구축 트러블슈팅 기록

RPi 없이 노트북(Legion-5, Ubuntu 24.04 / ROS2 Jazzy) 단독으로 `mycobot_280_moveit2`를
빌드하고, RViz에서 로봇 모델을 표시하고, Python(pymoveit2)으로 씬 오브젝트를 등록해
장애물 회피 경로 계획까지 성공시키는 과정에서 겪은 문제와 해결 기록.

---

## 배경: 왜 로컬 단독 실행인가

- `demo.launch.py`의 `ros2_control`은 `mock_components/GenericSystem`(FakeSystem,
  시뮬레이션 모드)으로 동작함. 즉 move_group도 ros2_control도 **실제 로봇팔 없이
  가상 관절값만 계산**하는 상태.
- 따라서 **경로 계획(MoveIt2) 자체를 연습/개발하는 단계에서는 RPi가 필요 없음** —
  성능 좋은 로컬 머신(Legion-5)에서 다 돌리는 게 오히려 빠르고 편함.
- **RPi가 다시 필요해지는 시점**: `mock_components/GenericSystem` → 실제 pymycobot
  기반 시리얼 하드웨어 인터페이스로 전환할 때. 즉 **RPi는 실제 로봇팔과 물리적으로
  연결해서 제어할 때만 필요**하고, 시뮬레이션/개발 단계에서는 로컬 PC 어디서든 가능.

---

## 문제 1: RViz에 로봇 3D 모델이 안 보임 (월드가 비어있음)

### 증상
```
[ERROR] [rviz]: Could not find parameter robot_description_semantic and did not
  receive robot_description_semantic via std_msgs::msg::String subscription
  within 10.000000 seconds.
Error: Could not parse the SRDF XML File. Error=XML_ERROR_EMPTY_DOCUMENT
[ERROR] [rdf_loader]: Unable to parse SRDF
[ERROR] [planning_scene_monitor]: Robot model not loaded
```
또는 `Frame [g_base] does not exist` 경고.

### 원인
`rviz2`를 단독 실행(`rviz2` 명령만)하면 URDF/SRDF를 아무도 파라미터로 넘겨주지
않음. `moveit_rviz.launch.py`는 URDF/SRDF를 **로컬 파일에서 직접 읽어** rviz2에
파라미터로 넘겨주지만, `robot_state_publisher`(TF 발행 담당)를 실행하지 않기 때문에
로봇 3D 모델을 그리는 데 필요한 TF(`g_base` 등)가 존재하지 않음.

### 핵심 구분: 두 launch 파일의 역할

| | `moveit_rviz.launch.py` | `demo.launch.py` |
| --- | --- | --- |
| 띄우는 것 | rviz2만 | rviz2 + move_group + ros2_control + robot_state_publisher |
| URDF/SRDF | 로드함 (rviz2 표시용) | 로드함 (전체 파이프라인용) |
| move_group | ❌ 없음 | ✅ 있음 |
| TF(`/tf`) 발행 | ❌ 없음 → 3D 모델 안 보임 | ✅ 있음 → 3D 모델 보임 |
| 용도 | 분산 구조에서 로컬 쪽 뷰어 전용 (원격 move_group 필요) | 단독 실행 (한 머신에서 전체 파이프라인 완결) |

### 해결
로컬 단독으로 로봇 모델까지 보려면 `demo.launch.py`를 실행해야 함:
```bash
cd ~/Projects/mycobot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mycobot_280_moveit2 demo.launch.py
```

---

## 문제 2: MoveIt2 관련 apt 패키지가 계속 하나씩 누락됨

`demo.launch.py`를 실행할 때마다 `move_group`이 특정 플러그인/라이브러리를 못 찾아
FATAL로 죽는 상황이 반복됨. RPi5는 이미 필요한 패키지가 다 설치돼 있었지만,
Legion-5는 처음 세팅하는 머신이라 하나씩 드러남.

### 순서대로 겪은 에러와 해결

1. **`controller_manager` 패키지 없음**
   ```
   package 'controller_manager' not found
   ```
   ```bash
   sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers -y
   ```

2. **`libsdformat14.so.14` 로드 실패** (robot_state_publisher가 계속 죽었다 재시작)
   ```
   Failed to load library .../libsdformat_urdf_plugin.so ...
   libsdformat14.so.14: cannot open shared object file
   ```
   ```bash
   sudo apt install ros-jazzy-sdformat-urdf -y
   ```

3. **`ompl_interface/OMPLPlanner` 플러그인 없음**
   ```
   [FATAL] Exception while loading planner 'ompl_interface/OMPLPlanner'
   ```
   ```bash
   sudo apt install ros-jazzy-moveit-planners-ompl -y
   ```

4. **`pilz_industrial_motion_planner/CommandPlanner` 없음**
   ```bash
   sudo apt install ros-jazzy-pilz-industrial-motion-planner -y
   ```

5. **`chomp_interface/CHOMPPlanner` 없음** — 여기서 패턴을 깨닫고 메타패키지로 전환
   ```bash
   sudo apt install ros-jazzy-moveit -y
   ```
   이 메타패키지 하나로 OMPL/Pilz/CHOMP/STOMP 등 관련 플래너 패키지가 전부 설치됨.

### 교훈
`demo.launch.py`처럼 여러 플래너(OMPL, Pilz, CHOMP, STOMP)를 한 번에 로드하려는
launch 파일을 새 머신에서 처음 실행할 때는, 하나씩 apt install 하며 두더지잡기
하지 말고 **처음부터 `ros-jazzy-moveit` 메타패키지를 설치**하는 게 훨씬 빠름.
RPi5도 결국 이 메타패키지 수준의 구성이 이미 되어 있었던 것으로 추정.

---

## 문제 3: 가상환경(venv)에서 `rclpy` / `pymoveit2` import 안 됨

### 증상
```
uv pip install rclpy
  × No solution found ... rclpy was not found in the package registry
```
가상환경 활성화 상태에서 VS Code 등으로 스크립트 실행 시 `ModuleNotFoundError: rclpy`.

### 원인
`rclpy`, `pymoveit2` 등은 PyPI 패키지가 아니라 **colcon 워크스페이스에서 빌드되는
ROS2 패키지**. `uv venv`로 만든 가상환경은 기본적으로 시스템 사이트 패키지와
격리되기 때문에 `/opt/ros/jazzy`나 `~/Projects/mycobot/install`에 있는 ROS2
파이썬 모듈을 못 봄.

### 해결: ROS2 작업은 가상환경보다 전역이 잘 맞음
ROS2 자체가 시스템 전역 설치(`/opt/ros/jazzy`) + colcon workspace(`install/`) 조합을
전제로 설계되어 있어서, 여기에 pip venv를 끼워 넣으면 계속 충돌이 생김.

```bash
cd ~/Projects/mycobot
deactivate      # venv 사용 중이면 빠져나오기
rm -rf .venv    # 필요 없으면 삭제

# pyyaml 등 순수 파이썬 패키지가 필요하면 --break-system-packages로 전역 설치
pip install pyyaml --break-system-packages

# 실행은 항상 아래 두 setup.bash를 source한 뒤 시스템 python3로
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/tomato_scene_test.py
```

### pymoveit2 설치 (PyPI에 없음, 소스 clone 필요)
```bash
cd ~/Projects/mycobot/src
git clone https://github.com/AndrejOrsula/pymoveit2.git
cd ~/Projects/mycobot
colcon build --packages-select pymoveit2 --symlink-install
source install/setup.bash
```

---

## 문제 4: Python 스크립트 실행해도 Scene Object가 RViz에 등록 안 됨

### 증상
- `add_collision_sphere()` / `add_collision_box()` 호출 시 에러는 없음
- `ros2 topic echo /collision_object`에도 아무것도 안 뜸
- RViz의 Scene Objects 탭에 아무것도 안 나타남
- 그런데 `move_to_pose()`(팔 이동)는 정상 동작함

### 원인: DDS discovery 레이스 컨디션
`MoveIt2` 객체를 생성하는 순간 `/collision_object` 퍼블리셔가 새로 만들어지는데,
객체 생성 직후 곧바로 `add_collision_sphere()`를 호출해버리면 **move_group(구독자)이
이 새 퍼블리셔를 아직 발견(discover)하기도 전에 메시지가 발행되고 유실**됨.
반면 `move_to_pose()`는 스크립트 흐름상 더 뒤에 호출되기 때문에, 그 사이에
자연스럽게 discovery 시간이 확보되어 정상 동작했던 것.

### 해결
`MoveIt2` 객체 생성 직후, 콜리전 오브젝트를 추가하기 전에 짧은 대기를 넣음:
```python
node.get_logger().info("퍼블리셔 discovery 대기 중...")
time.sleep(2.0)

node.get_logger().info("씬에 토마토 + 장애물 추가 중...")
moveit2.add_collision_sphere(...)
```
적용 후 `demo.launch.py` 로그에 `Published update collision object`가 정상적으로
찍히고, RViz Scene Objects 탭에도 오브젝트들이 나타남.

### 디버깅에 유용했던 명령어
```bash
# 씬 오브젝트가 실제로 발행되는지 직접 확인
ros2 topic echo /collision_object
```

---

## 문제 5: Scene Object 등록은 됐는데 Plan이 실패함

### 증상
```
[ERROR] [planner_manager]: Unable to sample any valid states for goal tree
[ERROR] [move_group]: Planner 'OMPL' failed with error code FAILURE
```

### 원인
`ros2 topic echo`로 실제 좌표를 확인해보니, 목표 접근점(`approach_position`,
x=0.23)이 `leaf_obstacle`(x=0.20 중심, 크기 0.08 → x=0.16~0.24 범위) **내부에
들어가 있었음**. 즉 회피에 실패한 게 아니라, 애초에 **목표 지점 자체가 장애물
속이라 도달 가능한 상태가 없는 상황**을 만든 것.

### 교훈
- 씬에 장애물을 배치할 때는 **박스/구 크기의 절반(half-extent)까지 감안**해서
  목표 좌표와 충분한 여유(수 cm 이상)를 둬야 함.
- OMPL이 "valid state를 못 찾는다"는 에러는 회피 알고리즘의 실패가 아니라
  **목표 자체가 충돌 상태**인 경우가 많으므로, 먼저 좌표를 직접 계산/확인하는 것이
  우선.

### 해결
`leaf_obstacle` 위치를 로봇 쪽으로 당겨(x=0.20 → 0.15) 목표 접근점과 충분히
떨어뜨림. 이후 Plan 성공, RViz에서 팔이 장애물을 피해 우회하는 자세로 정상
계산됨.

---

## 최종 체크리스트 (새 PC에서 이 과정을 반복할 때)

```bash
# 1. 워크스페이스 clone & 필요 패키지 빌드
cd ~/Projects/mycobot/src
git clone -b humble https://github.com/elephantrobotics/mycobot_ros2.git
git clone https://github.com/AndrejOrsula/pymoveit2.git
cd ~/Projects/mycobot

# 2. MoveIt2 관련 apt 패키지는 처음부터 메타패키지로
sudo apt install ros-jazzy-moveit -y
sudo apt install ros-jazzy-sdformat-urdf -y
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers -y

# 3. 전역 python 환경 (venv 사용 안 함)
pip install pyyaml --break-system-packages

# 4. 빌드
colcon build --packages-up-to mycobot_280_moveit2 mycobot_description pymoveit2 --symlink-install

# 5. 실행 (터미널 3개 유지)
# 터미널 1: 백엔드 (계속 켜둠)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mycobot_280_moveit2 demo.launch.py

# 터미널 2: 씬 오브젝트 발행 확인용 (선택)
ros2 topic echo /collision_object

# 터미널 3: 테스트 스크립트
source /opt/ros/jazzy/setup.bash
python3 scripts/tomato_scene_test.py
```

## 로컬 vs RPi 역할 정리

| 단계 | 필요한 것 | RPi 필요 여부 |
| --- | --- | --- |
| MoveIt2 학습, 경로 계획 로직 개발 | 로컬 PC (Legion-5) 단독 `demo.launch.py` | ❌ 불필요 |
| 씬 오브젝트/충돌 회피 테스트 | 로컬 PC 단독 | ❌ 불필요 |
| YOLO 좌표 → MoveIt 목표 변환 로직 개발 (좌표값은 하드코딩/모의값 사용) | 로컬 PC 단독 | ❌ 불필요 |
| 실제 myCobot 280 펌웨어와 시리얼 통신 (`/dev/ttyJETCOBOT`) | RPi + pymycobot 하드웨어 인터페이스 | ✅ 필요 |
| 실물 로봇팔 제어 (수확 동작 실행) | RPi (물리적으로 로봇팔에 연결) | ✅ 필요 |

**요약**: FakeSystem(시뮬레이션) 단계에서는 로컬 PC 어디서든 충분하고, RPi는
`mock_components/GenericSystem`을 실제 하드웨어 인터페이스로 교체해 **실물 로봇을
직접 움직이는 단계에서만** 필요함.