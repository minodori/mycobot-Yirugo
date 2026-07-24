# Octomap 장애물 회피 수동 테스트 절차 (2026-07-23)

`demo_octomap.launch.py` + `coord_to_goal_node`로, look pose에 고정된 실물
카메라가 보는 장애물을 MoveIt2가 실제로 피해서 목표 좌표까지 플래닝하는지
수동으로 확인하는 절차. `docs/look_pose.md`, `docs/handeye_calibration.md`와
이어지는 내용.

## 다음 세션 시작 안내 (2026-07-24 세션 종료 시점)

### 이번 세션에 완료된 것
- **Pointcloud 사전 필터링(로드맵 2단계) 구현 + 실물 검증 완료** — YOLO bbox로
  토마토 영역을 depth 단계에서 사전 제거해 Octomap이 토마토를 장애물로 안
  잡게 함. 신규 `pointcloud_tomato_filter_node.py`(`mycobot_280_pick`) +
  `yolo_d435_detector_node.py` 확장(`tomato_boxes`, `tomato_detections_image`
  발행). 개발 중 버그 2개 발견+수정(아래 "Pointcloud 사전 필터링" 절과 그
  하위 절 참고):
  1. raw pointcloud(depth 센서 그리드) vs 컬러 bbox 그리드 불일치 →
     `aligned_depth_to_color` 기반 자체 디프로젝션으로 재작성.
  2. 출력 topic QoS(BEST_EFFORT) vs occupancy_map_monitor 요구(RELIABLE)
     불일치 → 기본 QoS로 수정.
  수정 후 `bbox_padding_ratio`를 키워가며 실물 D435 + RViz로 토마토 위치에
  Octomap 구멍이 실제로 생기는 것 육안 확인함.
- 디버깅용 시각화 2개 추가: `tomato_detections_image`(YOLO bbox/클래스/
  confidence 그린 이미지, RViz Image 디스플레이용), `/debug_tomato_point`
  발행 + RViz Point 디스플레이로 특정 좌표 위치 육안 확인하는 절차.

### 다음 세션에 할 일 — 우선순위
1. **그리퍼 근접거리 self-filter 재평가** (남은 작업 우선순위 2번): pointcloud
   사전 필터링이 실물 검증됐으니, `sensors_3d.yaml`의 `padding_scale: 0.92`
   타협이 지금도 필요한지, 잔여 voxel 문제가 줄었는지 재확인.
2. **IK 마진널 실패 조사** (로드맵 3번, 급하지 않음): 이번 세션에 g_base
   [0.219,0.054,0.311] 근처 좌표에서 `Unable to sample any valid states for
   goal tree`(플래닝 자체 실패, orientation 무관)가 재현됨 — 지난 세션
   (0.125,-0.15,0.204)와 같은 부류. 근본 원인(실물-시뮬 미세 드리프트 vs
   순수 위치의 IK 특이점) 미확인. 아래 "새로운 위치에서도 ... 재현" 절 참고.
3. `bbox_padding_ratio` 최종값 확정 — 세션 종료 시점 `DEFAULT_BBOX_PADDING_RATIO
   = 0.5`(실물 조정 중, 0.2 기본값은 파일에 주석으로 남겨둠). 더 튜닝하거나
   확정할 것.
4. 이후 원래 로드맵(카메라 감지 → 좌표 계산 → 장애물 회피 플래닝 → 실물
   이동 전체 파이프라인 완성) 계속 진행, 완성되면 automato_ws로 포팅
   (사용자 확인된 방침, 다시 묻지 말 것).

### 참고
- 커밋은 요청 시에만.
- RPi `sync_plan`은 실물 이동 테스트 아닐 땐 꺼둔 채로 두는 게 안전(이번
  세션 종료 시점 꺼짐 상태).

## 사전 조건

- 실물 팔이 **look pose**(`docs/look_pose.md`, `[-2.98, 104.41, -31.81, -76.2, 10.54, 7.03]` 도, 혹은 그 이후 조정한 값)에 **토크 걸린 채 고정**돼 있어야
  함 — 릴리즈 상태로 두면 실물 카메라 위치가 시뮬레이션 가정과 어긋나서
  Octomap이 잘못된 곳에 장애물을 그림(이 문서 "왜 look pose 동기화가
  필요한가" 참고).
- `mycobot_280_moveit2/config/initial_positions.yaml`이 그 look pose와 같은
  라디안 값으로 설정돼 있어야 함(시뮬레이션 시작 자세 = 실물 자세). **주의**:
  이 파일은 `src/mycobot_ros2/`(gitignore됨) 안에 있어서 git으로 안 잡힘 —
  재클론/재vendoring 시 유실될 수 있으니 별도 백업 권장.
- D435가 노트북 USB에 연결돼 있고 `power/control=on`(USB autosuspend
  끔 — 아래 "카메라가 불안정하면" 참고).

## 왜 look pose 동기화가 필요한가

D435는 eye-in-hand(joint6 마운트)라, Octomap의 장애물 voxel은
`depth 포인트(카메라 프레임) → camera_link → joint6(handeye 고정 변환) →
... → g_base`를 거쳐 배치된다. 이 `joint6`은 **시뮬레이션(FakeSystem)의
현재 관절값**으로 계산되는데, 만약 실물이 그 값과 다른 자세에 있으면(예:
릴리즈 상태로 손이 움직여놨거나 시뮬레이션만 다른 곳으로 이동시킨 경우)
Octomap은 "시뮬레이션이 가정하는 위치"에 장애물을 그리고, 실물 팔은
"진짜 정확한 자기 위치"로 움직이기 때문에 화면(RViz)과 실제 물리 공간이
어긋난다. 그래서 **실물과 시뮬레이션을 같은 관절값으로 맞춰두고 시작**하는
게 전제 조건.

## 1. 실물 팔을 look pose로 이동 (토크 걸어서 고정)

이미 look pose에 있으면 생략. 아니라면 RPi 경유로 `send_angles`:

```bash
ssh jetcobot_126b 'python3 -c "
from pymycobot import MyCobot280
import time
mc = MyCobot280(\"/dev/ttyUSB0\", 1000000)
time.sleep(0.1)
mc.send_angles([-2.98, 104.41, -31.81, -76.2, 10.54, 7.03], 20)
time.sleep(3)
print(mc.get_angles())
"'
```

⚠️ **안전 확인**: 릴리즈 상태에서 토크가 다시 걸리면 현재 위치에서 목표
자세로 갑자기 움직일 수 있음 — 팔 주변에 사람/물건 없는지 확인 후 실행.

## 2. `initial_positions.yaml`을 같은 값(라디안)으로 맞추고 빌드

```bash
python3 -c "
import math
deg = [-2.98, 104.41, -31.81, -76.2, 10.54, 7.03] # 1번에서 보낸 값과 동일하게
for d in deg:
    print(math.radians(d))
"
```
결과를 `src/mycobot_ros2/mycobot_280/mycobot_280_moveit2/config/initial_positions.yaml`의
`joint2_to_joint1`~`joint6output_to_joint6`에 순서대로 채워넣고:

```bash
cd ~/Projects/mycobot
colcon build --packages-select mycobot_280_moveit2 --symlink-install
```

## 3. 기존 프로세스 전부 정리 후 시뮬레이션 스택 실행

이전에 띄운 게 남아있으면(특히 `ros2 launch` 부모 프로세스가 안 죽고
자식을 되살리는 경우가 있었음 — 반드시 `ps aux`로 실제 프로세스까지
확인) 전부 종료:

```bash
pkill -9 -f "move_group|rviz2|realsense2_camera_node|ros2_control_node|robot_state_publisher|handeye_publisher|static_transform_publisher|coord_to_goal_node"
ps aux | grep -iE "ros2 launch|move_group|rviz2|realsense2_camera_node" | grep -v grep   # 비어있는지 확인
```

그 다음 실행:

```bash
cd ~/Projects/mycobot && source install/setup.bash
ros2 launch mycobot_280_moveit2 demo_octomap.launch.py
```

`move_group` 로그에 `You can start planning now!`가 뜨면 준비 완료. RViz에
RobotModel/Octomap이 정상 표시되는지 확인.

## 4. `coord_to_goal_node` 실행

다른 터미널에서:

```bash
cd ~/Projects/mycobot && source install/setup.bash
ros2 run mycobot_280_pick coord_to_goal_node
```

`/target_point` 구독 대기 로그가 뜨면 준비 완료.

## 5. 목표 좌표 정하기 — RViz "Publish Point"로 직접 클릭

RViz 툴바에 Publish Point 도구가 없으면 `+` 버튼으로 추가. 장애물(Octomap
voxel) 옆 원하는 지점에 클릭하면 상태바에 `g_base` 기준 좌표가 뜸(이
방법은 `camera_mount_investigation.md` 10장에서 이미 확립됨). 그 좌표를
그대로 다음 단계에 사용.

**주의**: 그리퍼 현재 위치에서 너무 가까운 좌표(대략 반경 13cm 이내 —
`TARGET_OBJECT_RADIUS`(0.05m) + 그리퍼 자체 크기)를 찍으면 `coord_to_goal_node`가
등록하는 목표 표시용 구(sphere)가 그리퍼 자기 자신과 겹쳐서
`START_STATE_IN_COLLISION`으로 바로 실패함 — 현재 end-effector 위치는:

```bash
ros2 run tf2_ros tf2_echo g_base joint6_flange
```
로 확인 가능. 이 위치에서 20cm 이상 떨어진 좌표부터 시도할 것.

## 6. (필요시) Octomap 수동 클리어

`coord_to_goal_node`가 플래닝 직전 자동으로 `/clear_octomap`을 호출하도록
패치돼 있지만(2026-07-23), 카메라가 계속 스트리밍 중이라 클리어 직후 바로
새 프레임이 들어오면 그리퍼 근처에 노이즈성 voxel이 다시 생겨
`START_STATE_IN_COLLISION`이 재발할 수 있음. 목표 발행 전/후로 계속
실패하면 직접 한 번 더 클리어:

```bash
ros2 service call /clear_octomap std_srvs/srv/Empty {}
```

클리어 직후 바로 목표를 발행해야 재발 확률이 낮음(수백 ms~1초 정도의
좁은 창).

## 7. 목표 발행

```bash
ros2 topic pub --once /target_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'g_base'}, point: {x: <X>, y: <Y>, z: <Z>}}"
```

`coord_to_goal_node` 터미널에 `목표 위치로 플래닝: [...]` 로그가 뜨고,
RViz에서 팔이 장애물(Octomap voxel)을 피해 움직이면 성공.

## 알려진 문제 / 실패 패턴

- **`START_STATE_IN_COLLISION` (`<octomap> - gripper_base`) — 원인 규명,
  실용적 타협으로 완화(2026-07-23)**: "근접거리 노이즈"로 오래 추정했었으나,
  실제로는 **카메라가 자기 그리퍼(joint6_flange 기준 손가락 끝단까지
  ~70~110mm)를 프레임에 계속 담고 있는데, `sensors_3d.yaml`의 self-filter가
  그 형상을 다 못 덮어서 자기 그리퍼를 장애물로 오인**하는 것이 원인 중
  하나로 확인됨(카메라 프레임 캡처로 그리퍼 몸체가 실제로 화면에 찍히는
  것 확인). `padding_scale`을 1.0 이상(1.3, 1.05 등)으로 올려 그리퍼는
  안 잡히게 시도했으나, 그러면 **실제 물체(target 등)까지 과도하게
  필터링되는 부작용**이 있었음 — 이건 반드시 **RViz Occupied Voxels를
  육안으로 직접 확인**해서 판단해야 함(`get_planning_scene`의 바이트 수 등
  간접 지표는 free/unknown space까지 포함돼 신뢰할 수 없었고, octomap
  메시지가 표준 `.ot`/`.bt` 파일 헤더 형식이 아니라서 파이썬으로 직접
  디코딩하는 시도도 실패함). 최종적으로 사용자가 RViz 육안 확인 기준으로
  직접 튜닝한 **`padding_scale: 0.92`, `padding_offset: 0.0`**을 채택 —
  실제 물체는 정상적으로 잡히고, 그리퍼 근처엔 잔여 voxel이 1개 정도
  남는 수준까지 타협. 이 잔여분은 `coord_to_goal_node`가 플래닝 직전
  자동 호출하는 `/clear_octomap`으로 커버(완벽한 self-filter보다 실용적
  타협). (`sensors_3d.yaml` 값이 바뀔 때마다 `colcon build --packages-select
  mycobot_280_moveit2` 후 완전 재시작 필요 — `ros2 param set`으로 런타임에
  바꿔도 반영 안 됨을 확인함.)
- **`START_STATE_IN_COLLISION` (`gripper_base - target_object`) — ✅ 진짜
  근본원인 발견+수정(2026-07-23)**: 위 항목과 별개로, 목표 지점이 그리퍼에서
  30cm+ 떨어져 있어도 매번 이 충돌이 나던 문제의 진짜 원인은 **그리퍼 메시
  (.dae) 파일이 COLLADA 내부에 밀리미터 단위로 선언돼 있는데
  (`<unit meter="0.001"/>`) URDF `<mesh>` 태그에 `scale` 속성이 없어서, MoveIt
  충돌 메시 로더가 원시 좌표를 그대로 미터로 해석 — 그리퍼 콜리전 형상이
  실제 크기(~58mm)의 1000배(~58m)로 잡히고 있었던 것**(arm 자체 메시는 unit
  태그가 없고 이미 미터 단위라 이 버그의 영향 없음 — 그리퍼만 증상 있었던
  이유). SRDF의 자체충돌 disable_collisions는 로봇 링크끼리만 적용되고
  월드 오브젝트(target_object, octomap)와의 충돌은 못 막아서, 이게 근본
  원인이었는데도 SRDF 패치로는 전혀 안 잡혔던 것. **수정**:
  `mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5_adaptive_gripper.urdf`의
  그리퍼 7개 링크 `<collision>` 블록 `<mesh>` 태그에
  `scale="0.001 0.001 0.001"` 추가(`<visual>`은 안 건드림 — 렌더링은 이미
  정상이었음). `mycobot_description` 재빌드 필요. 수정 후 30cm+ 목표에서
  이 충돌 재현 안 됨 확인.
- **`Unable to sample any valid states for goal tree` — 원인 규명, 완전
  해결은 아직(2026-07-24)**: `coord_to_goal_node`가 쓰던 `approach_quat`
  identity 고정값은 이 URDF 좌표계에서 "그리퍼가 하늘을 보는" 방향이라
  대부분의 실제 목표에서 도달 불가/무의미했음. **1차 수정**으로 look
  pose에서 그리퍼가 실제로 갖는 orientation(`APPROACH_QUAT_XYZW =
  [-0.538, 0.482, -0.442, 0.532]`, `tf2_echo g_base joint6_flange`로
  실측)을 고정값으로 채택하고 `tolerance_orientation`도 0.5rad로 완화함
  (`coord_to_goal_node.py` 수정 완료, 재빌드 반영됨).
  **그런데 이 고정값도 범용은 아님이 확인됨** — 목표 (0.125,-0.15,0.204)는
  이 값으로 성공하지만, 목표 (0.158,0.198,0.152)는 오히려 identity가
  맞았고 이 고정값으로는 tolerance를 풀어도 실패함. 즉 **위치마다 필요한
  orientation이 달라서, 고정값 하나로는 작업공간 전체를 커버할 수 없음**
  (6축 팔의 IK 특성상 자연스러운 결과). `GRIPPER_ORIENTATION`
  (`arm_solver_direct.py`, pymycobot 자체 RPY `[-133,7,-100]`)도 오일러
  변환식 검증 후 쿼터니언으로 바꿔 여러 위치에서 시도했으나 전부 실패 —
  그 값은 과거 특정 실제 좌표에서만 유효했던 것으로 추정.
- **카메라가 불안정하면**(depth 스트림이 거의 안 들어오거나 `Hardware
  Error` 반복): USB autosuspend 문제일 수 있음.
  ```bash
  cat /sys/bus/usb/devices/*/power/control   # D435 포트가 'on'인지 확인
  ```
  udev 규칙(`/etc/udev/rules.d/99-realsense-no-autosuspend.rules`)이
  이미 있으면 재연결 시 자동 적용됨. 그래도 불안정하면 다른 케이블/포트로
  교체.

## 목표를 바라보는 orientation 동적 계산 (방식 B) — 구현 완료 (2026-07-24)

**배경**: 위 "알려진 문제"에서 확인했듯, 고정 orientation(identity든
look-pose 값이든 `GRIPPER_ORIENTATION`이든) 하나로는 작업공간 전체를
커버 못함. 근본 해결은 매 목표마다 "그리퍼가 목표 방향을 바라보는"
orientation을 동적으로 계산하는 것(그래픽스의 look-at 행렬과 같은 원리).

**그리퍼 "정면" 로컬 축 확정**: `joint6_flange`의 로컬 **+Z축**이 그리퍼가
물체를 향해 뻗어나가는 방향으로 확인됨. 실물 팔을 움직이는 실측(당초
계획) 대신, URDF 링크체인(`joint6_flange` -> `gripper_base`(고정 조인트,
origin z=0.034m) -> 손가락 링크들, `mycobot_280_m5_adaptive_gripper.urdf`)
으로 FK를 계산해 손가락 중점이 flange 원점 기준 거의 순수한 로컬 +Z
방향(정규화 벡터 `[0, -0.003, 0.99999]`, 거리 ~5.5cm)에 있음을 확인 —
움직임 없이 기하학적으로 결정 가능한 값이라 실물 조작의 안전 부담 없이
검증함(스크립트로 계산). look pose 실측 쿼터니언(`[-0.538, 0.482, -0.442,
0.532]`)으로 교차검증해도 이 +Z축이 g_base 기준 `[0.988, 0.146, -0.043]`
방향(대략 로봇 앞쪽)을 가리켜 물리적으로도 합당함 — 기존
`GRIPPER_BOX_Z_OFFSET` 계산이 가정했던 축과 일치함도 확인.

**구현**: `coord_to_goal_node.py`에 다음을 추가/교체함.
1. `_normalize`/`_cross`/`_dot`/`_rotation_matrix_to_quat_xyzw`/
   `_compute_look_at_quat_xyzw` — 순수 `math` 표준 라이브러리만으로 구현
   (numpy/scipy 신규 의존성 추가 안 함). `_rotation_matrix_to_quat_xyzw`는
   표준 Shepperd's method.
2. `_compute_look_at_quat_xyzw(forward)`: `forward` 정규화 → 기준 "up"
   벡터(`WORLD_UP = (0,0,1)`, forward와 거의 평행하면 `(1,0,0)`으로 대체해
   degenerate 회피) → `right = normalize(cross(up, forward))` →
   `up' = cross(forward, right)` → 세 축을 회전행렬 열(column)로 조립
   (로컬 X=right, Y=up', **Z=forward**) → 쿼터니언 변환.
3. `_compute_approach_quat(target_position)`(노드 메서드): `_tf_buffer`로
   현재 `g_base -> joint6_flange` 위치를 조회해 `forward = target -
   현재_EE_pos`를 계산 → 위 함수 호출. TF 조회 실패나 forward 벡터가
   degenerate(목표가 현재 위치와 거의 같음)면 기존 고정값
   (`FALLBACK_APPROACH_QUAT_XYZW`, 옛 `APPROACH_QUAT_XYZW`와 동일)으로
   안전하게 폴백.
4. `_on_target_point`가 목표 원좌표(`_pending_target_position`)도 저장하도록
   추가(기존엔 접근 오프셋이 적용된 `_pending_approach_position`만 저장했음).
5. `_start_planning`이 고정값 대신 `_compute_approach_quat()` 호출 결과를
   사용, 로그에 계산된 orientation도 같이 출력.

**검증**: look-at 수식 자체는 스크립트로 여러 forward 벡터(축 정렬, 임의
방향, `world_up`과 거의 평행한 degenerate 케이스 포함)에 대해 "계산된
쿼터니언을 회전행렬로 되돌렸을 때 로컬 +Z축이 의도한 forward 방향과
정확히 일치하는지"를 assert로 확인함 — 전부 통과(오차 ~1e-16 수준,
degenerate 폴백 케이스도 정상 동작). `colcon build --packages-select
mycobot_280_pick` 성공, 모듈 import 및 문법 확인 완료.

## 실물 end-to-end 검증 결과 (2026-07-24, 같은 세션)

실물 팔(look pose, 토크 고정, 실측 각도 look pose와 ±1도 이내)에 연결된
채로 `demo_octomap.launch.py`(FakeSystem 시뮬레이션 — 하드웨어 플러그인이
`mock_components/GenericSystem`이라 MoveIt2가 "실행"해도 실제 서보는
안 움직임, 실물은 정지 상태 유지) + 재빌드한 `coord_to_goal_node`로 과거
성공/실패가 갈렸던 두 좌표를 재테스트함.

**1차 결과 — orientation 문제(IK 도달 불가) 자체는 훨씬 개선됨, 단
pymoveit2 기본 플래닝 예산이 너무 짧았음**: 초기 테스트에서
(0.158,0.198,0.152)(과거 identity만 성공하던 좌표)가 새 동적 orientation
(`[0.342,0.706,0.558,0.27]`)으로 3회 연속 `Unable to sample any valid
states for goal tree`(플래닝 자체 실패)로 실패함. 원인 확인 결과
`pymoveit2` `MoveIt2` 기본값이 `allowed_planning_time=0.5초`,
`num_planning_attempts=5`로 — OMPL(RRTConnect)의 목표 상태 샘플링이
확률적이라 IK 여유가 좁은 목표에서는 이 예산으로 못 찾는 경우가 많음.
**수정**: `coord_to_goal_node.py`에 `PLANNING_TIME_SEC=3.0`,
`PLANNING_ATTEMPTS=10`을 추가하고 `MoveIt2` 생성 직후
`allowed_planning_time`/`num_planning_attempts` 프로퍼티로 설정.

**수정 후 재테스트**:
- **(0.158,0.198,0.152)**: 3/3 **플래닝 성공**("Solution found")으로 전환
  — 동적 orientation + 늘어난 플래닝 예산의 조합으로 과거 실패 좌표가
  안정적으로 풀림. (실행은 그리퍼 근접 잔여 octomap voxel과 충돌해
  중간에 abort되긴 했으나, 이는 아래 별개 항목 참고 — orientation/IK
  문제와는 무관.)
- **(0.125,-0.15,0.204)**: 늘어난 예산으로도 3/3 `Unable to sample any
  valid states for goal tree`로 실패. **회귀(동적 orientation 때문에
  나빠진 것) 여부를 확인하기 위해** 같은 접근 위치(`[0.045,-0.15,0.204]`)에
  **옛 고정값**(`FALLBACK_APPROACH_QUAT_XYZW`, look pose 실측 orientation)
  으로도 별도 스크립트(`/tmp` 스크래치패드, 세션 한정)로 직접 테스트했더니
  **이것도 동일하게 3초/10회 예산 전부 소진하며 실패**함. 즉 이 위치는
  **orientation 선택과 무관하게 현재 상태에서 IK가 마진널(경계)한 위치**로
  보임 — 동적 orientation이 이 특정 좌표를 "악화"시킨 게 아니라, 애초에
  이 지점 자체가 어려운 케이스였을 가능성이 높음(실물 관절각이 문서
  기록값과 ±1도 정도 미세하게 달라진 것도 마진널한 IK 샘플링 결과에
  영향을 줬을 수 있음 — 확정 원인은 아님, 후속 조사 필요).

**종합 평가**: 동적 look-at orientation은 최소 한 좌표에서 명확한 개선을
보였고(0/3 → 3/3), 다른 좌표에서 원인 불명의 실패가 있었지만 그건 옛
고정값도 마찬가지로 실패해 orientation 계산 자체의 결함으로 보이진 않음.
**세션 시간 제약으로 이 마진널 케이스의 근본 원인까지는 못 팠음** — 다음
세션 후속 조사 대상.

**다음 세션 후속 조사**:
- (0.125,-0.15,0.204) 마진널 실패의 근본 원인(실물-시뮬 미세한 각도
  드리프트 때문인지, 순수 위치 자체의 IK 특이점 근접 때문인지) 확인
- `WORLD_UP` 기준(roll 고정)이 유일한 정답은 아님 — 이 기준으로 고정한
  roll이 마침 IK 불가능한 각도일 수 있어, 향후엔 forward축 둘레로 몇 개
  후보 roll을 순차 시도하는 방식도 고려할 만함(지금은 구현 안 함, 위
  마진널 케이스가 실제로 roll 선택 문제인지부터 확인 필요)
- 그리퍼 근접 잔여 octomap voxel로 인한 실행 중 abort(기존 알려진 문제,
  아래 항목 참고)는 이번 테스트에서도 재현됨 — 별개로 계속 미해결

## Pointcloud 사전 필터링 (로드맵 2단계, 2026-07-24 구현)

**배경**: 위 "알려진 문제"의 `START_STATE_IN_COLLISION` 계열은 전부 "카메라가
실제 물체(토마토, 자기 그리퍼)를 depth로 보고 Octomap이 장애물로 찍은
것"에서 파생됨. 지금까지 대응은 전부 **사후 대응**이었음 — 목표 지점에
sphere CollisionObject를 등록해 그 자리만 필터되게 유도(`coord_to_goal_node`)
하거나, 플래닝 직전 `/clear_octomap`을 호출해 통째로 비우거나
(`padding_scale` 타협). 둘 다 타이밍/범위 의존적인 임시방편이라 잔여 voxel
문제가 계속 재발함.

**근본 대응**: YOLO 검출 bbox로 depth pointcloud 자체에서 토마토 영역
포인트를 사전에 제거(NaN 처리)해서, 목표든 아니든 검출된 토마토가 애초에
Octomap에 장애물로 등록되지 않게 함.

**구현** (`mycobot_280_pick` 패키지):
1. `yolo_d435_detector_node.py`: 기존 YOLO 추론 결과(1회, 추가 추론 없음)에서
   검출된 **모든** bbox(클래스/ripe 여부 무관 — 이 모델은 토마토 상태
   4클래스만 검출하므로 전부 "토마토")를 `[x1,y1,x2,y2, ...]` 평탄화해
   `tomato_boxes`(`std_msgs/msg/Float32MultiArray`)에 추가로 발행. 검출이
   없는 프레임에도 빈 배열을 발행해 필터 노드의 마스크가 제때 풀리게 함.
2. **신규** `pointcloud_tomato_filter_node.py`: `tomato_boxes` +
   `aligned_depth_to_color`(image_raw + camera_info)를 구독해, bbox 픽셀
   영역의 depth를 NaN 처리한 뒤 픽셀->3D 변환(핀홀 근사, `yolo_d435_detector_node`와
   같은 공식)을 스스로 수행해 `/camera/camera/depth/color/points_filtered`에
   발행. (아래 "raw pointcloud 그리드 불일치 버그" 참고 — 최초 구현은 realsense
   raw pointcloud를 그대로 썼다가 실패해서 이 방식으로 교체함.) bbox가
   없으면(`tomato_boxes` 미수신) 마스킹 없는 pointcloud를 그대로 발행.
3. `sensors_3d.yaml`의 `point_cloud_topic`을 raw 토픽에서
   `.../points_filtered`로 변경 — occupancy_map_monitor가 필터링된
   pointcloud만 보게 됨.
4. `demo_octomap.launch.py`: `pointcloud_tomato_filter_node`를 항상 같이
   띄우도록 추가(YOLO 노드가 없어도 안전하게 마스킹 없는 pointcloud를 그대로
   발행).

⚠️ **`sensors_3d.yaml`/`demo_octomap.launch.py`는 `src/mycobot_ros2/`
(gitignore된 vendor clone) 안에 있어서 git으로 안 잡힘** — 재클론/재vendoring
시 이 절 내용을 참고해서 다시 적용할 것 (`initial_positions.yaml`과 같은
패턴, 이 문서 맨 위 "사전 조건" 참고).

### raw pointcloud 그리드 불일치 버그 — 발견 + 수정 (2026-07-24, 같은 세션)

**증상**: 첫 구현(`pointcloud.ordered_pc:=true`로 받은 realsense raw
pointcloud `/camera/camera/depth/color/points`를 그대로 구독해 YOLO bbox
픽셀 좌표를 그 그리드에 직접 적용)을 실물로 띄워보니, `/tomato_boxes`와
`/camera/camera/depth/color/points_filtered`는 정상 발행되는데(각각
`ros2 topic echo`/`hz`로 확인) **RViz Octomap에서 토마토 위치에 구멍이 전혀
안 생김**.

**원인**: `ros2 topic echo /camera/camera/depth/color/points --field header`로
확인한 결과 `frame_id: camera_depth_optical_frame`— 이 pointcloud는 **depth
센서 고유의 픽셀 그리드**를 따름(컬러 텍스처는 별도 재투영으로 입혀지지만
그리드 자체는 depth 원본 해상도/광학중심 기준이고, `align_depth.enable`을
켜도 이 pointcloud 생성 경로 자체는 안 바뀜). 반면 YOLO bbox는 컬러 이미지
픽셀 좌표. 두 그리드는 해상도 숫자(640x480)만 우연히 같을 뿐 광학중심/FOV가
다른 별개 좌표계라서, bbox를 그대로 적용하면 엉뚱한 픽셀이 마스킹되고
실제 토마토 위치는 그대로 남음. `yolo_d435_detector_node`가 애초에 raw
depth 대신 `aligned_depth_to_color`를 쓰는 이유가 정확히 이 정렬 문제
때문인데(그 모듈 상단 주석에 이미 있었음), 필터 노드를 새로 만들 때 이
교훈을 놓쳤던 것.

**수정**: `pointcloud_tomato_filter_node`가 realsense pointcloud를 아예
구독하지 않고, `yolo_d435_detector_node`와 똑같이
`aligned_depth_to_color`(image_raw + camera_info)를 직접 구독해서 픽셀->3D
변환(핀홀 근사, 왜곡 무시 — 같은 근거로 yolo 노드도 무시함)을 스스로
수행하도록 재작성함. bbox 마스킹과 pointcloud 생성이 완전히 같은 픽셀
그리드(컬러 프레임에 정렬된 depth) 위에서 이뤄지므로 그리드 불일치가
구조적으로 불가능해짐. 이에 따라 `demo_octomap.launch.py`의
`pointcloud.ordered_pc:=true` 인자도 더 이상 필요 없어져 제거함(원래 목적이
raw pointcloud를 organized로 받기 위함이었는데, 이제 그 raw pointcloud
자체를 안 씀). `package.xml`의 `sensor_msgs_py` 의존성도 더 이상 안 써서
제거.

**단위 테스트(세션 한정 스크래치패드, 하드웨어 없이 검증, 수정 후 버전
기준)**: 4x4 합성 depth 이미지(픽셀마다 다른 depth값 인코딩 + 사전 존재하는
무효(0) 픽셀 1개 포함)로 (1) bbox 영역이 정확히 NaN 처리되고 나머지 픽셀은
핀홀 공식으로 정확한 x/y/z가 나오는지, (2) 원본 depth 메시지의
`frame_id`(color-aligned 프레임)가 출력에 그대로 유지되는지, (3) bbox가
없으면 마스킹 없이(사전 무효 픽셀만 NaN인 채로) 전체 발행하는지, (4)
camera_info를 아직 못 받았으면 아예 발행하지 않는지(intrinsics 없이 잘못된
좌표 발행 방지) 각각 assert로 확인 — 전부 통과. `yolo_d435_detector_node`의
`_publish_tomato_boxes`도 mock `Result`(빈 리스트/`None`/복수 박스)로 평탄화
로직만 별도 검증 — 전부 통과. `colcon build --packages-select
mycobot_280_pick` 성공, `ament_flake8`/`ament_pep257`은 이 패키지 전체가
이미 갖고 있던 것과 동일한 종류의 pre-existing 스타일 경고(D205/D400)만
있고 새 회귀 없음.

**디버깅용 시각화 추가**: 실물 재검증 중 "Octomap에 구멍이 안 생김"이
관측됐을 때 그 원인이 (a) 필터링 버그인지 (b) YOLO가 애초에 그 토마토를
검출/분류 못한 것인지 구분이 안 되는 문제가 있었음(`/target_point`,
`/tomato_boxes`는 숫자만 나와 육안 디버깅 불가). 그래서
`yolo_d435_detector_node`에 `tomato_detections_image`
(`sensor_msgs/msg/Image`, ultralytics `Results.plot()`로 bbox/클래스명/
confidence를 그린 컬러 이미지) 발행을 추가함. RViz에서 Image 디스플레이를
추가하고 토픽을 이걸로 설정하면 YOLO가 실제로 무엇을 어떤 클래스로
검출했는지 바로 확인 가능.

**미검증 (다음 세션 실물 재확인 필요)**:
- 수정된 버전으로 실제 D435를 다시 띄워서 RViz Octomap에 토마토 위치 구멍이
  실제로 생기는지 육안 확인(이번 세션엔 위 버그 발견까지만 하고 코드
  수정/단위테스트로 끝남 — 실물 재검증은 다음 차례).
- `tomato_detections_image`로 YOLO가 수확 대상 토마토를 실제로 검출/분류
  하는지부터 확인(작아서 못 잡거나 'ripe'가 아닌 다른 클래스로 분류될
  가능성 있음 — 이번 세션 사용자 관찰).

**bbox 패딩 추가(2026-07-24, 같은 세션)**: 실물 테스트에서 토마토가 작아서
Octomap 반영 여부를 육안으로 판단하기 애매하다는 관찰이 있었음. bbox
자체는 물체 경계에 딱 맞게(타이트) 잡히는 경우가 많고, depth-color
정렬에도 픽셀 단위 슬랙이 있을 수 있어, bbox를 그대로만 마스킹하면
테두리 voxel이 안 지워지고 남을 위험이 있음. `pointcloud_tomato_filter_node`에
`bbox_padding_ratio` 파라미터(기본값 0.2 = 각 변 20% 확장, 노드 실행 시
`--ros-args -p bbox_padding_ratio:=<값>`로 조정 가능)를 추가해 bbox를
비율만큼 바깥으로 확장해서 마스킹하도록 함. 비율 기반이라 거리가 멀어져
bbox가 작아져도 상대적 여유가 유지됨. 합성 데이터로 패딩 확장 범위가
정확한지(원본 bbox보다 넓고, 패딩 전에는 안 지워졌을 테두리 픽셀이 실제로
지워지는지) 단위 테스트로 검증 — 통과. `colcon build` 성공.
- 이 필터링이 실제로 적용된 뒤에도 그리퍼 근접 잔여 voxel
  (`padding_scale: 0.92` 타협) 문제가 얼마나 줄어드는지 재평가 — 남은 작업
  우선순위 2번("그리퍼 근접거리 self-filter 재평가")은 이 검증 이후 진행할 것.
- `filtered_cloud_topic: filtered_cloud`(occupancy_map_monitor 자체 self-filter
  출력, 이번 변경과 별개)와 새 `points_filtered` 토픽명이 혼동되지 않는지
  RViz 토픽 목록에서 확인.

### QoS 불일치 버그 — 발견 + 수정 (2026-07-24, 같은 세션)

**증상**: 패딩 파라미터 테스트 중 `pointcloud_tomato_filter_node` 실행 시
`New subscription discovered on topic '.../points_filtered', requesting
incompatible QoS. No messages will be sent to it. Last incompatible policy:
RELIABILITY` 경고 발생 — occupancy_map_monitor가 이 토픽에 구독은 했지만
메시지를 전혀 못 받는 상태였음.

**원인**: 출력 publisher를 `qos_profile_sensor_data`(BEST_EFFORT)로 만들었는데,
`occupancy_map_monitor`(MoveIt `PointCloudOctomapUpdater`)는 이 토픽을
RELIABLE로 구독 요청함. BEST_EFFORT 발행자는 RELIABLE 구독자를 만족 못 시켜서
(QoS 호환성 규칙상 구독자가 요구하는 신뢰성 수준이 발행자보다 높으면
비호환) 메시지가 전달 안 됨. (raw 카메라 토픽이 RELIABLE로 발행되는 걸
`ros2 topic info --verbose`로 확인했었는데, 그때는 "필터 노드가 구독하는
입력" QoS 얘기였고, 이번에 문제가 된 건 반대로 "필터 노드가 발행하는 출력"
QoS라 별개로 다시 확인이 필요했던 것.)

**수정**: 출력 publisher를 기본 QoS(reliable, keep-last depth 10)로 변경.
`pointcloud_tomato_filter_node.py`에서 `qos_profile_sensor_data` import 및
사용 제거.

**교훈**: 이 프로젝트에서 실물 D435/occupancy_map_monitor와 연동하는 새
토픽을 만들 때는 입력/출력 양쪽 QoS를 각각 `ros2 topic info <토픽> --verbose`로
확인할 것 — "카메라 쪽은 보통 best-effort"라는 가정만으로 반대쪽(MoveIt
내부 구독자)까지 넘겨짚으면 틀릴 수 있음.

### 실물 재검증 결과 — pointcloud 사전 필터링 동작 확인 (2026-07-24, 같은 세션)

위 두 수정(그리드 정렬 버그, QoS 버그) 이후 실물 D435 + RViz로 재검증:
- `bbox_padding_ratio`를 기본 0.2에서 점차 키워보며(0.4 → 1.0 → 0.5 순으로
  실물에서 직접 조정) 테스트한 결과, **패딩을 충분히 키우자 RViz Octomap에서
  토마토 위치에 실제로 구멍이 생기는 것을 육안으로 확인함** — 사전 필터링이
  의도대로 동작함. 최종 채택 값은 사용자가 실물로 조정 중(현재 파일 내
  `DEFAULT_BBOX_PADDING_RATIO = 0.5`, `# DEFAULT_BBOX_PADDING_RATIO = 0.2`는
  주석으로 남겨둠 — 필요시 되돌릴 수 있게).
- 검출 좌표 육안 검증 절차 확립: `/target_point`로 나온 카메라 프레임 좌표를
  별도 디버그 토픽(`/debug_tomato_point`, `geometry_msgs/msg/PointStamped`)에
  발행하고 RViz에 **Point** 디스플레이(반경 조절 가능)로 추가하면, TF를 통해
  Fixed Frame으로 변환된 위치가 Octomap 구멍과 일치하는지 바로 비교 가능
  (`/target_point`로 직접 보내면 `coord_to_goal_node`가 떠 있을 때 실제
  플래닝이 트리거될 수 있어 별도 토픽 사용 권장).
- Octomap은 새 프레임이 "이전에 이미 점유된 voxel"을 능동적으로 지우지
  않는다는 점(위 배경 설명 참고)이 실물 테스트에서도 재확인됨 — 패딩 값을
  바꾼 뒤에는 반드시 `/clear_octomap` 호출 후 재축적해서 비교해야 정확한
  before/after가 보임.

**결론**: pointcloud 사전 필터링(로드맵 2단계)은 실물에서 동작 확인 완료.

### 참고: 새로운 위치에서도 "Unable to sample any valid states for goal tree" 재현 (2026-07-24, 같은 세션)

Pointcloud 필터링 검증 중 `coord_to_goal_node` + YOLO 실시간 검출로
실제 이동을 시도했을 때, 목표(g_base 약 [0.219, 0.054, 0.311], 접근 위치
[0.139, 0.054, 0.311], 동적 orientation [0.392, 0.502, 0.608, 0.474])에서
매번 동일하게 `move_group` 로그 `Unable to sample any valid states for goal
tree` → `ParallelPlan::solve()` 3초 예산 소진 → `Planner 'OMPL' failed with
error code FAILURE`로 플래닝 자체가 실패함(`coord_to_goal_node`가 찍는
`STATUS_ABORTED`는 이 플래닝 실패가 액션 레벨에서 뭉뚱그려진 표시일 뿐,
실행 중 충돌로 중단된 게 아니었음 — 처음엔 그리퍼 근접 잔여 voxel 문제로
오판했다가 move_group 로그 확인 후 정정함).

**이번 세션의 pointcloud 필터링과는 무관** — 위 "목표를 바라보는 orientation
동적 계산" 절의 (0.125,-0.15,0.204) 마진널 실패(같은 증상, orientation
선택과 무관하게 실패)와 같은 부류의 문제로 보임. 다음 세션 후속 조사
대상(로드맵 3번, 급하지 않음)에 이 좌표도 추가 사례로 포함. 사용자 판단으로
이번 세션은 여기서 마무리하고 이 이슈는 파고들지 않기로 함.

## 상태 (2026-07-24 기준)

✅ 그리퍼 메시 스케일 버그 수정 — `gripper_base - target_object` 등
월드 오브젝트 충돌 문제 해결됨(위 항목 참고).
✅ look pose 확정 + 실물 동기화 절차 확립, 5회 반복 왕복(시뮬레이션+실물)
성공 확인.
✅ orientation 동적 계산(방식 B) 구현 + 실물 end-to-end 검증 완료 — 그리퍼
정면축(flange 로컬 +Z) 확정, look-at 함수 작성, `coord_to_goal_node.py`
적용, 플래닝 예산 확대(`allowed_planning_time=3.0s`,
`num_planning_attempts=10`)까지 반영. 과거 실패 좌표 중 최소 하나가 확실히
개선됨을 실물 스택에서 확인(위 "실물 end-to-end 검증 결과" 참고). 다른
좌표의 마진널 실패는 orientation 계산과 무관해 보이나 근본 원인 후속 조사
필요.
⬜ 그리퍼 근접거리 self-filter 튜닝(`padding_scale: 0.92`/`padding_offset: 0`)은
실용적 타협 수준 — 아래 pointcloud 사전 필터링 실물 검증 이후 재평가 예정.
✅ **Pointcloud 사전 필터링(로드맵 2단계) 구현 + 실물 검증 완료
(2026-07-24)** — YOLO bbox로 토마토 영역을 depth 단계에서 사전 제거하는
`pointcloud_tomato_filter_node` 신규 작성 + `yolo_d435_detector_node` 확장
(bbox 전체 발행 + `tomato_detections_image` 디버그 시각화) +
`sensors_3d.yaml`/`demo_octomap.launch.py` 연동. 개발 중 발견+수정한 버그
2개: (1) 1차 구현이 realsense raw pointcloud(depth 센서 고유 그리드)를 직접
마스킹해서 컬러 bbox와 그리드가 안 맞던 문제 → `aligned_depth_to_color`
기반 자체 디프로젝션으로 재작성, (2) 출력 topic QoS가 BEST_EFFORT라
occupancy_map_monitor(RELIABLE 요구)에 메시지가 전달 안 되던 문제 → 기본
QoS로 변경(위 "raw pointcloud 그리드 불일치 버그", "QoS 불일치 버그" 절
참고). 두 수정 후 `bbox_padding_ratio`를 키워가며 실물 D435 + RViz로
**토마토 위치에 Octomap 구멍이 실제로 생기는 것을 육안 확인함** — 로드맵
2단계 완료. 그리퍼 근접 self-filter 재평가(로드맵 우선순위 2번)는 다음
세션 진행 대상.
⬜ 새 좌표(g_base 약 [0.219,0.054,0.311])에서도 "Unable to sample any valid
states for goal tree" 플래닝 실패 재현됨 — (0.125,-0.15,0.204)와 같은 부류의
기존 마진널 IK 미해결 이슈(로드맵 3번, 급하지 않음)에 사례 추가, 이번
세션엔 조사 안 하기로 함(위 "새로운 위치에서도 ... 재현" 절 참고).
