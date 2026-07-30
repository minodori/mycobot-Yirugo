# Octomap 장애물 회피 수동 테스트 절차 (2026-07-23)

`demo_octomap.launch.py` + `coord_to_goal_node`로, look pose에 고정된 실물
카메라가 보는 장애물을 MoveIt2가 실제로 피해서 목표 좌표까지 플래닝하는지
수동으로 확인하는 절차. `docs/look_pose.md`, `docs/handeye_calibration.md`와
이어지는 내용.

## 다음 세션 시작 안내 (2026-07-24 두 번째 세션 종료 시점)

### 이번 세션(두 번째)에 완료된 것
- **`Unable to sample any valid states for goal tree` 근본 원인 규명 + 수정**
  (g_base [0.219,0.054,0.311] 재현 케이스) — 아래 "PointStamped 플래닝 실패
  근본 원인 규명 + 수정" 절 참고. 핵심: roll(WORLD_UP 기준) 문제가 아니라
  `target_object` 클리어런스 구(반경 5cm)가 그리퍼 자신의 접근 자세와 항상
  충돌하던 것 — `compute_ik`/`check_state_validity`로 직접 증명함. roll 후보
  탐색 + ACM(Allowed Collision Matrix) 기반 그리퍼<->target_object 충돌 허용을
  구현. 같은 좌표로 4회 재검증 — 플래닝 4/4 성공(기존엔 100% 실패), 실행까지
  3/4 완료(1회는 아래 "그리퍼 근접거리 self-filter" 항목의 잔여 voxel로 실행 중
  abort — 별개의 기존 이슈, 오늘 고친 것과 무관).
- **`_on_target_point`의 기존 race condition 발견 + 수정**: YOLO
  실시간 검출(`yolo_d435_detector_node`)이 `/target_point`를 프레임마다 계속
  재발행하는 상황에서, ReentrantCallbackGroup 때문에 `_busy` 체크-후-설정이
  원자적이지 않아 `self._clear_timer`가 두 번 생성되고 노드가 `NoneType.cancel()`로
  죽는 크래시를 실물로 재현함. 구독 콜백만 별도 `MutuallyExclusiveCallbackGroup`으로
  분리해 해결.
- **(사고 기록) ACM 부분 diff 발행 사고 + 복구**: 원인 조사 중 ACM을 부분적으로만
  발행했다가 SRDF 기반 self-collision-disable 항목이 전부 날아가는 사고가 있었음
  (PlanningScene diff로 발행하는 ACM은 병합이 아니라 통째로 대체됨을 실측 확인).
  `move_group` 재시작으로 정상 복구함 — `coord_to_goal_node.py`의 최종 구현은
  항상 `/get_planning_scene`으로 현재 전체 ACM을 먼저 조회해 보존한 채로만
  수정하도록 작성함(재발 방지).
- **실물 sync_plan 연동 확인 + 수확 순차 처리(harvest_sequence_node) 구현**:
  사용자가 직접 `sync_plan`을 켠 상태에서 위 수정된 `coord_to_goal_node`로
  같은 좌표에 실물 이동 성공 확인(시뮬레이션과 실물 일치). 이어서 "YOLO가
  연속으로 검출/송출하는 상황에서 ripe/disease 토마토를 깊이(z) 가까운
  순서로 순차 수확 접근"을 위해 `harvest_sequence_node`를 신규 구현 — 아래
  "수확 순차 처리" 절 참고. 그리퍼 actuation(실제로 쥐고 따는 동작)은 이번
  세션 범위에서 제외(다음 세션, 사용자 확인).
- ⚠️ **사고: `coord_to_goal_node`의 look pose 자동 복귀 기능이 YOLO
  연속 스트림과 맞물려 무한 루프를 일으킴 + 실물 최종 위치 불확실** — 아래
  "look pose 자동 복귀 + 실물 자동 루프 사고" 절 참고. **다음 세션 시작 전
  반드시 실물 팔의 실제 현재 위치를 육안으로 확인할 것** — look pose가
  아닐 수 있음(위 "다음 세션에 할 일" 1번 참고).

### 이전 세션(2026-07-24 첫 번째)에 완료된 것
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

### 다음 세션에 할 일 — 우선순위 (2026-07-24 두 번째 세션 종료 시점 갱신)
0. ⚠️ **(가장 먼저) 실물 팔의 실제 현재 위치를 육안으로 확인할 것** — 아래
   "look pose 자동 복귀 + 실물 자동 루프 사고" 절 참고. 세션 종료 시점
   시뮬레이션(FakeSystem)의 `/joint_states`는 look pose 값으로 보였지만,
   그 이후 벌어진 무한 루프의 마지막 사이클이 깔끔하게 끝났는지 확인 안 된
   채 세션이 끝남 — **실물이 look pose가 아닌 "이상한 위치"에 멈춰있을 수
   있음**(사용자 보고). 토크 걸린 채면 무리한 위치라도 버티고 있겠지만,
   다음 세션 시작 전 반드시:
   1. 실물 팔을 육안으로 확인(주변 충돌 위험 있는 자세인지).
   2. `docs/look_pose.md` 1번 순서대로 다시 look pose로 `send_angles` 보내
      맞출 것(시뮬레이션 값을 믿지 말고 실물 기준으로).
   3. `initial_positions.yaml`도 그 값과 일치하는지 재확인 후 시작.
1. **그리퍼 근접거리 self-filter 재평가** (로드맵 2번, 아직 미착수): pointcloud
   사전 필터링이 실물 검증됐으니, `sensors_3d.yaml`의 `padding_scale: 0.92`
   타협이 지금도 필요한지, 잔여 voxel 문제가 줄었는지 재확인. 이번 세션
   재검증(아래 "PointStamped 플래닝 실패 근본 원인 규명" 절)에서 4회 중 1회
   여전히 이 잔여 voxel로 실행 중 abort가 재현됨 — 여전히 살아있는 이슈.
2. **(0.125,-0.15,0.204) 좌표를 새 코드(roll 탐색 + ACM 수정)로 재검증**: 이번
   세션에 g_base [0.219,0.054,0.311]의 `Unable to sample any valid states for
   goal tree`는 근본 원인을 찾아 수정했지만(아래 절 참고), 지난 세션에 같은
   증상을 보였던 다른 좌표(0.125,-0.15,0.204)는 아직 새 코드로 재테스트 안 함
   — 원인이 정말 같았는지(target_object 충돌) 확인 필요.
3. `bbox_padding_ratio` 최종값 확정 — 세션 종료 시점 `DEFAULT_BBOX_PADDING_RATIO
   = 0.5`(실물 조정 중, 0.2 기본값은 파일에 주석으로 남겨둠). 더 튜닝하거나
   확정할 것.
4. **`harvest_sequence_node` 실물 end-to-end 검증** (이번 세션 신규 구현,
   미검증): 실제 D435로 여러 ripe/disease 토마토가 보이는 상황에서
   `/start_harvest_sequence` 호출 -> 큐가 깊이 가까운 순서로 정렬되는지 ->
   `sync_plan` 켠 상태에서 순차로 실물이 이동하는지 확인. 위 "수확 순차
   처리" 절 참고.
5. 이후 원래 로드맵(카메라 감지 → 좌표 계산 → 장애물 회피 플래닝 → 실물
   이동 전체 파이프라인 완성) 계속 진행, 완성되면 automato_ws로 포팅
   (사용자 확인된 방침, 다시 묻지 말 것).

### 참고
- 커밋은 요청 시에만.
- RPi `sync_plan`은 실물 이동 테스트 아닐 땐 꺼둔 채로 두는 게 안전. 세션
  종료 시점 RPi에서 `pgrep -af sync_plan`으로 확인했을 때 프로세스가 없었음
  (꺼진 상태로 추정) — 단, 아래 사고 절 참고: 정상 종료했는지 크래시로
  죽었는지는 확인 안 됨.
- **[Tier4, so101-ros-physical-ai 교훈 이식] 실물 테스트 시작 전, 세션 종료
  시점뿐 아니라 매번 먼저 확인할 것**: so101(자매 프로젝트)에서 이미 내려간
  줄 알았던 스택이 실제로는 살아서 같은 실물 포트를 물고 있어 두 스택이
  충돌한 사고가 있었음(`fuser`로 포트 점유 프로세스를 추적해서야 발견함).
  mycobot은 실물 명령이 이 워크스페이스의 ros2_control이 아니라 RPi
  (jetcobot_126b)의 `sync_plan`을 거쳐 나가므로, 로컬 `ps aux`(아래 3번
  섹션)만으로는 이 리스크를 못 잡음 — RPi 쪽도 같이 확인해야 함:
  ```bash
  ssh jetcobot_126b 'pgrep -af "sync_plan|MyCobot280"; fuser /dev/ttyUSB0 2>&1'
  ```
  `fuser`가 뭔가를 출력하면(포트 점유 중) 어떤 프로세스인지 확인 후 의도한
  것이 아니면 정리하고 시작할 것 — 특히 `sync_plan`을 새로 띄우기 전에
  이전 세션의 `sync_plan`이 안 죽고 남아있으면 두 프로세스가 같은 실물에
  동시에 명령을 보내는 구조적 위험이 있음(so101에서 mock/real 스택이 같은
  네임스페이스를 공유해 실제로 겪은 사고와 같은 클래스의 문제).

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

## PointStamped 플래닝 실패 근본 원인 규명 + 수정 (2026-07-24, 두 번째 세션)

**배경**: 위 "새로운 위치에서도 ... 재현" 절에서 미뤄뒀던 g_base
[0.219,0.054,0.311] 좌표(접근 위치 [0.139,0.054,0.311], 동적 orientation
[0.392,0.502,0.608,0.474])의 `Unable to sample any valid states for goal
tree`를 이번 세션에 실제로 재현하며 파고듦.

**1차 가설(틀림): WORLD_UP 기준 roll이 IK 데드존에 걸린다** — `/compute_ik`
서비스로 같은 forward 방향에 대해 roll을 24단계(15도 간격)로 스윕한 결과,
`avoid_collisions=False`로는 11/24 각도에서 IK가 풀렸는데 `coord_to_goal_node`가
실제로 쓰던 roll(=WORLD_UP 기준 단일값)은 그 안 풀리는 13개 중 하나였음. 이
결과만 보고 "roll 선택이 문제"라고 처음엔 결론 내림.

**2차 검증(진짜 원인 발견): `/check_state_validity`로 충돌 주체를 직접 확인**
— roll을 바꿔가며 `avoid_collisions=True`로 재검증했더니 **12개 후보 전부
실패**함. `avoid_collisions=False`로 찾은 IK 해에 `check_state_validity`를
돌려보니 **모든 roll에서 예외 없이 `gripper_base`/`gripper_left1~3`/
`gripper_right1~3`이 `target_object`(우리가 목표 지점에 등록해둔 반경 5cm
클리어런스 구)와 충돌**하고 있었음(깊이 최대 ~2.9cm). 즉 roll 선택의 문제가
아니라, **`APPROACH_OFFSET_X`(8cm) − `TARGET_OBJECT_RADIUS`(5cm) = 3cm인데
그리퍼 손가락이 flange에서 실제로 ~5.5cm 뻗어나가 있어서(look-at이 정확히
목표를 겨냥할수록) 손가락이 항상 그 클리어런스 구를 뚫고 들어가는 구조적
문제**였음. `APPROACH_OFFSET_X`를 늘리거나(0.10~0.18) forward축 기준으로
당겨도 이 근처 좌표는 오히려 통째로 IK 불가 지점이 돼버려(팔 베이스에
너무 가까워짐) 해결이 안 됐고, `TARGET_OBJECT_RADIUS`를 실제 토마토
반지름(2cm)보다도 작은 1.5cm까지 줄여야 겨우 일부 roll이 풀렸음(Octomap
노이즈 클리어런스 여유를 포기해야 해서 채택 안 함).

**최종 수정**: 그리퍼가 자기가 접근하려는 목표 지점 자체(`target_object`)와
겹치는 건 오히려 당연한 것(집으려면 가까이 가야 함)이라는 판단 하에, **Allowed
Collision Matrix(ACM)로 그리퍼 7개 링크와 `target_object` 사이의 충돌만
명시적으로 허용**함(Octomap이나 다른 world 장애물과의 충돌 검사는 그대로
유지). `coord_to_goal_node.py`에 `_allow_gripper_target_object_collision()`을
추가해 노드 시작 시 1회 실행:
1. `/get_planning_scene`(`ALLOWED_COLLISION_MATRIX` 컴포넌트)으로 현재 전체
   ACM을 조회.
2. `target_object` 행/열을 추가하고 `GRIPPER_LINK_NAMES`(7개, URDF
   `mycobot_280_m5_adaptive_gripper.urdf`의 그리퍼 링크와 동일) 칸만 `True`로
   설정.
3. `PlanningScene(is_diff=True)`로 재발행.

⚠️ **사고 + 교훈**: 처음에 이 조사를 하며 실험적으로 `target_object`와
그리퍼 7개 링크만 담은 **부분** ACM을 발행했다가, 기존 SRDF 기반
self-collision-disable 항목(arm 8개 링크)이 전부 사라지는 사고가 남 —
**PlanningScene diff로 발행하는 ACM은 기존 매트릭스와 병합되는 게 아니라
발행한 내용으로 통째로 대체됨**을 직접 확인함(`/get_planning_scene`으로
전/후 비교). `move_group` 재시작으로 SRDF 원본 ACM을 다시 로드해 복구함.
그래서 최종 구현은 반드시 전체 ACM을 먼저 조회해 기존 항목을 보존한 채로만
수정하도록 작성함(위 절차의 1번). **향후 ACM을 다시 건드릴 일이 있으면 이
방식(조회 → 보존 → 추가 → 재발행)을 반드시 따를 것 — 부분 diff 절대 금지.**

**roll 후보 탐색도 별도로 구현해 유지함**: 근본 원인은 ACM이었지만, 1차
가설(roll 데드존)도 100% 틀린 건 아님 — 같은 forward에서도 특정 roll만
IK 자체가 안 풀리는 경우가 실제로 있어서(6축 팔 특성), `_compute_look_at_quat_xyzw()`에
`roll_rad` 파라미터를 추가하고 `ROLL_CANDIDATES_RAD`(30도 간격 12개)를
`/compute_ik`로 순차 사전체크해 실제로 풀리는 첫 후보를 채택하도록 함.
KDL IK가 같은 조건에서도 매 호출 성공률이 편차 있는 확률적 솔버임을 실측
확인해(같은 조건 10회 중 7~9회 성공) 후보당 `ROLL_CANDIDATE_IK_RETRIES=3`번
재시도하도록 함.

**부수 발견: `_on_target_point`의 기존 race condition** — 재검증 중
`yolo_d435_detector_node`가 프레임마다 `/target_point`를 계속 재발행하는
상황에서 노드가 `AttributeError: 'NoneType' object has no attribute 'cancel'`로
죽는 크래시가 실물로 재현됨. `ReentrantCallbackGroup` 때문에 `_on_target_point`
콜백 자신이 동시에 여러 스레드에서 실행될 수 있어 `_busy` 체크-후-설정이
원자적이지 않았던 게 원인(이 버그는 오늘 만든 게 아니라 원래 있던 것 —
그동안 `ros2 topic pub --once`로만 테스트해서 안 드러났다가, YOLO 실시간
스트림과 맞물리며 처음 발현됨). 구독 콜백만 별도
`MutuallyExclusiveCallbackGroup`으로 분리해 자기 자신과는 절대 동시 실행되지
않도록 수정(다른 콜백들은 기존 `ReentrantCallbackGroup` 유지).

**검증**: 같은 좌표(g_base [0.219,0.054,0.311])로 수정 후 4회 연속 테스트
— **플래닝 4/4 성공**(수정 전엔 100% `Unable to sample any valid states for
goal tree`로 실패). 실행까지는 3/4 완료, 1회는 그리퍼 근접 잔여 octomap
voxel(`<octomap>` <-> `gripper_right1`)로 실행 중간에 abort — 이건 위
"그리퍼 근접거리 self-filter" 항목(`padding_scale: 0.92` 타협)에 해당하는
별개의 기존 이슈로, 오늘 고친 문제와 무관함(다음 세션 우선순위 1번으로
재평가 예정).

## 수확 순차 처리 (harvest_sequence_node, 2026-07-24 두 번째 세션)

**배경**: 위 근본 원인 수정 이후 사용자가 직접 `sync_plan`을 켜고 같은 좌표로
실물 이동까지 성공 확인함. 다음 질문은 "YOLO가 검출한 토마토를 실제로 바로
수확하려면?" — 두 가지로 나눠 범위를 정함:
1. 그리퍼 actuation(실제로 쥐고 따는 동작)은 **이번 세션 범위 밖**(다른
   세션에서 진행, 사용자 확인). 지금은 "접근"까지만 자동화됨.
2. YOLO는 카메라가 살아있는 한 프레임마다 계속 검출을 재발행함(연속
   스트림) — 이 상황에서 어떻게 여러 토마토를 순서대로 처리할지가 이번
   세션에 다룬 부분. 사용자 요구사항: **look pose에서 촬영한 스냅샷 기준으로,
   z(카메라로부터의 깊이)가 가장 낮은(가까운) 것부터 ripe/disease를 순차
   처리**.

**왜 "매 프레임 반응"이 아니라 "스냅샷 한 번"이어야 하는가**: 카메라가
eye-in-hand라, 팔이 목표로 접근하기 시작하면 카메라 시점이 바뀌어 (1) 같은
물체의 좌표가 흔들리거나 (2) 팔 자신이 시야를 가려 완전히 다른 검출로
바뀔 수 있음. 그래서 순차 처리 목록은 **look pose에 있을 때 한 번만
캡처**하고, 그 목록을 g_base 좌표로 즉시 변환해 고정한 뒤, 그 이후로는
새로 들어오는 검출 스트림과 무관하게 큐만 갖고 끝까지 진행함.

**구현** (`mycobot_280_pick` 패키지):
1. `yolo_d435_detector_node.py`: 기존 `target_point`(ripe 중 confidence
   1등 하나만) 로직은 그대로 두고, 새 토픽 `tomato_candidates`
   (`std_msgs/msg/Float32MultiArray`)를 추가 — 이번 추론 결과에서
   `HARVEST_CLASS_NAMES = ('ripe', 'disease')` 클래스에 해당하는 검출
   **전부**를 `[class_id, x, y, z, confidence, ...]`로 평탄화해 발행(카메라
   프레임 기준 3D 좌표, 기존 픽셀->3D 변환식 재사용, 추가 추론 없음).
2. `coord_to_goal_node.py`: `plan_result`(`std_msgs/msg/Bool`) 토픽 추가 —
   `_check_motion_complete`가 플래닝/실행 성공 여부를 판단할 때마다 같이
   발행함. 상위 시퀀서가 "이 목표가 끝났으니 다음 목표를 보내도 되는지"
   판단하는 신호로 씀.
3. **신규** `harvest_sequence_node.py`: `/start_harvest_sequence`
   (`std_srvs/srv/Trigger`) 서비스 호출 시점에:
   - 그 순간의 최신 `tomato_candidates`를 스냅샷.
   - TF(`g_base` <- 카메라 프레임)로 즉시 전부 변환(호출 시점 1회 조회 —
     이후 팔이 움직여도 이 변환을 다시 안 함).
   - **원본 카메라 프레임 z(깊이) 오름차순 정렬**(g_base로 변환한 뒤의 z는
     높이라 의미가 다르므로, 정렬은 변환 전 z 기준).
   - 큐에 저장 후 첫 목표를 `/target_point`(`g_base` 기준)로 발행.
   - `plan_result`가 들어올 때마다(성공/실패 무관) `NEXT_TARGET_DELAY_SEC`
     (1.5초, `coord_to_goal_node`의 옥토맵 클리어 지연보다 여유 있게) 대기
     후 큐의 다음 목표를 발행. 큐가 비면 "수확 시퀀스 완료" 로그.
   - 진행 중 재호출은 거부(`이미 수확 시퀀스 진행 중`), 후보가 비어있거나
     TF 실패 시에도 명확한 실패 메시지로 즉시 응답.

**검증**: 합성 데이터(가짜 `tomato_candidates` 3개, 서로 다른 깊이)로
서비스 시작 시 후보가 없을 때 정상적으로 실패 응답하는 것 확인. 노드 기동/
서비스 등록(`/start_harvest_sequence`) 확인. **실제 YOLO 검출 + 실물 로봇
이동으로 큐 정렬/순차 접근까지 끝까지 도는 end-to-end 테스트는 아직 안 함**
— `coord_to_goal_node`가 `sync_plan`과 함께 떠 있는 상태에서 가짜 좌표로
트리거하면 실물이 임의 위치로 움직이는 위험이 있어 이번 세션엔 보류함
(다음 세션 실물 재검증 대상).

**미검증 (다음 세션 실물 재확인 필요)**:
- 실제 D435로 여러 토마토(ripe/disease 섞어서)가 동시에 보이는 상황에서
  `/start_harvest_sequence` 호출 -> 큐 정렬 순서(가까운 것부터) -> 순차
  실물 이동까지 전체 흐름 확인.
- 접근 도중 카메라 시야에서 다음 대상이 아예 안 보이게 되는 경우(팔이
  가려서) 실제로 문제가 안 되는지(스냅샷 고정 방식이라 이론상 문제
  없어야 하나 실물로 미확인).

## ⚠️ 사고: look pose 자동 복귀 + 실물 자동 루프 (2026-07-24, 두 번째 세션 종료 직전)

**배경**: 사용자가 "목표점 도달 후 1~2초 대기하고 look pose로 돌아와야
한다"고 요청 — 다음 토마토를 보려면 카메라가 넓은 시야(look pose)로
돌아와야 하기 때문. `coord_to_goal_node`에 `RETURN_TO_LOOK_POSE_DELAY_SEC`
(1.5초 대기) + `move_to_configuration(LOOK_POSE_JOINT_POSITIONS)`로 목표
도달 후 자동 복귀하는 로직을 추가함(`_check_motion_complete` ->
`_start_return_to_look_pose` -> `_check_return_complete` -> `plan_result`
발행 순서로 재구성).

**문제**: 이 기능을 테스트하려고 `yolo_d435_detector_node`(연속 검출/발행)와
`coord_to_goal_node`를 둘 다 띄운 상태로 뒀는데, **둘을 동시에 켜두면
사용자가 명시적으로 트리거하지 않아도 무한 루프가 자동으로 돎**:
1. `yolo_d435_detector_node`가 `ripe` 검출 시 `/target_point`를 자동 발행
   (기존부터 있던 동작, 1Hz).
2. `coord_to_goal_node`가 그 좌표로 자동 접근.
3. 새로 추가한 로직이 도착 후 자동으로 look pose 복귀.
4. look pose에서 카메라가 다시 같은(또는 다른) `ripe` 토마토를 보고 →
   `yolo_d435_detector_node`가 또 `/target_point`를 발행 → 2번부터 반복.

로그로 실제 여러 사이클이 자동으로 돈 것을 확인함(목표 좌표가 매번
달라짐: (0.167,0.053,0.187) → (0.117,0.013,0.402) → (-0.025,0.060,0.249) →
...). `harvest_sequence_node`가 의도한 "명시적 트리거 + 스냅샷 큐" 방식과
전혀 다른, **의도하지 않은 자동 반복 이동**이 실물에서 발생한 것 — 이건
`yolo_d435_detector_node`가 계속 켜져 있는 한 `coord_to_goal_node` 혼자서도
(harvest_sequence_node 없이도) 벌어지는 문제임.

**조치**: 발견 즉시 `yolo_d435_detector_node`를 강제 종료해 `/target_point`
발행원을 끊어 루프를 멈춤. 그 직후 확인한 `/joint_states`는 look pose
값이었으나, **이후 세션 정리(모든 로컬 프로세스 kill) 시점에 사용자가
"목표 도착 → look pose 복귀 → 다시 이상한 위치로 가서 멈췄다"고 정정함**
— 즉 내가 확인한 시점 이후에도 최소 한 사이클이 더 돌았고, 그 마지막
사이클이 깔끔하게 완료되지 않은 채(또는 완료됐지만 다음 목표로 또 넘어간
채) 프로세스가 죽어서 **실물의 최종 정지 위치가 불확실함**. 세션 종료
시점 로봇/카메라 모두 연결 해제된 상태라 원격으로 재확인 불가.

**교훈 + 재발 방지**:
1. **`yolo_d435_detector_node`(연속 자동 발행)와 `coord_to_goal_node`를
   동시에 띄우는 것 자체가 "자동 추적 모드"임을 명확히 인지할 것** —
   테스트/디버깅 목적으로 잠깐 띄우더라도 무한 루프가 될 수 있음을 미리
   예상하고, 짧게 확인 후 바로 끄거나 애초에 `target_point` 구독을 끊어둘
   것.
2. **자동 복귀 같은 "연쇄 동작" 기능을 추가할 땐, 그 기능이 기존의 다른
   자동 트리거(YOLO 연속 발행)와 결합했을 때 루프를 만들 수 있는지 미리
   따져볼 것** — 각 기능은 개별적으로 안전해 보여도 조합하면 무한 루프가
   될 수 있음.
3. **실물이 얽힌 세션을 정리할 때, "안전한 상태로 끝났다"고 보고하기 전에
   반드시 정지 직후 최신 상태를 재확인할 것** — 중간에 확인한 스냅샷을
   최종 상태로 오인하면 안 됨(이번에 내가 한 실수).
4. `harvest_sequence_node`를 실제로 쓸 때도 같은 위험이 있음 — 시퀀스가
   끝난 뒤(`_publish_next_target`에서 큐가 비어 "완료" 로그가 찍힌 뒤)에도
   `yolo_d435_detector_node`가 계속 켜져 있으면, `coord_to_goal_node`가
   그 뒤로도 계속 새 `/target_point`에 반응해서 똑같이 자동 루프가 될 수
   있음(harvest_sequence_node의 큐 로직과 무관하게, `coord_to_goal_node`는
   `/target_point`를 받으면 무조건 반응하는 구조라서). **다음 세션에 고려할
   구조 개선**: 수확 시퀀스 진행 중이 아닐 때는 `yolo_d435_detector_node`의
   `/target_point` 발행을 끄거나(파라미터화), `coord_to_goal_node`가
   `harvest_sequence_node`가 관리하는 세션 중에만 `/target_point`를
   받도록 하는 등의 게이팅 필요.

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
✅ **g_base [0.219,0.054,0.311] "Unable to sample any valid states for goal
tree" 근본 원인 규명 + 수정 완료 (2026-07-24, 두 번째 세션)** — 진짜 원인은
roll 선택이 아니라 목표 지점에 등록하는 `target_object` 클리어런스 구(5cm)가
그리퍼 자신의 접근 자세와 항상 충돌하던 것(`compute_ik`+`check_state_validity`로
직접 증명, 위 "PointStamped 플래닝 실패 근본 원인 규명 + 수정" 절 참고).
ACM으로 그리퍼<->target_object 충돌 허용 + roll 후보 탐색(보조)으로 수정,
같은 좌표 4회 재검증 플래닝 4/4 성공. 부수적으로 YOLO 고빈도 발행 시
`_on_target_point`가 죽던 기존 race condition도 발견+수정.
⬜ (0.125,-0.15,0.204)는 지난 세션에 같은 증상을 보였던 별도 좌표 — 아직 새
코드로 재검증 안 함, 다음 세션 우선순위 2번.
🔶 **수확 순차 처리(harvest_sequence_node) 구현 완료, 실물 end-to-end 미검증
(2026-07-24, 두 번째 세션)** — 위 "수확 순차 처리" 절 참고. 로직/서비스
등록만 스모크테스트, 실제 다중 토마토 순차 접근은 다음 세션 대상.
⚠️ **목표 도달 후 look pose 자동 복귀 기능 추가 + 실물 자동 루프 사고
(2026-07-24, 두 번째 세션 종료 직전)** — 위 "사고: look pose 자동 복귀 +
실물 자동 루프" 절 참고. `yolo_d435_detector_node` + `coord_to_goal_node`를
동시에 켜두면 자동 반복 이동이 발생함(harvest_sequence_node 없이도 재현).
**세션 종료 시점 실물의 정확한 최종 위치 불확실 — 다음 세션 시작 전
최우선으로 실물 육안 확인 + look pose 재동기화 필요**(위 "다음 세션에 할
일" 0번 참고).

## [Tier5] 자매 프로젝트(so101-ros-physical-ai) 교훈 — 향후 주의사항 (코드 변경 없음, 기록만)

so101-ros-physical-ai(SO-101 5축 팔 + D435 + YOLO, 같은 문제 도메인)가
2026-07-26~27 실물 테스트에서 겪은 사고 중, mycobot이 아직 그 단계에
도달하지 않아 지금 당장은 코드로 옮길 필요 없지만 **나중에 같은 실수를
반복하지 않도록 미리 기록해두는 두 가지**.

### 1. 그리퍼 actuation 구현 시 "그리퍼 열기"를 반드시 명시적 첫 단계로 넣을 것

현재 `coord_to_goal_node`는 실제 그리퍼 개폐(actuation)를 구현하지 않고
목표보다 `APPROACH_OFFSET_X`만큼 당긴 위치로 flange가 접근하는 것으로만
대체하고 있음(모듈 docstring 참고) — URDF에는 이미
`mycobot_280_m5_adaptive_gripper`가 있고 `gripper_controller` 조인트가
ros2_control에도 등록돼 있어서, 실제 파지 로직을 붙이는 것 자체는 남은
로드맵 항목일 뿐 하드웨어가 없는 게 아님.

so101에서는 파지 실패를 여러 세션에 걸쳐 반복 조사했는데, 최종 원인이
"정렬→접근→파지→후퇴" 4단계 어디에도 그리퍼를 **여는** 단계 자체가 없어서
접근/파지 내내 그리퍼가 거의 닫힌 값(-0.16rad)이었던 것으로 밝혀짐 — 위치
정확도 문제와는 무관하게 이 버그 하나만으로 모든 파지가 실패할 수밖에
없었음. mycobot도 실제 파지 로직을 구현할 때 같은 함정에 빠지기 쉬움(그리퍼
제어 코드를 추가했다고 "당연히 열려있겠지"라고 가정하기 쉬움) — 구현 시
**정렬 → 그리퍼 열기(명시적 단계) → 접근 → 파지(닫기) → 후퇴** 순서를
처음부터 의식적으로 넣을 것, 그리퍼 열기 단계를 빠뜨리지 않았는지 실물
검증 시 `/joint_states`의 그리퍼 조인트 값을 직접 확인할 것.

### 2. 파지 후 후퇴 궤적이 관절공간 플랜이라 직선을 보장 안 함 — so101도 미해결인 공유 리스크

so101은 파지 후 "복귀"가 (반지름만 다른) 두 관절 상태 사이의 OMPL
관절공간 플랜이었는데, 실제 경로는 직선/일정 높이를 전혀 보장하지 않아서
"잡고 바로 위로 솟았다 내려오는" 예상 밖 모션이 됐음(사용자가 실물로
관찰). mycobot의 `_start_return_to_look_pose`도 동일하게
`move_to_configuration(LOOK_POSE_JOINT_POSITIONS, ...)`(관절공간 플랜)이라
구조적으로 같은 리스크가 있음 — 지금은 그리퍼 actuation이 없어서 아직
체감되지 않았을 뿐, 실제 파지가 붙으면 같은 증상이 나올 수 있음.
`RETURN_TO_LOOK_POSE_DELAY_SEC`(1.5초 대기)는 이미 있어서 so101이 나중에
추가한 "파지 후 1초 대기"에 해당하는 부분은 커버돼 있음 — 부족한 건 경로
자체의 직선성. so101도 아직 완성된 해법이 없음(Cartesian 경로 제약, 중간
waypoint 강제 등을 다음 과제로 남긴 상태) — **so101 쪽에서 검증된 해법이
나오면 그때 참고할 것, 지금 mycobot에서 먼저 재설계할 필요는 없음**(그리퍼
actuation이 아직 없어 실익이 낮음).

## 2026-07-27 그리퍼 actuation 1단계 — 메커니즘 검증 완료, "직진 접근" reachability는 미해결 (다음 세션 최우선)

**결론부터**: Tier5에서 "향후 과제"로만 남겨뒀던 실제 그리퍼 파지(actuation)
구현을 시작함. `coord_to_goal_node`를 정렬→**그리퍼 열기**→직진 접근→
**파지(닫기)**→후퇴 5단계로 재구성(so101 교훈 그대로 "그리퍼 열기"를
명시적 단계로 추가). 인프라(SRDF/컨트롤러/자기충돌)를 여러 번 고친 끝에
**정렬(1/5)·그리퍼 열기(2/5)는 mock 스택에서 안정적으로 성공** 확인함.
**직진 접근(3/5)은 아직 미해결** — 다음 세션 최우선 조사 대상.

### 그리퍼 인프라 구축 (전부 실물 파지 최초 시도라 처음 겪는 문제들)

SRDF에 `gripper`/`gripper_open`/`gripper_closed` 그룹만 추가하면 될 줄
알았는데, 그리퍼 조인트를 MoveIt으로 실제로 움직여본 게 이번이 처음이라
연쇄적으로 인프라 공백이 드러남:

1. **`gripper_group_controller` 자체가 없었음** — `ros2_controllers.yaml`/
   `moveit_controllers.yaml`에 `arm_group_controller`만 있고 그리퍼용
   컨트롤러가 아예 없어서, MoveIt이 "gripper" 그룹 궤적을 실행할 방법이
   없었음. 둘 다 추가(URDF `firefighter.ros2_control.xacro`엔 이미
   `gripper_controller` 조인트의 command/state interface가 있었음 — 이것만
   빠져있었음).
2. **`GRIPPER_OPEN_POSITION`을 URDF 조인트 상한(0.15rad) 그대로 쓰면
   실패** — 정확히 한계값이면 OMPL이 목표 상태를 샘플링할 여유가 없어서
   `Insufficient states in sampleable goal region`으로 매번 실패함. 양 끝에서
   여유를 두고 `0.12`/`-0.6`으로 조정.
3. **`gripper_controller`에 속도/가속 한계가 없었음** — URDF엔
   `velocity="0"`(팔 6관절과 같은 패턴, `joint_limits.yaml`로 채워야 함)인데
   그리퍼는 안 채워져 있어서 `AddTimeOptimalParameterization`이 "No velocity
   limit was defined" 에러로 궤적 생성 자체를 실패함 — `joint_limits.yaml`에
   `gripper_controller`(max_velocity=1.0, max_acceleration=2.0) 추가.
4. **좌우 손가락 교차 자기충돌 미등록** — 그리퍼를 열어보니
   `gripper_left3`<->`gripper_right3`가 충돌로 잡힘. 기존 SRDF
   self-collision 패치(`gripper_left1~3`/`gripper_right1~3`)는 "같은 쪽"
   링크끼리만 다뤄서 좌우 교차 9쌍이 통째로 빠져있었음 — 전부 추가.

### ⚠️ 그리퍼 마운트 180도 방향 오류 발견 + 수정 (사용자 육안 확인)

사용자가 "실물 그리퍼는 체결용 구멍 여러 개 있는 면이 상단인데 RViz는
하단"이라고 지적 — `mycobot_280_m5_adaptive_gripper.urdf`의
`joint6output_to_gripper_base`(flange->gripper_base 고정 조인트) origin을
보니 `rpy="1.579 0 0"`(≈+90.5도)로 돼 있었고, 바로 위에 주석 처리된
이전 값이 `rpy="-1.5708 0 0"`(≈-90도) — 정확히 180도 반대. Roll 부호만
뒤집어(`-1.579`, z 오프셋 0.034는 유지) 수정 → **RViz/실물 방향 일치
확인함(사용자 확인)**. 이 수정이 부수적으로 `joint5`(손목)<->
`gripper_left1` 자기충돌을 새로 드러냈는데(방향이 뒤집혀 있을 때는 우연히
안 겹쳤던 것), **RViz look pose->목표 애니메이션과 실물 둘 다 실제 간섭
없음을 사용자가 확인**해줘서 콜리전 메시 여유분으로 판단, `joint5`도
그리퍼 7개 링크 전부와 충돌 허용으로 SRDF 패치함.

**중요**: 이 마운트 방향 버그는 그리퍼가 실제로 물체를 잡는 자세/각도에
영향을 줄 수 있는 실제 버그였음 — 다음 세션에 그리퍼 실물 파지 테스트할
때 이 수정이 반영된 상태인지(재빌드 여부) 반드시 확인할 것.

### ⚠️ 미해결: "직진 접근"(3/5) reachability — 다음 세션 최우선

정렬(오프셋 위치, APPROACH_OFFSET_X=0.08m 만큼 로봇 쪽으로 당긴 지점)에서
같은 orientation으로 목표 지점 **그 자체**까지 8cm 더 들어가는 마지막
이동이 계속 실패함. 여러 좌표(0.219,0.054,0.311 / 0.30,0,0.25)에서
재현됨 — 특정 좌표만의 문제가 아니라 구조적 문제로 보임.

**시도했으나 효과 없었던 것들**:
1. roll 후보 IK 사전검증 대상을 정렬 위치 대신 목표 지점(더 먼 쪽)으로
   변경 — 그래도 실패.
2. `target_object` 충돌 허용 ACM에 `joint6_flange` 추가(그리퍼 링크만
   있었음) — 콜리전 로그 자체가 안 찍혀서 애초에 원인이 아니었음.
3. 위치 tolerance 완화(0.005m -> 0.02m) — 효과 없음.
4. **Cartesian 경로 계획으로 전환**(`compute_cartesian_path`,
   `plan_async()`+`get_trajectory()`+`execute()`를 직접 조합해 구현 —
   `move_to_pose(cartesian=True)`를 그냥 쓰면 `use_move_group_action`
   설정과 무관하게 내부적으로 blocking `plan()`을 타서 이 노드의
   MultiThreadedExecutor와 충돌함, 그래서 non-blocking 조합으로 직접
   구현함). **결과가 오히려 더 뚜렷한 단서를 줌**: 8cm 이동 중 겨우
   **2.9%(≈2.3mm)만 진행하고 바로 막힘** — 목표 근처의 문제가 아니라
   **정렬 위치의 IK 해 자체가 관절 한계/특이점에 거의 붙어있어서, 그
   자리에서 어느 방향으로든 조금만 움직여도 막히는 자세**라는 뜻으로
   해석됨.

**유력 가설(확정 아님, 다음 세션 조사 대상)**: 정렬 단계가 "목표까지 이어
지는 경로 전체"를 고려하지 않고 정렬 위치+orientation만 독립적으로 풀림
(OMPL RRTConnect가 아무 유효해나 하나 고름, orientation tolerance도
0.5rad로 넉넉해서 후보가 넓음) — 그 결과 관절 여유(manipulability)가
안 좋은 해를 우연히 고르면 그 다음 이동(직진 접근)이 막힘. look pose
자체가 나쁜 자세라는 근거는 없음(look pose는 정렬 단계의 시작점/forward
벡터 계산 기준일 뿐, 정렬 단계 결과 관절해 자체는 OMPL이 사실상
임의로 고름).

### 다음 세션 우선순위

1. **"직진 접근" reachability 문제 해결** (최우선) — 후보 방향:
   - 정렬과 직진 접근을 하나의 플래닝 문제로 묶기(예: 목표 지점을 먼저
     풀고 거기서 거꾸로 정렬 위치까지의 경로가 유효한지 확인, 또는 둘 다
     만족하는 궤적을 한 번에 요청).
   - roll 후보 검증 시 "목표 지점의 IK가 풀리는지"뿐 아니라 "정렬 위치에서
     목표 지점까지 Cartesian 경로가 실제로 이어지는지"까지 사전 확인.
   - orientation tolerance를 더 좁혀서 OMPL이 매번 비슷한(안정적인) 해를
     고르도록 유도해보는 것도 시도해볼 가치 있음.
2. 1번이 해결되면 실제 파지(그리퍼 닫기) + 후퇴까지 mock 전체 파이프라인
   end-to-end 검증.
3. 그 다음 실물로 그리퍼 마운트 방향 수정 반영 확인 + 실제 파지 시도.
4. 오늘 추가한 인프라(gripper_group_controller, joint_limits, 자기충돌
   패치)는 이미 mock에서 검증됨 — 실물에서 다시 볼 필요 없음, 코드/설정
   문제 없었음.

## 2026-07-27 "직진 접근" reachability 진짜 원인 발견 + 수정 (세 번째 세션)

**결론부터**: 위 "미해결: 직진 접근 reachability" 절의 유력 가설(정렬 단계가
OMPL로 임의의 관절해를 골라서 그 해가 나쁘면 다음 이동이 막힌다)은 **틀렸음이
이번 세션에 실측으로 확인됨**. 진짜 원인은 훨씬 단순했음: **`target_object`
충돌 허용 ACM에 `joint5`/`joint6` 링크가 빠져있었던 것**. `/check_state_validity`로
목표 지점 IK 해를 직접 검사해서 발견함. 이 두 링크를 ACM에 추가하는 것만으로
문서에 기록된 두 실패 좌표(g_base [0.219,0.054,0.311], [0.158,0.198,0.152])가
mock 스택에서 매번(또는 최대 1~2회 재시도 내) 5/5 단계 전부 성공으로
전환됨(아래 "검증 결과" 참고). 추가로 안전망 2개(직진 접근 dry-run 재시도,
후퇴 OMPL 폴백)를 넣어 나머지 소수 실패 케이스도 위험하게 방치되지 않도록 함.

### 시도했다가 되돌린 접근: 정렬+직진 접근 IK 시드 결합

유력 가설을 검증하려고 먼저 "목표 지점 IK 해를 시드로 정렬 위치 IK도 같이
확인하고, 성공하면 그 특정 관절해로 정렬을 joint-space 이동시키는" 방식을
구현했음(`compute_ik`의 `robot_state.joint_state`를 시드로 사용). 그런데
`/tmp` 스크래치패드 스크립트로 직접 검증해보니, **`avoid_collisions=True` +
정확한 orientation(tolerance 없음) 조합 자체의 IK 성공률이 이 그리퍼 자세에서는
타임아웃을 1초로 늘려도 사실상 0에 가까움**(목표 지점 단독 IK조차 10회 중
0회, `avoid_collisions=False`로는 40~50% 수준) — 즉 기존 roll 후보 사전
필터(`ROLL_CANDIDATE_IK_TIMEOUT_SEC=0.05`) 자체가 "가끔 맞으면 좋고 아니면
말고" 수준이었지 신뢰 가능한 사전 검증 수단이 아니었음. 여기에 시드 조건까지
얹으면 이미 낮은 성공률이 더 낮아져 거의 항상 exhaustion(12개 roll 후보 x 3
재시도 전부 실패)으로 빠져 사실상 무의미한 지연만 추가함이 확인돼 되돌림
(코드는 원래의 단순한 roll 후보 사전 필터로 복원, `_on_roll_candidate_result`
주석 참고).

### 진짜 원인 발견: `/check_state_validity`로 목표 지점 IK 해 직접 검사

시드 접근을 되돌린 뒤, "OMPL이 목표 지점에서 왜 매번 `Unable to sample any
valid states for goal tree`로 실패하는지"를 `avoid_collisions=False`로 구한
목표 지점 IK 해에 `/check_state_validity`를 직접 돌려서 확인함. 결과:

```
collision: joint5 <-> target_object  depth=-0.0033
collision: joint6 <-> target_object  depth=-0.0007
```

**모든 IK 해에서 예외 없이 이 두 충돌이 재현됨**. 기존 ACM 패치(2026-07-23,
2026-07-27 오전)는 `GRIPPER_LINK_NAMES`(그리퍼 7개 링크) + `END_EFFECTOR_NAME`
(`joint6_flange`)만 `target_object`와 충돌 허용했는데, **flange 바로 위
링크인 `joint5`/`joint6`는 빠져있었음** — 정렬 위치(목표에서
`APPROACH_OFFSET_X`=8cm 뒤로 뺀 곳)에서는 이 링크들이 `TARGET_OBJECT_RADIUS`
=5cm 클리어런스 구 밖에 있어서 안 겹쳤지만, 목표 지점 그 자체(그리퍼가
실제로 파고드는 자리)에서는 flange뿐 아니라 그 위 두 링크까지 구 안으로
들어와서 매번 충돌 판정됐던 것. `avoid_collisions=True`(OMPL이 실제로 쓰는
설정)로는 이 충돌 때문에 목표 지점 근방에 collision-free 상태가 아예
없었으니, "정렬 단계가 나쁜 관절해를 골라서"가 아니라 **애초에 목표
지점이라는 위치 자체가 항상 self-collision 판정이었던 것** — 정렬 단계가
무엇을 고르든 상관없이 실패할 수밖에 없는 구조였음.

**수정**: `WRIST_LINK_NAMES = ['joint5', 'joint6']` 상수 추가, ACM 허용
목록을 `GRIPPER_LINK_NAMES + WRIST_LINK_NAMES + [END_EFFECTOR_NAME]`(10개)로
확장.

### 안전망 1: "직진 접근" 실행 전 dry-run 재시도 (`_verify_grasp_approach_reachable`)

ACM 수정만으로 대부분 해결됐지만, 워크스페이스 경계에 가까운 좌표(예:
g_base (0.30,0,0.25), 반지름 0.39m — `MAX_TARGET_RADIUS_M`=0.45에 근접)는
ACM 수정 후에도 여전히 Cartesian 직진 접근이 26~58% 정도에서 막히는 경우가
남아있음이 확인됨(`/compute_cartesian_path`를 직접 호출해 마지막 유효
waypoint의 관절값을 URDF 한계와 대조해봤더니 어느 관절도 한계 근처가
아니었고, 대신 연속 waypoint 간 관절 이동량이 끝에 갈수록 0에 수렴하는
패턴 — 특이점 근처에서 수치해석 IK 추적이 멈추는 전형적인 신호로 보임,
관절 한계 문제가 아니라 Cartesian 직선 추적 자체의 한계로 추정).

이런 경우를 실행 전에 미리 걸러내기 위해, 정렬(1/5) 완료 직후 **실행하지
않고** 직진 접근 Cartesian 경로가 `CARTESIAN_FRACTION_THRESHOLD`(0.95) 이상
풀리는지 dry-run(`plan_async`만 호출, `execute()` 생략)으로 먼저 확인함.
안 풀리면 정렬을 다시 풀어서(OMPL RRTConnect는 확률적이라 재시도마다 다른
관절해를 고름) 이어지는 해가 나올 때까지 반복(`MAX_ALIGN_RETRIES=5`).
5회를 다 써도 못 찾으면 그 자리에서 안전하게 abort(look pose 복귀) —
실제 실행 중간에 멈추는 것보다 미리 걸러내는 편이 안전함.

### 안전망 2: 후퇴(5/5) Cartesian 실패 시 OMPL 폴백 (`_start_retreat_ompl_fallback`)

후퇴 단계도 같은 종류의 Cartesian 직선 추적 실패가 가끔 재현됨(mock
실측, 46.9% 완료 사례 확인). 접근(3/5)은 실패해도 빈 그리퍼로 그 자리에서
포기(abort)하면 되지만, **후퇴(5/5)는 이미 물체를 쥔 상태**라 그냥
포기하는 것보다 OMPL 자유 경로로라도 정렬 위치까지는 물러나는 게 안전함
(정확한 직선보다 안전한 탈출 우선). Cartesian이 실패하면 OMPL
`move_to_pose`로 폴백하도록 추가.

### 검증 결과 (mock 스택, `demo.launch.py use_rviz:=false`, 카메라 없이
IK/OMPL/Cartesian 로직만 격리 검증 — Octomap 없이도 재현 가능한 문제라 D435
불필요)

- **g_base [0.158, 0.198, 0.152]** (과거 identity만 성공하던 좌표): 3회 연속
  테스트, 매번 5/5 전부 성공(0회 또는 1회 정렬 재시도 내). ACM 수정 전에는
  roll 후보 IK 사전 필터 자체가 매번 exhaustion으로 실패했었음(target IK가
  `avoid_collisions=True`로 전혀 안 풀렸으므로) — 수정 후 "roll 후보 0도에서
  IK 확인됨"으로 즉시 성공.
- **g_base [0.219, 0.054, 0.311]** (문서에 최초로 `Unable to sample any valid
  states for goal tree`가 기록된 좌표): 2회 테스트, 매번 5/5 전부 성공(0회
  또는 1회 정렬 재시도 내).
- **g_base [0.30, 0, 0.25]** (반지름 0.39m, 워크스페이스 경계 근접): ACM
  수정 후에도 5회 정렬 재시도를 전부 소진하며 실패, 안전하게 abort + look
  pose 복귀함(실행 중 충돌/정지가 아니라 사전에 걸러진 것). 진짜 워크스페이스
  경계 문제로 보이며, 이 좌표 자체가 실제 수확 시나리오에서 나올 가능성이
  낮다면 우선순위 낮음.

### 다음 세션에 할 일

1. **실물 검증** (최우선, 아직 전혀 안 함): 이번 세션은 전부 mock 스택
   (FakeSystem, 카메라 없이 `demo.launch.py`)로만 검증함. 실물 파지는 아직
   한 번도 시도 안 됨 — `docs/obstacle_avoidance_manual_test.md` 사전 조건대로
   look pose 동기화 후, 그리퍼 마운트 방향 수정(2026-07-27 오전 세션)이
   반영된 상태인지 재확인하고 실제 D435 + `pick_pipeline.launch.py`로 진행.
2. **워크스페이스 경계 근접 좌표(예: (0.30,0,0.25) 부류)를 어떻게 다룰지
   결정**: 재시도로도 못 푸는 게 정상(진짜 도달 불가 근처)이라면 현재의
   "안전하게 abort" 동작으로 충분한지, 아니면 `MAX_TARGET_RADIUS_M`을
   실제 도달 가능 범위에 맞춰 더 보수적으로 줄일지 판단 필요(harvest
   시나리오에서 이런 먼 좌표가 실제로 나오는지 실물 데이터로 확인 후 결정).
3. `MAX_ALIGN_RETRIES=5`, `ROLL_CANDIDATE_IK_TIMEOUT_SEC`/`RETRIES` 등 이번
   세션에 손대지 않은 관련 상수들은 실물 검증하면서 필요시 튜닝.

## 2026-07-27 실물 첫 파지 성공 + 그리퍼 실물 릴레이 발견 (네 번째 세션)

**결론부터**: 이번 세션에 **실물 파지가 처음으로 완전히 성공**했음(정렬→그리퍼
열기→직진 접근→파지→후퇴→look pose 복귀 5단계 전부, 실물 sync_plan 경유).
가는 길에 큰 버그를 여럿 발견/수정함 — 가장 중요한 건 **`sync_plan.py`가
그리퍼를 아예 몰랐다**는 것(아래 참고). 그 외 카메라 하드웨어 불안정,
depth hole로 인한 검출 누락, YOLO CPU 상시 과부하, 3D 프린트 소품 색상
오분류까지 전부 이번 세션에 확인/수정함. 남은 핵심 이슈는 **파지 정확도**
(줄기를 잡은 사례 1회 확인) — 다음 세션 최우선.

### 핵심 수정 1: `sync_plan.py`가 그리퍼 명령을 실물로 전혀 안 보내고 있었음

RPi(`jetcobot_126b`, `~/smh_ws/src/mycobot_ros2_humble/mycobot_280/
mycobot_280_moveit2_control/mycobot_280_moveit2_control/sync_plan.py`)의
`/joint_states` -> `send_angles()` 릴레이 로직이 **팔 6축만 relay**하고
`gripper_controller` 조인트는 애초에 목록(`rviz_order`)에 없어서 완전히
무시되고 있었음 — 이번 세션에 처음 그리퍼 actuation을 실물로 시도해보고서야
드러남(그전엔 그리퍼 자체를 실물로 시도한 적이 없어서 몰랐던 것). MoveIt/
FakeSystem 쪽은 "성공"으로 보고하는데 실물 그리퍼는 절대 안 움직이는 상태.

**수정**: `sync_plan.py`에 `gripper_controller` 값 변화를 감지해
`mc.set_gripper_state(flag, speed, gripper_type=1)`(0=open,1=close, 어댑티브
그리퍼)로 relay하는 로직 추가(중간값이 아니라 open/close 목표값과의 거리로
판정, `/joint_states`가 자주 들어와도 상태 변화 시에만 전송). 이 파일은
**이 워크스페이스 밖(RPi의 별도 `smh_ws`)에 있어서 git으로 안 잡힘** —
재현하려면 이 문서 내용을 참고해서 다시 적용할 것. RPi에서 재시작할 때
**`ROS_DOMAIN_ID=21`을 명시적으로 export해야 함**(비대화형 SSH 세션은
`.bashrc`의 `export ROS_DOMAIN_ID=21`을 안 읽어서 다른 도메인으로 떠버려
그래프에 아예 안 잡히는 사고가 있었음 — 반드시 `export ROS_DOMAIN_ID=21`을
같은 명령에 포함해서 실행할 것).

### 핵심 수정 2: YOLO depth hole — 컬러는 검출되는데 좌표 계산이 조용히 스킵됨

`_accumulate_detections`가 bbox **중심 픽셀 딱 한 점**의 depth만 보는데,
그 지점이 정확히 무효(depth=0, D435의 반사/각도 depth hole)인 경우가 실물에서
재현됨 — 컬러 검출은 0.84 신뢰도로 성공하는데 `raw_depth <= 0.0`이라 매번
조용히 버려짐(에러도 안 남음). 카메라 프레임을 직접 캡처해서 같은 모델로
재현 + 5x5 주변 depth 전부 0임을 실측 확인. **수정**: bbox 영역 전체에서
0이 아닌 depth 값의 **중앙값**을 쓰도록 변경(`_robust_bbox_depth_m` 신규
함수). 이 변경 후 검출 0개 -> 13개로 즉시 회복됨.

### 핵심 수정 3: 3D 프린트 소품 색상 오분류 (pick8.py에서 포팅)

같은 모델(v6)을 쓰는 다른 파이프라인(`pick8.py`, 자체 FK/IK 기반, 이
워크스페이스 안에 있지만 `mycobot_280_pick`과는 독립된 스크립트)에서 이미
겪고 해결한 문제: v6 모델이 **3D 프린트 소품 특유의 단색**(실제 토마토처럼
중간색이 없음) 때문에 초록 소품→ripe, 노랑 소품→unripe 등으로 자주
오분류함. `pick8.py`의 `classify_by_color`/`COLOR_OVERRIDE`를
`yolo_d435_detector_node.py`에 포팅 — bbox 안 지배적 HSV 색상(빨강=ripe/
초록=unripe/노랑=disease)으로 YOLO 클래스를 덮어씀. **포팅 중 발견한 버그**:
`result.boxes.data`가 PyTorch inference-mode 텐서라 그냥 덮어쓰면
`RuntimeError`(직접 재현 확인) — `result.boxes.data = result.boxes.data.clone()`
먼저 호출해야 함. `detect_green_blobs`(v6가 초록 소품을 아예 못 찾는 문제
보완)는 **포팅 안 함**(unripe는 `HARVEST_CLASS_NAMES`에 없어 수확 대상이
아니므로 우선순위 낮다고 판단, 사용자 확인).

### 그 외 수정 (`coord_to_goal_node.py`, `yolo_d435_detector_node.py`)

- **그리퍼 마운트 방향 재수정**: 오전 세션에 "180도 반대"로 착각해 뒤집었던
  것(rpy roll 부호 반전)이 실제로는 잘못된 축이었음이 이번 세션에 밝혀짐 —
  제조사 원본값(`elephantrobotics/mycobot_ros2` origin/humble,
  `rpy="1.579 0 0"`)으로 되돌린 뒤, 사용자가 실물 TF를 직접 비교해서 확정한
  **gripper_base 로컬 +Y축(전방) 기준 180도 롤**을 새로 합성해 적용
  (`rpy="1.562593 0 3.141593"`, `mycobot_280_m5_adaptive_gripper.urdf`,
  git으로 안 잡히는 vendor 경로). 계산 과정: 원본 회전(Rx(1.579))에
  로컬 Y축 180도 회전을 post-multiply — 전방(Y)은 유지, 좌우(X)/상하(Z)만
  반전.
- **파지 후 안전 시퀀스 강화**: 파지(4/5) 완료 후 `POST_GRASP_DWELL_SEC=2.0`
  초 대기(육안 확인용, 이전엔 없었음) 추가. 후퇴(5/5)+look pose 복귀 구간은
  `RETREAT_VELOCITY_SCALING/ACCELERATION_SCALING=0.05`로 감속(그리퍼가
  물체를 쥔 채 움직이는 유일한 구간이라 so101 교훈 그대로 적용, look pose
  도착 후 원래 속도로 복원). 실물 첫 테스트라 전체 속도(`VELOCITY_SCALING/
  ACCELERATION_SCALING`)도 0.3→0.1로 보수적으로 낮춤.
- **YOLO 상시 추론 -> look pose 누적 구간에서만 추론으로 되돌림**: Tier2에서
  "시각화는 항상 발행"으로 설계했던 것이, 실물 세션에서 CPU를 상시 크게
  잡아먹어 시스템 부하가 쌓이고 카메라 스트림 자체가 불안정해지는 악순환의
  주요 원인 중 하나로 확인됨(`ps aux` CPU% 실측, `yolo_d435_detector_node`
  단독 80%+). look pose 누적 구간에서만 추론하도록 되돌림 — 트레이드오프로
  평소엔 `tomato_boxes`/`tomato_detections_image`가 안 갱신됨(누적 중에만
  보임). Octomap 파이프라인을 나중에 같이 켜는 세션에서는 이 게이팅 때문에
  look pose를 벗어난 동안 토마토가 다시 Octomap 장애물로 잡힐 수 있음 —
  그럴 땐 재검토 필요.
- **YOLO depth 보정값 추가 조정**: `MAX_ESTIMATED_RADIUS_M` 3.5cm→2cm,
  `DEPTH_SAFETY_MARGIN_M` 신규 추가(0.03→0.05cm으로 증가, 실물 피드백
  기반 반복 조정). **주의**: depth hole 버그(핵심 수정 2) 수정 이후로
  depth 측정 자체가 더 정확해졌으므로, 이 마진이 지금은 과보정돼 있을
  가능성이 있음 — 다음 세션에 재검증 필요(주석으로 남겨둠).
- **`gripper_isolated_test.py`류 스크래치패드 스크립트**: 전체 5단계 없이
  그리퍼 열기/닫기만 단독 테스트하는 패턴을 여러 번 씀 — 재현하려면
  `MoveIt2Gripper` 인스턴스 하나로 `open()`/`close()`만 호출하면 됨(코드는
  세션 스크래치패드에만 있고 저장 안 함, 필요하면 다음 세션에 다시 작성).

### 카메라(D435) 하드웨어 불안정 — 부분 완화, 근본 해결은 아님

이번 세션 내내 반복적으로 스트림이 끊기거나(`Hardware Error`, `Depth stream
start failure`) 서서히 프레임레이트가 떨어지는(30Hz→5Hz) 문제가 있었음.
시도한 것들과 효과:
- USB 전원 자동서스펜드(`power/control`) — udev 규칙이 있는데도 재부팅 후
  `auto`로 돌아가는 경우가 있었음(원인 불명) — 수동으로 `on` 재설정 필요할
  수 있음(sudo 필요, AI가 직접 못 함).
- 케이블/포트 교체 — 한 번은 실수로 **USB 2.1 포트**로 연결돼 오히려
  악화됨("Reduced performance is expected" 경고 직접 확인) — 반드시 USB
  3.0/3.1(파란 포트/SS 마크) 확인할 것.
- **재부팅**이 가장 효과적이었음(시스템 load average가 5점대까지 쌓여있던
  것이 0.5대로 초기화됨) — 이 워크스테이션에서 다른 무거운 프로그램(다른
  Claude Code 세션 등)이 다수 실행 중이면 카메라 스트림 안정성에 실측으로
  영향을 준다는 정황 확인.
- `pointcloud.enable:=false`로 realsense 노드 자체 pointcloud 생성을 끔
  (이 파이프라인은 pointcloud를 안 쓰므로 불필요한 CPU 소비였음).
- 그래도 완전히는 해결 안 됨 — 다음 세션 시작 시 카메라 스트림이 이미
  불안정하면 위 항목들을 순서대로 재확인할 것. `ros2 topic hz
  /camera/camera/color/image_raw`로 먼저 상태 체크 권장.

### 실물 검증 결과 요약

- **첫 완전 성공 사이클**: 정렬(1~2회 재시도) → 그리퍼 열기 → 직진 접근 →
  파지 → 2초 대기 → 감속 후퇴 → look pose 복귀, 전부 실물에서 성공. 이후
  같은 패턴으로 여러 차례 반복 성공(자동 순차 처리도 확인 — look pose
  복귀 즉시 다음 타겟 재검출 후 자동 재시작).
- **⚠️ 파지 정확도 이슈**: 한 사이클에서 그리퍼가 목표 토마토가 아니라
  **나무 줄기를 잡은 사례 확인됨**(육안 확인). 그리퍼 열고 look pose로
  복귀시켜 회수함. depth hole 수정 + DEPTH_SAFETY_MARGIN_M 조정이 이
  사이클 이전/이후 어느 시점이었는지 명확치 않아, 이 정확도 문제가 지금도
  남아있는지 다음 세션에 재검증 필요.
- **정렬 재시도 중 화면이 계속 바뀌는 현상**: 버그 아님 — OMPL(RRTConnect)이
  재시도마다 확률적으로 다른 관절해를 실제로 찾아 실행하기 때문(플래닝
  미리보기가 아니라 매번 진짜 실행). 손목(J6)이 재시도 사이에 크게
  달라지는 것도 같은 원인(같은 목표 orientation을 만족하는 IK 해가 여러
  개라 그 중 다른 branch를 고르는 것으로 추정) — 실제 이상 동작은 아닌
  것으로 판단됨.
- **비상 정지 방법**: 사용자가 직접 관리하기로 함(AI가 임의로 sync_plan을
  켜고 끄지 않음) — `ssh jetcobot_126b 'pkill -9 -f sync_plan'`이 가장
  직접적(소프트 정지, 토크는 안 끊김 — 팔은 그 자리에서 힘이 걸린 채
  멈춤). 워크스테이션 쪽 `/emergency_stop` 서비스는 그보다 상위(MoveIt
  궤적 취소)라 sync_plan까지 이미 나간 명령은 못 막음.

### 다음 세션에 할 일 — 우선순위

1. **파지 정확도 재검증** (최우선): depth hole 수정 + color override
   포팅 이후 상태로 여러 좌표에서 반복 테스트, 줄기를 잡는 사례가 재현되는지
   확인. 재현되면 `DEPTH_SAFETY_MARGIN_M`을 지금(0.05) 값 그대로 유지할지
   줄일지 실물 데이터로 재조정.
2. **카메라 안정성**: 세션 시작 전에 다른 무거운 프로그램(다른 Claude
   Code 세션 등) 정리 + `ros2 topic hz` 로 스트림 상태 먼저 확인하는 걸
   습관화. 계속 불안정하면 USB 포트/케이블을 표시해두고 고정할 것(실측
   검증된 조합을 라벨링 등으로 고정하는 것도 고려).
3. `sync_plan.py`(RPi, git 밖)에 추가한 그리퍼 릴레이가 재부팅/재클론
   후에도 유지되는지 확인 — 유실됐으면 이 문서의 "핵심 수정 1" 내용으로
   재적용.
4. 그리퍼 마운트 방향 수정(`rpy="1.562593 0 3.141593"`, git 밖)도 마찬가지로
   재클론/재vendoring 시 유실 가능 — "그 외 수정" 절 참고해서 재적용.
5. `harvest_sequence_node`(다중 타겟 순차 처리)는 이번 세션에 명령어만
   확인했고 실물로 실행은 안 함 — 다음 세션 실물 end-to-end 검증 대상으로
   여전히 남아있음.

## 접근축(approach axis) 리팩터 — 방식 B의 후속 (2026-07-30)

**이 절은 위 "목표를 바라보는 orientation 동적 계산 (방식 B)"을 대체한다.**
방식 B의 방향 계산 자체는 맞았지만, 그 방향과 **웨이포인트 계산이 서로 다른 축을
쓰고 있었다**는 것이 이번에 드러났다.

### 무엇이 틀렸었나

방식 B는 forward(접근 방향)를 "현재 EE → 목표"로 잡았는데, 정작 flange 목표와 정렬
위치는 **g_base X축 성분만 빼서** 계산했다. 그리퍼 축(joint6_flange 로컬 +Z)은
X축과 최대 29°까지 벌어지므로 두 축이 어긋났고, 그 결과:

1. **손가락이 닿는 지점이 목표에서 빗나갔다** — z=0.40에서 +44mm(위로),
   z=0.14에서 −30mm(아래로). 토마토 지름이 25~36mm이므로 양 끝에서 완전히 빗나감.
   오차가 최소(6mm)인 z≈0.26 근방에서만 우연히 맞았다.
2. **정렬 위치가 목표 높이와 무관하게 항상 반지름 0.102m**에 놓였다. 손목 측면
   오프셋(URDF `joint6_to_joint5` origin의 0.0732m) 때문에 그 반지름에서는 J1이
   최소 57° 돌아야 한다(관절공간 400만 샘플로 검증). look pose의 J1이 −7.5°이므로
   매 사이클 60~120° 베이스 스윙이 강제됐다 — 실물 관측 "몸을 틀면서 이동한다"의
   정체가 이것이었다.
3. **접근 이동 방향이 그리퍼 축과 평행하지 않아** Cartesian fraction이 성공
   구간에서조차 0.85~0.90에 머물렀고, 후퇴가 매번 OMPL 폴백으로 빠졌다.

### 무엇을 바꿨나

- **순서 반전**: 위치 확정 → 방향 탐색 이었던 것을 **접근축 탐색 → 그 축 위에서
  위치 계산**으로. `flange = 목표 − 0.09·축`, `정렬 = 목표 − 0.11·축`.
- **접근축 정의**: 방위각은 목표에서 직접, 고도각은 고정 기준점
  `(-0.136, -0.035, 0.241)`(look pose의 flange 위치)에서 목표를 향하는 방향.
  `atan2`라 목표 높이에 대해 연속·단조 → 낮은 목표는 위에서, 중간은 직진, 높은
  목표는 아래에서 접근하는 프로파일이 공식 없이 나온다.
  (방식 B가 라이브 TF의 현재 EE를 쓰던 것을 고정 기준점으로 바꾼 것 — 팔이 look
  pose에 없을 때 축이 흔들리던 문제도 함께 해소.)
- **탐색 공간**: 고도각 25개(−60~+60°, 5° 간격) × roll 12개. 이상 고도각에서 5°
  이내를 한 티어로 묶고 티어 안에서 관절 이동량 최소를 채택.
- **새 게이트 3종**: 정렬 반지름 ≥ 0.13m, flange 도달 ≤ 0.29m, 정렬 도달 ≤ 0.26m
  (전부 실측 근거 있음 — `docs/VISUALIZATION_HANDOFF.md` 1·2절).

### 결과 (RViz FakeSystem, z=0.40→0.14 스윕 27개 목표)

| 지표 | 개선 전 | 개선 후 |
|---|---|---|
| 성공률 | 15/29 (52%) | **27/27 (100%)** |
| 성공 구간 | z=0.26~0.38만 | z=0.14~0.40 전 구간 |
| 관절 이동량 | 345~692° | 369~467° |
| 손가락 도달 오차 | −30~+44mm | **0.00mm** (구성상) |
| 후퇴 Cartesian 실패 | 거의 매 사이클 | **0건** |
| 정렬 재시도 | 대부분 | **0건** |

부수 효과로 **속도 여유도 늘었다** — 베이스 스윙이 60~120°에서 약 34°로 줄면서
구조 진동이 작아져, `VELOCITY_SCALING`이 0.35 → 0.5로 올라갔다(실물 검증).
`docs/SPEED_TUNING_HANDOFF.md` 6-(3)절 참고.

### 남은 한계

- **z ≥ 0.32는 아래에서만 접근 가능**하다. 토마토가 그 높이면 flange를 그보다 위로
  올릴 수 없다(어깨 기준 최대 도달 0.302m). 기구학적 한계라 우회량을 줄이는 것까지가
  최선이다.
- **실물(sync_plan 경유) end-to-end 검증은 아직**이다. 위 스윕은 전부 RViz
  FakeSystem이다.
- **J1 실제 스윙 미측정** — 로그에 관절 값이 찍히지 않아 예상치(약 34°)를 확인하지
  못했다.

상세 수치와 시각화용 정리는 `docs/VISUALIZATION_HANDOFF.md`에 있다.
