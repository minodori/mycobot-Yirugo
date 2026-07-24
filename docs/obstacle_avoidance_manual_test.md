# Octomap 장애물 회피 수동 테스트 절차 (2026-07-23)

`demo_octomap.launch.py` + `coord_to_goal_node`로, look pose에 고정된 실물
카메라가 보는 장애물을 MoveIt2가 실제로 피해서 목표 좌표까지 플래닝하는지
수동으로 확인하는 절차. `docs/look_pose.md`, `docs/handeye_calibration.md`와
이어지는 내용.

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

**아직 안 한 것 — 실물 end-to-end 검증**: 위 구현은 기하학적/수학적으로는
검증됐지만, **실제 로봇+MoveIt2 스택에서 여러 목표 좌표에 대해 플래닝이
성공하는지는 다음 세션에 실물로 확인 필요**(이 문서 1~7단계 절차 그대로,
단 이번엔 orientation이 매번 자동 계산됨). 특히 확인할 것:
- 과거 실패/성공이 갈렸던 두 좌표((0.125,-0.15,0.204), (0.158,0.198,0.152))
  둘 다 새 방식으로 성공하는지
- `tolerance_orientation=0.5`가 여전히 필요한지, 동적 계산이라 더 타이트하게
  줄여도 되는지
- `WORLD_UP` 기준이 실제 작업공간 전체에서 자연스러운 접근 자세를 만드는지
  (특이한 위치에서 그리퍼가 부자연스럽게 뒤집히는 경우가 있는지)

**향후 계획(참고, 지금 당장은 아님)**: AI Service가 토마토의 기울기
정보를 제공할 예정 — `WORLD_UP` 상수를 world +Z 대신 그 기울기 값으로
대체하면, 목표를 향하는 방향은 유지하면서 그 축 둘레 회전(roll)만
토마토 기울기에 맞추는 자연스러운 확장이 됨.

## 상태 (2026-07-24 기준)

✅ 그리퍼 메시 스케일 버그 수정 — `gripper_base - target_object` 등
월드 오브젝트 충돌 문제 해결됨(위 항목 참고).
✅ look pose 확정 + 실물 동기화 절차 확립, 5회 반복 왕복(시뮬레이션+실물)
성공 확인.
✅ orientation 동적 계산(방식 B) 구현 완료 — 그리퍼 정면축(flange 로컬
+Z) 확정, look-at 함수 작성, `coord_to_goal_node.py` 적용, 빌드 확인.
다음 세션에 실물 end-to-end 검증 필요(위 "아직 안 한 것" 참고).
⬜ 그리퍼 근접거리 self-filter 튜닝(`padding_scale: 0.92`/`padding_offset: 0`)은
실용적 타협 수준 — 정식 해결(근접거리 depth 필터 등) 아직 미착수.
