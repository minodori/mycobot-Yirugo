# Tier1~4 실물 검증 핸드북 (2026-07-27)

so101-ros-physical-ai(자매 프로젝트)가 최근 이틀간 실물 테스트로 얻은 교훈을
mycobot에 이식한 변경사항(Tier1~4, 코드 변경분)을 **실물 로봇에 연결해서**
검증하는 순서와 방법. 각 변경의 배경/설계 이유는
`docs/obstacle_avoidance_manual_test.md`("Tier5" 절 포함)에 있고, 이 문서는
그걸 실행하는 순서에만 집중함. 자주 쓰는 개별 명령어(좀비 프로세스 정리,
관절 한계 복구 등)는 `docs/obstacle_avoidance_manual_test.md`/`docs/look_pose.md`
참고.

**변경사항은 아직 커밋 안 됨** — `git status`/`git diff`로 무엇이 바뀌었는지
먼저 훑어볼 것(문제 생기면 되돌리기 쉬움).

## 변경 요약 (검증 대상)

| Tier | 파일 | 내용 |
|---|---|---|
| 1 | `coord_to_goal_node.py` | TF lookup을 검출 시점(`header.stamp`) 기준으로, 목표 반지름(0.08~0.45m)/방위각(±75°) 안전장치, 속도/가속 스케일링(0.3/0.3) 명시 |
| 2 | `yolo_d435_detector_node.py` | 시각화는 항상 발행, "판단"(`target_point`/`tomato_candidates`)은 look pose에서 2초 누적→클러스터링→1회만 발행 |
| 3 | `yolo_d435_detector_node.py` | bbox 크기 기반 depth 반지름 보정(표면→중심) |
| 4 | `coord_to_goal_node.py`, `rviz_control_panel_node.py`(신규) | `/go_to_look_pose`, `/emergency_stop`(소프트 정지) 서비스 + RViz 우클릭 제어판 |

## ⚠️ 절대 원칙 (매번)

1. **`sync_plan`을 켜는 순간 실물은 그 시점 시뮬레이션 위치로 보간 없이
   즉시 점프함**(`docs/Moveit2 setup record.md` "실행 방법" 절 실측 확인).
   시뮬레이션을 실물과 같은 자세(보통 look pose)로 맞춰두고 나서 켤 것 —
   아래 3번 순서를 건너뛰지 말 것.
2. `sync_plan`이 시리얼 포트를 물고 있는 동안 다른 pymycobot 스크립트
   (`get_angles()` 등)를 **동시 실행 금지** — 포트 충돌로 `sync_plan`이
   크래시함(실물 손상은 없었지만 재시작 필요).
3. Tier4의 `/emergency_stop`은 **MoveIt 궤적 실행 취소(소프트 정지)일 뿐,
   실물 서보 토크를 끊는 하드웨어 e-stop이 아님** — mycobot 280의 실물
   실행은 RPi `sync_plan`이 `/joint_states`를 계속 구독해서 트는 구조라,
   진짜 위급 상황엔 RPi에서 `sync_plan` 프로세스 자체를 죽이거나
   `mc.release_all_servos()`를 직접 호출해야 함(9번 순서 참고).

## 0. 빌드 확인 (실물 연결 불필요)

```bash
cd ~/Projects/mycobot
colcon build --packages-select mycobot_280_pick --symlink-install
source install/setup.bash

cd src/mycobot_280_pick
PYTHONPATH=.:$PYTHONPATH python3 -m pytest test/test_yolo_detector_logic.py -q
```

9개 유닛 테스트(클러스터링/반지름 clamp/look pose 판정) 전부 통과해야 다음
단계로 진행.

## 1. 좀비 프로세스 정리 (로컬)

```bash
ps aux | grep -E "move_group|rviz2|realsense2_camera_node|ros2_control_node|robot_state_publisher|handeye_publisher|static_transform_publisher|coord_to_goal_node|yolo_d435_detector_node|harvest_sequence_node|rviz_control_panel_node" | grep -v grep
pkill -9 -f "move_group|rviz2|realsense2_camera_node|ros2_control_node|robot_state_publisher|handeye_publisher|static_transform_publisher|coord_to_goal_node|yolo_d435_detector_node|harvest_sequence_node|rviz_control_panel_node"
```

## 2. RPi(jetcobot_126b)/시리얼 포트 사전 점검 (Tier4)

이전 세션의 `sync_plan`이나 다른 pymycobot 스크립트가 안 죽고 남아있으면
포트 충돌/이중 명령 위험이 있음(`docs/obstacle_avoidance_manual_test.md`
"참고" 절 Tier4 항목):

```bash
ssh jetcobot_126b 'pgrep -af "sync_plan|MyCobot280"; fuser /dev/ttyUSB0 2>&1'
```

뭔가 출력되면 원치 않는 프로세스인지 확인 후 정리.

## 3. 실물 팔을 look pose로 동기화 (필수, sync_plan 점프 사고 방지)

현재 코드의 `LOOK_POSE_JOINT_POSITIONS`(`coord_to_goal_node.py`/
`yolo_d435_detector_node.py` 공통, 원본은 `docs/look_pose.md`)는 degree로
환산하면 `[-7.47, 104.06, -56.68, -50.0, 15.99, 1.4]`임 — **주의:
`docs/obstacle_avoidance_manual_test.md` "사전 조건"/"1. 실물 팔을 look
pose로 이동" 절에 나오는 `[-2.98, 104.41, -31.81, -76.2, 10.54, 7.03]`는
2026-07-24에 그리퍼 충돌 문제로 이미 교체된 옛 값이니 쓰지 말 것.**
`initial_positions.yaml`은 이미 현재 값으로 맞춰져 있음(재확인:
`src/mycobot_ros2/mycobot_280/mycobot_280_moveit2/config/initial_positions.yaml`).

이미 look pose에 있으면 생략, 아니면:

```bash
ssh jetcobot_126b 'python3 -c "
from pymycobot import MyCobot280
import time
mc = MyCobot280(\"/dev/ttyUSB0\", 1000000)
time.sleep(0.1)
mc.send_angles([-7.47, 104.06, -56.68, -50.0, 15.99, 1.4], 20)
time.sleep(3)
print(mc.get_angles())
"'
```

⚠️ 릴리즈 상태에서 토크가 걸리면 현재 위치에서 목표 자세로 갑자기 움직일
수 있음 — 팔 주변 확인 후 실행.

## 4. 로컬 시뮬레이션 스택 실행

```bash
cd ~/Projects/mycobot && source install/setup.bash
ros2 launch mycobot_280_moveit2 demo_octomap.launch.py
```

`move_group` 로그에 `You can start planning now!`가 뜨고, RViz에
RobotModel/Octomap이 정상 표시되면 준비 완료(RViz는 이 launch에 이미
포함됨, 별도 실행 불필요).

## 5. RPi에서 `sync_plan` 시작 (실물이 시뮬레이션을 따라 움직이기 시작함)

시뮬레이션이 3번에서 맞춘 look pose 그대로인지 RViz로 다시 한번 확인한 뒤:

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
ssh jetcobot_126b 'tail -f ~/sync_plan_session.log'
```

로그에 `port:/dev/ttyUSB0, baud:1000000`(또는 `ttyJETCOBOT`)이 뜨면 연결
성공. 이 시점부터 `/joint_states`가 바뀌면 실물이 그대로 따라 움직임 —
아래 단계부터는 전부 실물이 실제로 움직임을 전제로 진행.

## 6. Tier1 — 목표 안전장치 단독 검증 (실물 미이동, 안전)

`coord_to_goal_node`를 아직 안 띄웠으면:

```bash
ros2 run mycobot_280_pick coord_to_goal_node
```

안전장치는 TF 변환 직후, 플래닝/실행 **이전**에 거부하므로 아래는 실물이
움직이지 않음 — 각각 발행 후 `coord_to_goal_node` 터미널에 거부 로그가
뜨는지 확인:

```bash
# 반지름 위반(0.028m < MIN_TARGET_RADIUS_M 0.08m) — 그리퍼 바로 코앞
ros2 topic pub --once /target_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'g_base'}, point: {x: 0.02, y: 0.02, z: 0.2}}"

# 방위각 위반(atan2(0.3,0.05) ≈ 80.5° > MAX_TARGET_AZIMUTH_DEG 75°)
ros2 topic pub --once /target_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'g_base'}, point: {x: 0.05, y: 0.3, z: 0.2}}"
```

`목표 반지름(...)이 안전 범위 [...]를 벗어나 실행 거부` /
`목표 방위각(...)이 안전 범위 ±75.0도를 벗어나 실행 거부` 로그가 뜨고
실물이 그대로 있으면 성공.

## 7. Tier1 — 정상 목표로 실물 이동 + 속도 체감 확인

기존에 이미 성공 검증된 좌표(`docs/obstacle_avoidance_manual_test.md`
"PointStamped 플래닝 실패 근본 원인 규명" 절, g_base `[0.219,0.054,0.311]`,
반지름 0.226m/방위각 13.9°로 Tier1 범위 안쪽)를 그대로 재사용:

```bash
ros2 topic pub --once /target_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'g_base'}, point: {x: 0.219, y: 0.054, z: 0.311}}"
```

확인할 것:
- `coord_to_goal_node` 로그에 `목표 위치로 플래닝: [...]`이 뜨고 실물이
  실제로 접근 자세로 이동.
- **속도가 이전보다 눈에 띄게 느려졌는지**(Tier1에서 처음으로
  `max_velocity`/`max_acceleration`을 0.3으로 명시했음 — 그 전엔 사실상
  기본 속도로 동작 중이었음). 너무 느리거나 여전히 빠르게 느껴지면
  `coord_to_goal_node.py`의 `VELOCITY_SCALING`/`ACCELERATION_SCALING`
  조정 후 재빌드.
- 완료 후 자동으로 look pose로 복귀하는지(`RETURN_TO_LOOK_POSE_DELAY_SEC`
  1.5초 대기 후).

## 8. Tier2/3 — YOLO 게이팅 + 누적판단 + depth 보정 (실물 카메라/YOLO 필요)

앞서 띄운 `coord_to_goal_node`를 끄고, 카메라+YOLO+coord_to_goal_node를
한 번에 띄우는 파이프라인으로 재시작(`pick_pipeline.launch.py`가
`demo_octomap.launch.py` + `yolo_d435_detector_node` + `coord_to_goal_node`를
전부 포함함 — 4번 단계의 launch를 이걸로 대체):

```bash
# 4번 단계에서 띄운 demo_octomap.launch.py / coord_to_goal_node를 정리한 뒤
ros2 launch mycobot_280_pick pick_pipeline.launch.py
```

**확인 순서**:
1. 팔이 look pose가 **아닐 때**(예: RViz Joints 탭이나 `/go_to_look_pose`
   서비스로 잠깐 다른 자세로): `tomato_boxes`/`tomato_detections_image`는
   계속 발행되는지(`ros2 topic hz tomato_boxes`), `target_point`는
   발행되지 않는지 확인.
2. look pose로 복귀(또는 처음부터 look pose에서 시작): 팔이 look pose
   도착 판정된 순간부터 `yolo_d435_detector_node` 로그에 `look pose 도착
   감지, 2.0초 누적 시작`이 뜨는지 확인.
3. 약 2초 뒤 `누적 종료 — 검출 N개 -> 클러스터 M개로 판단` +
   (ripe가 있으면) `ripe 판단 완료(...)` 로그가 **딱 한 번만** 뜨는지 —
   그 이후 look pose를 벗어나기 전까지 같은 로그가 다시 뜨면 안 됨(뜨면
   게이팅 실패, Tier2 회귀).
4. ripe 판단이 있었다면 `coord_to_goal_node`가 그 좌표로 자동 실행되고,
   완료 후 look pose로 복귀 — 복귀 후에도 (아직 이 방문에서) 추가로
   `target_point`가 재발행되지 않는지 확인(이게 so101/mycobot이 실제로
   겪었던 자동 재트리거 사고의 재현 여부 판정 포인트).
5. 일부러 look pose를 벗어났다 다시 들어와서, 새 누적 사이클이 다시
   시작되는지(`look pose 도착 감지...` 로그 재발생) 확인.
6. depth 보정 체감: 좌표 로그의 z 값이 raw depth보다 약간(1.5~3.5cm)
   커졌는지 — 정밀 검증은 어려우니 "이상하게 튀지 않는지" 정도만 확인.

## 9. Tier4 — `/go_to_look_pose`, `/emergency_stop` 서비스 단독 호출

```bash
ros2 service call /go_to_look_pose std_srvs/srv/Trigger {}
```
팔이 다른 자세에 있을 때 호출해 look pose로 이동하는지 확인.

```bash
# 위 6~8번 중 팔이 움직이는 도중에 호출해서 취소되는지 확인
ros2 service call /emergency_stop std_srvs/srv/Trigger {}
```
`emergency_stop 요청 수신 — 현재 궤적 실행 취소함(소프트 정지)` 로그와
함께 그 순간 궤적이 멈추는지 확인 — **단, 위 "절대 원칙" 3번대로 이건
소프트 정지임을 재확인**(정지 후에도 필요하면 RPi에서 `sync_plan`을 직접
끄거나 `mc.release_all_servos()`로 토크를 끌 것).

## 10. Tier4 — RViz 인터랙티브 마커 제어판

```bash
ros2 run mycobot_280_pick rviz_control_panel_node
```

RViz에서 Displays 패널 -> Add -> `rviz_default_plugins/InteractiveMarkers`
추가 -> "Interactive Marker Namespace"를 `rviz_control_panel`로 직접
타이핑(자동완성 안 될 수 있음). 파란 박스 마커가 보이면 우클릭 -> 메뉴 3개
(`look pose로 이동`/`수확 시퀀스 시작`/`정지(소프트, 궤적 취소)`) 클릭해서
각각 `rviz_control_panel_node`와 호출 대상 노드 양쪽 로그에 반응이 뜨는지
확인. `수확 시퀀스 시작`은 `harvest_sequence_node`를 먼저 띄워야 함
(11번 참고) — 안 띄운 상태로 누르면 `서비스가 아직 준비 안 됨` 경고만
뜨는 게 정상.

## 11. (선택) `harvest_sequence_node` 통합 확인

Tier2 변경으로 `tomato_candidates` 발행 시점이 "매 프레임"에서 "look pose
방문당 1회"로 바뀌었으므로, `harvest_sequence_node`가 여전히 정상 동작하는지
같이 재확인 권장(이 노드 자체는 이번 Tier 변경 대상이 아니지만 의존 관계가
있음):

```bash
ros2 run mycobot_280_pick harvest_sequence_node
```

look pose에서 ripe/disease 토마토가 여러 개 보이는 상태로 누적 종료 로그가
뜬 뒤(8번 참고) 호출:

```bash
ros2 service call /start_harvest_sequence std_srvs/srv/Trigger {}
```

큐가 깊이(z) 가까운 순서로 정렬되어 순차 접근하는지, 완료 후
`수확 시퀀스 완료 — 큐 비어있음` 로그가 뜨는지 확인.

## 12. 정리

```bash
# RPi: sync_plan 종료 (실물 토크가 걸린 채로 남을 수 있음 — 팔 자세 확인 후)
ssh jetcobot_126b 'pkill -f "sync_plan$"'

# 로컬 프로세스 정리
pkill -9 -f "move_group|rviz2|realsense2_camera_node|ros2_control_node|robot_state_publisher|handeye_publisher|static_transform_publisher|coord_to_goal_node|yolo_d435_detector_node|harvest_sequence_node|rviz_control_panel_node"
```

`sync_plan` 종료 후에도 서보에 마지막 명령이 남아있을 수 있음 — 완전한
토크 해제가 필요하면 `docs/handeye_calibration.md`에 이미 기록된 절차대로
`mc.release_all_servos()`를 직접 호출할 것(단, `sync_plan`이 완전히
죽어서 포트를 놓은 다음에만 — 절대 원칙 2번 참고).

## 13. (참고) 0~11번 통과 시 mycobot 파이프라인 완성도와 남은 과제

이 문서의 0~11번을 전부 실물로 통과했다면, "카메라가 토마토를 보고 →
안전하게 장애물 피해서 → 그 위치 근처까지 팔이 접근한다"까지는 완성/검증된
상태. 세부적으로:

### 완성된 부분

1. **검출** — D435 + YOLO, 시각화는 항상/판단은 look pose에서만 게이팅+누적
   (Tier2) — 자동 재트리거 사고 원천 차단.
2. **좌표 계산** — handeye 캘리브레이션 기반 TF 변환(검출 시점 기준, Tier1)
   + depth 반지름 보정(표면→중심, Tier3).
3. **안전장치** — 목표 반지름/방위각 범위 검증, 속도/가속 스케일링(Tier1).
4. **장애물 회피 플래닝** — Octomap + pointcloud 사전 필터링(로드맵 2단계,
   이미 실물 검증됨) + MoveIt2 플래닝.
5. **실물 이동** — 정렬 → (그리퍼 없이) 목표 오프셋 위치까지 접근 → look
   pose 자동 복귀.
6. **다중 대상 순차 처리** — `harvest_sequence_node`로 깊이 순 큐잉 → 순차
   접근.
7. **운영 편의** — RViz 제어판, `/go_to_look_pose`·`/emergency_stop`
   서비스(Tier4).

### 남은 과제

1. **실제 그리퍼 파지(actuation)** — 가장 큰 공백. 지금은 오프셋 위치로
   접근하는 것으로만 대체하고 있고, 실제로 쥐고 따는 동작 자체가 없음.
   구현 시 so101 교훈(그리퍼 열기를 명시적 첫 단계로 넣을 것 —
   `docs/obstacle_avoidance_manual_test.md` "[Tier5]" 절 참고)을 반드시
   반영할 것.
2. **파지 성공 여부 판정** — 그리퍼가 없으니 당연히 없음. 나중엔 "MoveIt
   실행 성공 ≠ 실제 파지 성공"(so101이 겪은 문제) 구분법도 필요.
3. **후퇴 궤적 직선성** — 관절공간 플랜이라 잡고 위로 솟았다 내려오는
   문제가 날 수 있음(그리퍼가 없어서 아직 안 드러남). so101도 미해결이라
   그쪽에서 검증된 해법이 나오면 참고하기로 기록만 해둔 상태(위 문서
   "[Tier5]" 절 2번 참고).
4. **완전 자동 반복 오케스트레이션** — `harvest_sequence_node`는 한 번 큐
   처리하면 끝. look→캡처→수확→복귀를 계속 반복하는 상위 루프는 아직
   없음, 매번 `/start_harvest_sequence`를 다시 호출해야 함.
5. **위치 정확도 추가 개선** — Tier3 depth 보정은 부분 완화일 뿐(so101도
   3~4cm 부족 문제를 완전히 못 고침), 캘리브레이션 잔차 등 추가 원인
   조사가 남을 수 있음.
6. **automato_ws로 포팅** — 문서에 명시된 최종 목표(`docs/
   obstacle_avoidance_manual_test.md` "이후 원래 로드맵... 완성되면
   automato_ws로 포팅" 참고)인데 아직 시작 전.
