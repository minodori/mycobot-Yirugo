# 수확 런북 — 무엇을 어떻게 돌리나

> **이 문서의 역할.** `docs/ARMED_POSE_HANDOFF.md`는 **왜 이 수치인가**를 남기는
> 측정 기록이고(1700줄, 함정 18개), 이 문서는 **어떻게 돌리는가**만 담는다.
> 근거가 궁금하면 그쪽 절 번호를 따라가면 된다.
>
> **명령에 줄임표를 쓰지 않는다.** 예전에 `--cycle asc ...`처럼 줄여 적었더니
> 그대로 실행한 결과가 "씬이 비고, ACM이 안 걸리고, 화면에 아무것도 안 뜨는"
> 상태로 **에러 없이 100% 성공**했다(핸드오프 함정 18). 여기 있는 명령은 전부
> 복사해서 그대로 쓸 수 있는 완전한 형태다.
>
> 2026-08-03 기준. **실물 검증은 아직 없다** — 5절을 반드시 읽을 것.

---

## 1. 세 가지 수확 사이클

대기 자세를 무엇으로 두느냐가 곧 사이클의 종류다.

| 이름 | 뜻 | 대기 자세 | 통에 놓나 | 언제 |
|---|---|---|---|---|
| **LPC** | Look-Parked Cycle | look pose | 아니오 | 원판 |
| **ASC** | Armed-Staged Cycle | armed pose | 아니오 | 2026-08-01 |
| **BSC** | Bin-Staged Cycle | 통 자세 | **예** | 2026-08-02, **현재 기본** |

비용은 BSC가 가장 싸다(같은 세션, 반복 10회, 핸드오프 3.0절):

| 사이클 | 성공률 | 정렬 이동량 중앙 | 복귀 합 |
|---|---|---|---|
| LPC | 100% | 509° | 7753° |
| ASC | 100% | 461° | 7465° |
| **BSC** | 100% | **362°** | **6247°** |

### 고르는 법 — 세 자리 모두 같은 이름

아래 셋은 **차례로 실행하는 절차가 아니라 서로 대체하는 명령**이다.

```bash
# (A) 실물/시뮬 파이프라인 — 스택 + YOLO + 좌표변환 + 수확 시퀀스를 한 번에
ros2 launch mycobot_280_pick pick_pipeline.launch.py cycle:=bsc

# (B) 노드만 (스택이 이미 떠 있을 때)
ros2 run mycobot_280_pick coord_to_goal_node --ros-args -p waiting_pose:=bsc

# (C) 측정 스윕 — 노드가 아니라 스크립트가 계획한다(3절)
```

> **(A)와 (B)를 같이 띄우지 말 것.** `coord_to_goal_node`가 2개가 되어 같은
> `/target_point`에 **둘 다 반응하고 같은 팔을 동시에 계획한다.** (A)는 노드를
> 이미 포함한다. 실제로 그렇게 떠 있는 것을 발견한 적이 있다.

`look|armed|bin`도 그대로 받는다(약칭은 별칭이다). 값과 이름의 단일 출처는
`coord_to_goal_node.WAITING_POSES` 하나이고 스윕도 거기서 읽는다.

---

## 2. 목표를 어디서 받나

### 2.1 두 가지 출처

| 출처 | 언제 쓰나 | 좌표계 |
|---|---|---|
| `yolo` (기본) | look pose에서 검출한 스냅샷 그대로 | 카메라 → g_base 변환(팔이 look pose에 있어야 유효) |
| `file` | **검출 좌표를 사람이 보정한 파일** | g_base 그대로 — TF를 안 타므로 팔이 어디 있든 유효 |

YOLO 좌표는 실제로 파지해 보면 오차가 있다(bbox depth가 물체의 *카메라 쪽
표면*이라 중심보다 짧게 잡힌다). 그 오차를 손으로 고친 파일을 쓰는 경로다.

```bash
# 1) look pose에서 검출 결과를 파일로 뽑는다 (팔이 look pose에 있는 동안!)
python3 -u scripts/export_detections.py --out bags/today --timeout 60

# 2) bags/today.json을 복사해 좌표를 손으로 고친다
cp bags/today.json bags/today_fixed.json

# 3) 무엇을 얼마나 고쳤는지 확인한다 (ROS 불필요, 단위는 mm)
python3 scripts/export_detections.py --compare bags/today.json bags/today_fixed.json

# 4) 그 파일로 수확한다
ros2 launch mycobot_280_pick pick_pipeline.launch.py \
    cycle:=bsc target_source:=file targets_file:=bags/today_fixed.json
```

`--compare`는 목표를 가까운 것끼리 짝지어 Δx/Δy/Δz를 **mm로** 찍는다. 수십 mm가
보이면 m/mm를 헷갈린 것이다 — 로봇을 움직이기 전에 여기서 걸러야 한다.

### 2.2 수확 시작

**YOLO가 팔을 직접 움직이지 않는다.** 시작은 항상 서비스다:

```bash
ros2 service call /start_harvest_sequence std_srvs/srv/Trigger
```

`yolo` 출처면 팔이 look pose에 있고 판단(누적 5초)이 끝난 뒤에 부를 것.

### 2.3 수확 중에는 검출이 얼어 있다

두 겹으로 막는다:

- `publish_target_point`(기본 **false**) — YOLO는 `tomato_candidates`만 내고
  `/target_point`는 안 쏜다. 파지 명령은 오직 시퀀스 노드를 거친다.
- 시퀀스가 시작될 때 판단 자체를 얼리고(`set_judgment_enabled(false)`), 끝나서
  look pose로 복귀 요청한 뒤 푼다.

왜 필요한가 — 판단이 열리는 시점이 사이클마다 다르다. **LPC는 목표마다 look
pose로 돌아가 매 수확마다 열린다.** ASC/BSC도 시퀀스 끝에서 한 번 열린다. 예전엔
그 자리에서 YOLO가 `/target_point`를 쏴 "도달 → 복귀 → 재검출 → 다시 이동"이
무한 반복된 사고가 있었다(2026-07-24).

단발 수동 테스트로 예전 동작이 필요하면:

```bash
ros2 launch mycobot_280_pick pick_pipeline.launch.py \
    cycle:=bsc      # 그리고 별도 터미널에서
ros2 param set /yolo_d435_detector_node publish_target_point true
```

> 이 파라미터를 켜면 위 사고 경로가 다시 열린다. 실물에서는 손을 e-stop에 둘 것.

---

## 3. 화면으로 보기 (FakeSystem)

계획만 하고 실행하지 않는 스윕이다. **로봇도 카메라도 필요 없다.**

### 3.1 합성 씬 위에서 (기본)

```bash
# 터미널 A — 측정 스택
ros2 launch mycobot_280_moveit2 demo_octomap.launch.py \
    enable_octomap:=false enable_camera:=false \
    enable_pointcloud_filter:=false use_rviz:=false

# 확인: 반드시 1이어야 한다(핸드오프 함정 1)
pgrep -xc move_group

# 터미널 B — RViz
rviz2 -d src/mycobot_280_pick/config/sweep_view.rviz

# 터미널 C — 스윕(한 사이클 전체 재생)
python3 -u scripts/sweep_targets_planned.py \
    --repeat 5 --orientation-tolerance 0.2 \
    --tomatoes --octomap bags/bed_look_octomap_wall.bin --octomap-acm \
    --cycle bsc --straight-in --full-cycle \
    --display-pause 1.0 --display-seconds 6 --display-repeats 2 --display-loop 0
```

세 사이클을 비교하려면 `--cycle`만 갈아 끼워 **같은 스택 안에서** 연달아 돌린다
(세션이 다르면 ±8%가 그냥 흔들린다 — 함정 10).

터미널에 목표마다 한 줄씩 나온다. 화면 라벨 4줄은 `BSC:bin / #4/15:ripe:z0.22 /
trv534:J1+127 / J6:11:B-wrap:3/5:ret421` 형식이고 마지막의 `ret###`이 복귀
이동량(`--full-cycle`)이다. 파랑=A분기, 주황=B분기, 빨강=도달 불가.

> 옵션을 빼면 **조용히 다른 조건**이 된다. 씬 인자가 없으면 빈 씬이고(뚫고 가는
> 경로가 성공으로 잡힌다), `--display-pause`가 없으면 화면에 아무것도 안 그린다.
> 스크립트가 그 둘을 각각 경고하고 `씬 확인(되읽음):` 줄에 **실제로 씬에 든 것**을
> 찍는다 — 그 줄을 보고 시작할 것.

### 3.2 실제 녹화 장면 위에서

합성 `.bin` 대신 bag이 만든 **살아 있는 octomap·포인트클라우드**를 배경으로 쓴다.

```bash
# 터미널 A — 재생 스택 (측정 스택과 **같이 띄우지 말 것**, move_group 2개가 된다)
ros2 launch mycobot_280_pick scene_replay.launch.py \
    rviz_config:=replay_sweep.rviz

# 재생이 끝나고 octomap이 안정된 뒤(로그에 "Octomap cleared." + 40초쯤),
# 터미널 B — octomap 인자 **없이** 돌린다. 씬에 이미 있다.
python3 -u scripts/sweep_targets_planned.py \
    --repeat 5 --orientation-tolerance 0.2 \
    --tomatoes --cycle bsc --straight-in --full-cycle \
    --display-pause 1.0 --display-seconds 6 --display-repeats 2 --display-loop 0
```

`replay_sweep.rviz`는 `scene_replay.rviz`에 재생용 디스플레이 둘
(`/sweep_robot_state`, `/tomato_markers`)만 더한 것이다.

> 재생 **중**에는 voxel이 계속 늘어 같은 조건이 아니다 — 수치로 인용하려면
> 재생이 끝난 뒤에 돌리거나, `octomap_io.py capture`로 파일에 굳혀 쓸 것.

**재생 배경과 octomap 주입은 양립하지 않는다.** bag이 `--loop`로 도는 동안에는
카메라 클라우드가 octomap을 계속 갱신해 **주입한 voxel을 덮어쓴다.** 그렇다고
재생을 멈추면 포인트클라우드와 YOLO 이미지가 같이 죽는다(실제로 겪었다 — 화면에
배경이 사라졌다). 둘 다 원하면 **장애물을 collision object로 넣을 것**(4.1절).

#### 구가 실제 열매를 가릴 때

토마토 collision object는 검출 반지름 **+10mm**짜리 구라 배경의 진짜 열매를 덮는다.
셋 다 조절할 수 있다:

| 무엇 | 어디서 |
|---|---|
| collision object 구(초록, 플래너가 보는 것) | RViz `PlanningScene > Scene Geometry > Scene Alpha` (기본 0.35, GUI에서 실시간) |
| `/tomato_markers` 구(상태 색) | 스윕 `--marker-alpha 0.25` (0이면 사실상 숨김) |
| 아예 끄기 | RViz 트리에서 `토마토 (검출 결과)` 또는 `Show Scene Geometry` 체크 해제 |

---

## 4. 장애물 데모 — 있을 때와 없을 때

베드와 수확통 **사이**에 기둥을 세워 경로가 돌아가는 것을 보여준다.
`--preset`으로 무엇을 세울지 고른다:

| 프리셋 | 무엇 | 위치 |
|---|---|---|
| `pillar` (기본) | 베드–통 사이 기둥 | x 0.06~0.14, y 0.08~0.16, z 0~0.30 |
| `wall` | 베이스 오른쪽 벽(실측) | y = −0.20, x −0.50~0, 높이 150mm |
| `both` | 둘 다 | |

(`--preset`은 `--as-object`에만 적용된다. voxel 쪽은 `add-box --wall` /
`add-box --obstacle`로 파일을 만든다.)

```bash
# FakeSystem: octomap voxel로 (화면에 voxel로 보인다)
python3 scripts/octomap_io.py obstacle add
python3 scripts/octomap_io.py obstacle remove

# 실물·재생 중: collision object로 (아래 이유 때문에 그때는 이쪽만 된다)
python3 scripts/octomap_io.py obstacle add    --as-object --preset both
python3 scripts/octomap_io.py obstacle remove --as-object --preset both
```

수치로도 비교한다 — 같은 세션에서 유무만 바꿔 두 번:

```bash
python3 -u scripts/sweep_targets_planned.py --repeat 10 --orientation-tolerance 0.2 \
    --tomatoes --octomap bags/bed_look_octomap_wall.bin \
    --cycle bsc --full-cycle
python3 -u scripts/sweep_targets_planned.py --repeat 10 --orientation-tolerance 0.2 \
    --tomatoes --octomap bags/bed_look_octomap_wall.bin --obstacle \
    --cycle bsc --full-cycle
```

실측(반복 10회, ACM 완화 없음):

| 장애물 | 성공률 | 도달 | 정렬 이동량 중앙 | 복귀 합 |
|---|---|---|---|---|
| 없음 | 68.0% | 13/15 | 437° | 4887° |
| **있음** | 59.3% | 13/15 | **529° (+21%)** | **6163° (+26%)** |

**도달 목표 수가 그대로**인 것이 중요하다 — 막는 것이 아니라 돌아가게 하는
위치를 골랐다(정렬 자세도 통 자세도 안 건드린다).

### 4.1 화면으로 대비를 보여줄 때 (녹화용)

위 스윕은 목표 15개를 **순회**하므로 화면에는 매번 다른 목표가 지나간다 —
"장애물 때문에 경로가 이렇게 달라졌다"가 안 보인다. 비교하려면 **목표를 고정하고
조건만 바꿔야** 한다:

```bash
python3 -u scripts/demo_obstacle_ab.py \
    --target 4 --cycle bsc --preset both \
    --repeat 15 --display-seconds 6 --display-repeats 2 --pause 1.5
```

> **`--repeat`를 15회 이상으로 줄 것.** 5회로 돌렸더니 어떤 회차는
> ON 516° / OFF 512°로 **차이가 아예 안 보였다**(녹화 중 실제로 겪음).
> 장애물이 없는 쪽이 가끔 비싼 분기(B무리)에 걸리기 때문이고, 이게 핸드오프
> 함정 16(두 분기 비용이 비슷해 최소 선택이 동전 던지기가 된다)이다. 반복을
> 올리면 최소가 수렴해 짧은 경로를 안정적으로 찾는다 — 계획은 목표당 0.05초라
> 비용도 거의 없다.

한 목표에 대해 `장애물 ON`(주황 라벨) → `OFF`(파랑 라벨)를 무한 반복한다.
실측(목표 #4, 기둥+벽):

| 조건 | 정렬 이동량 | 복귀 이동량 |
|---|---|---|
| ON | 429~527° | 411~513° |
| OFF | 333~357° | 331~356° |

**정렬 +20~58%, 복귀 +15~55%** (반복 15회, 회차별 범위).

- `--target 1~15`로 목표를 바꾼다(차이가 큰 목표를 골라 쓸 것)
- `--preset pillar|wall|both`
- 장애물은 **collision object**로 넣는다 — 재생 중이든 실물이든 안 지워진다
- 끝나면 스크립트가 장애물과 열매 구를 스스로 치운다

> **이 데모에 `--octomap-acm`을 켜지 말 것.** 그리퍼·손목 10개 링크가 octomap
> voxel을 통과해도 되게 만드는 옵션이라(핸드오프 6.3절) 장애물을 손목까지
> 무시하게 된다. `--as-object` 쪽은 ACM을 안 건드리므로 모든 링크가 피한다.

**왜 실물에서는 `--as-object`인가**: 실물은 D435 클라우드가 octomap을 계속
갱신해 주입분을 덮어쓰고, `coord_to_goal_node`가 목표마다 `/clear_octomap`을
부른다(핸드오프 5.6b). 둘 중 하나만 있어도 주입한 voxel이 사라진다. collision
object는 둘 다에 안 지워진다.

---

## 4.2 팔이 너무 느릴 때 (재생 속도)

FakeSystem에서 사이클을 볼 때 팔이 느린 것은 **속도·가속 스케일링**과 **단계 사이
정지 시간** 둘이 정한다. 지금 값(`coord_to_goal_node.py` 상수):

| 구간 | 속도 | 가속 |
|---|---|---|
| 일반(경유·정렬·복귀) | `VELOCITY_SCALING` 0.2 | `ACCELERATION_SCALING` 0.1 |
| [3/5] 직진 접근 | 0.2 | 0.2 |
| [5/5] 후퇴 | 0.2 | 0.2 |

정지 시간은 목표당 약 6.7초다(파지 후 2.0 + 복귀 전 1.5 + octomap clear 1.2 +
다음 목표 1.5 + 놓기 0.5).

바꾸는 법 — 셋 다 **재시작 없이** 먹는다:

```bash
# 런치할 때
ros2 launch mycobot_280_pick pick_pipeline.launch.py cycle:=bsc \
    speed_scale:=3.0 dwell_scale:=0.3

# 이미 돌고 있으면 (다음 동작부터 반영)
ros2 param set /coord_to_goal_node speed_scale 3.0
ros2 param set /coord_to_goal_node dwell_scale 0.3
```

**RViz에서도 된다** — `rviz_control_panel_node`의 우클릭 메뉴에 `재생 속도 >
x1/x2/x3/x5`가 있다. (RViz **MotionPlanning 패널**의 Velocity Scaling 슬라이더는
이것과 무관하다 — 그건 그 패널이 직접 보내는 계획에만 붙고, 우리 사이클은
`coord_to_goal_node`가 계획한다.)

> **이 배수는 시뮬 전용이 아니다.** 계획한 궤적의 시간축이 곧 실물 속도다
> (`sync_plan`은 `/joint_states`를 중계할 뿐이다). 그래서 노드가 **실물
> 브릿지(`/release_servos` 서비스)가 붙어 있으면 speed_scale을 1.0으로 자르고
> 경고한다.** 실물 속도를 진짜로 바꾸려면 `VELOCITY_SCALING` 상수를 튜닝할 것
> — 근거는 `docs/SPEED_TUNING_HANDOFF.md`에 있다.

---

## 4.3 수확 중 화면이 요동칠 때 (장면 고정)

카메라가 **eye-in-hand**(joint6)라 팔이 움직이는 동안에도 클라우드를 계속 내면
`occupancy_map_monitor`가 그때그때 본 것을 합치고 시야를 따라 지운다. 수확 중
장면이 쉬지 않고 바뀌어 무엇을 보고 있는지 읽기 어렵다.

> **`sync_plan`을 안 켠 세션에서는 더 나쁘다.** 실물 팔은 가만히 있는데 카메라
> TF는 `/joint_states`(= 시뮬 팔)로 계산되므로, **정지한 카메라가 본 클라우드를
> 움직이는 팔의 위치에 갖다 붙인다.** 장면이 작업공간 전체에 번져 쌓인다.

look pose에 있을 때만 클라우드를 내게 막는다. 이 노드는
`demo_octomap.launch.py`가 띄우므로 **실행 중에 파라미터로** 켠다:

```bash
ros2 param set /pointcloud_tomato_filter_node look_pose_gate true
ros2 param set /pointcloud_tomato_filter_node look_pose_gate false   # 되돌리기
```

켜지면 로그에 `look pose 이탈 — 클라우드 발행 중지(장면 고정)`가 찍힌다.

**무엇이 바뀌고 무엇이 안 바뀌나:**

| | 게이트 켜면 |
|---|---|
| 포인트클라우드 표시 | look pose에서 본 마지막 장면으로 **고정**된다 |
| octomap voxel | 목표마다 `/clear_octomap`이 여전히 돌므로 **수확 중에는 비어 있다**(다음 look pose 방문에서 다시 쌓인다) |
| 계획 동작 | **안 바뀐다** — 지금처럼 장애물 회피가 사실상 꺼진 상태 그대로다 |

octomap까지 look pose 장면으로 **고정**하려면 목표마다 지우는 것을 시퀀스마다로
바꿔야 하는데, 그건 **장애물 회피를 켜는 결정**이다(핸드오프 5절: 성공률
100% → 77%, 직진 12/15 → 3/13). 핸드오프 7절 남은 일 3번이 그 항목이다.

---

## 5. 실물 연결

구조: **노트북(계획) → 네트워크 → RPi `sync_plan` → USB → 로봇**.
순서와 절대 원칙은 `docs/real_robot_tier_verification.md`에 있다. 여기서는 그중
**반드시 지켜야 할 셋**만 옮긴다.

1. **`sync_plan`을 켜는 순간 실물은 시뮬 위치로 보간 없이 점프한다.** 켜기 전에
   시뮬을 실물과 같은 자세(보통 look pose)로 맞출 것.
2. `sync_plan`이 시리얼을 물고 있는 동안 다른 pymycobot 스크립트를 **동시 실행
   금지**.
3. `/emergency_stop`은 **MoveIt 궤적 취소(소프트 정지)**다. 하드웨어 e-stop이
   아니다 — 진짜 위급하면 RPi에서 `sync_plan`을 죽이거나
   `mc.release_all_servos()`를 직접 부를 것.

### 5.1 실물에서 처음 돌릴 때 순서 (권장)

**모든 수치가 FakeSystem이다.** 아래 순서는 "위험이 적은 것부터" 정렬한 것이다.

| # | 무엇 | 왜 이 순서인가 |
|---|---|---|
| 1 | **통의 물리적 위치 확인** — g_base `(0, +0.15)`, 림 z=0.100 | 통 자세는 이 좌표를 전제로 뽑은 값이다. 통이 없거나 어긋나면 팔이 **허공에서 그리퍼를 연다** |
| 2 | 팔만 세 자세로 보내 보기(`/go_to_look_pose`, `cycle:=` 바꿔 기동) | 통 자세(J1 +122.5°)는 **실물에서 한 번도 실행된 적이 없다.** 케이블·작업대와의 간섭을 눈으로 볼 것 |
| 3 | `target_source:=file`로 목표 1~2개만 | TF도 카메라도 안 타는 결정적 경로다. **좌표 파일은 오늘 새로 뽑을 것** — 베드가 옮겨졌으면 예전 파일은 허공을 가리킨다 |
| 4 | 놓기 확인 | 그리퍼가 연직에서 6.5° 기운 채 림 위 29mm에서 연다. 열매가 통에 들어가는지, 튀지 않는지 |
| 5 | `target_source:=yolo` 전체 시퀀스 | 위가 다 되고 나서 |
| 6 | 장애물 데모(`--as-object`) | 경로 회피는 마지막 |

### 5.2 실물에서 달라지는 것

- **octomap이 실제 카메라로 채워진다.** 다만 `coord_to_goal_node`가 목표마다
  `/clear_octomap`을 부르고 갱신이 1Hz라, 정렬 플래닝이 도는 동안 octomap은 거의
  비어 있다(핸드오프 5.6b). **즉 지금 실물은 장애물 회피가 사실상 꺼진 채 돈다.**
  켜면 성공률이 77%가 된다 — 어느 쪽을 원하는지가 핸드오프 7절 3번의 결정이다.
- **그리퍼 폭이 목표마다 달라진다**(2026-08-02). 작은 열매(반지름 12.5mm)에서
  쓰이는 `GRIPPER_OPEN_POSITION_SMALL = -0.49`는 코드 주석이 *"URDF 근사로
  역산한 값, 완전한 실측 아님 — 실물 확인 필요"*라고 적어 둔 값이다.
- **스윕 스크립트를 실물 스택에 대고 돌리지 말 것.** 계획만 하지만 씬에 토마토
  구와 octomap을 **주입**하므로 실제 수확의 플래닝 씬을 오염시킨다.

### 5.3 아직 검증이 없는 것

- 통의 실제 치수와 위치(입구 8cm 가정)
- 놓는 순간 열매가 통에 들어가는가
- BSC 경로가 실물 작업대·케이블과 간섭하지 않는가
- 실물 octomap이 재생본만큼 안정적인가

---

## 6. 자주 밟는 함정

전체 목록과 근거는 **핸드오프 2절(함정 18개)**에 있다. 자주 나오는 것만:

| # | 요약 |
|---|---|
| 1 | `move_group`이 2개면 조용히 섞인다 — `pgrep -xc move_group`으로 확인 |
| 3 | 빈 씬에서 재면 뚫고 가는 경로가 성공으로 잡힌다 |
| 4·17 | 열매 ACM 완화는 **접근할 하나가 아니라 전부**에 걸린다 |
| 10 | 세션이 다르면 같은 씬도 ±8% 다르다 — 비교는 한 세션에서 |
| 13 | `/clear_octomap`이 비동기라 뒤이은 주입을 덮어쓴다 |
| 14 | RViz 라벨에 **공백·한글·`°`를 넣지 말 것**(폰트 아틀라스에 없다) |
| 18 | 옵션을 줄인 스윕은 빈 씬에서 조용히 100% 성공한다 |
