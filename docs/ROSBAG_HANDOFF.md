# rosbag 핸드오프 — 스윕 기록·재생·분석

> **이 문서의 용도**
> Claude Desktop에게 **그림과 도표**를 만들게 하기 위한 자료다. `docs/VISUALIZATION_HANDOFF.md`의
> 자매 문서로, 그쪽이 "무엇을 만들었나"라면 이쪽은 **"어떻게 측정하나, 그래서 무엇을 알아냈나"**를 다룬다.
> 1. 왜 rosbag이 필요했나 — 텍스트 로그로는 측정 불가능했던 것
> 2. 기록·재생·분석 방법 (명령과 함정)
> 3. 첫 측정 결과 — **예측이 절반만 맞았다**
>
> **두 종류의 bag을 다룬다.** 0~4절은 **스윕 bag**(팔의 움직임, `scripts/record_sweep.sh`),
> 6~7절은 **장면 bag**(카메라가 본 토마토 베드, `scripts/record_scene.sh`)이다. 목적도
> 담는 토픽도 함정도 다르니 절 번호를 확인하고 읽을 것.
>
> **모든 수치는 이 문서 안에서 완결된다.** 각 절 끝에 `[시각화 제안]`을 달았다.

---

## 0. 왜 rosbag인가 — 6번의 스윕 동안 못 본 것

`coord_to_goal_node`는 진행 상황을 텍스트 로그로 찍는다. 그동안 분석은 그 로그를 `tee`로 받아
정규식으로 긁는 방식이었다. 이 방식의 한계가 하나 있었다:

**로그에는 관절값이 없다.**

접근축 리팩터의 핵심 근거는 *"손목 측면 오프셋 때문에 정렬 위치 반지름이 작으면 J1(베이스)이
크게 돌아야 한다. 반지름을 0.102 m → 0.131 m로 올려 J1 스윙을 60~120°에서 **약 34°**로 줄였다"*
였다. 그런데 로그에 찍히는 것은 `compute_ik`가 **예상한** 관절 이동량 합뿐이고, 팔이 **실제로**
어떻게 움직였는지는 없다. 그래서 6번의 스윕을 돌면서도 이 예측을 한 번도 검증하지 못했다.

`/joint_states`를 rosbag으로 남기면 100 Hz 실제 궤적이 그대로 들어온다. 그것이 이 작업의 동기다.

### 텍스트 로그 vs rosbag

| 지표 | 텍스트 로그 | rosbag |
|---|---|---|
| 단계 전환, 채택 고도각, 게이트 거부 사유 | ✅ | ✅ (`/rosout`, 필드 분리되어 파싱이 쉬움) |
| **J1 실제 스윙** | ❌ | ✅ `/joint_states` |
| 실제 관절 이동량 (예상치가 아닌) | ❌ | ✅ |
| 명령값 대비 추종 오차 | ❌ | ✅ `/arm_group_controller/controller_state` |
| 단계별 정확한 소요 시간 | 근사 | ✅ |
| 속도·가속 프로파일 | ❌ | ✅ (미분) |
| octomap 상태 | ❌ | ✅ `/monitored_planning_scene` |

`[시각화 제안]` **2열 대조표**를 그대로 그림으로. 왼쪽 "텍스트 로그"에 ❌가 몰려 있고 오른쪽 "rosbag"이 전부 ✅인 대비가 메시지다. 가운데에 "J1 스윙 = 리팩터의 핵심 근거인데 측정 불가였음"을 강조 박스로.

---

## 1. 기록 방법

### 명령

```bash
# 터미널 A — 스택 (enable_octomap:=false 가 중요, 아래 함정 참고)
ros2 launch mycobot_280_moveit2 demo_octomap.launch.py enable_octomap:=false

# 터미널 B — 기록
./scripts/record_sweep.sh              # bags/sweep_MMDD_HHMM/
./scripts/record_sweep.sh my_test      # 이름 지정

# 터미널 C — 파지 노드
ros2 run mycobot_280_pick coord_to_goal_node

# 터미널 D — 스윕 (27점, 약 20분)
./scripts/sweep_target_z.sh
```

끝나면 **터미널 B에서 Ctrl+C** — bag은 닫힐 때 `metadata.yaml`을 쓰므로, 그 파일이 있어야 정상 마감이다.

**B와 C의 순서는 바뀌어도 된다.** `--all-topics`는 기록 시작 시점에 없던 토픽도 나중에 자동으로
붙는다(실측 검증: 기록 6초 뒤 만든 토픽이 19개 메시지와 함께 잡혔다). 다만 B를 먼저 두면
`coord_to_goal_node`의 기동 로그(파라미터 값 등)까지 남는다.

### 무엇을 담고 무엇을 빼는가 — 실측으로 정했다

10초 시범 기록의 토픽별 용량을 재서 정했다. 그냥 전부 담으면 **71 MB/분**이다.

| 제외 대상 | 비중 | 이유 |
|---|---|---|
| `/camera/**` (22개 토픽) | — | 초당 수십 MB. 1번 용도엔 불필요 |
| `/controller_manager/statistics/*` | 32% | 순수 진단용 |
| `/filtered_cloud` | 26% | **메시지 1개가 3.4 MB** (포인트클라우드) |
| `/controller_manager/introspection_data/*` | 15% | 진단용 |
| `/dynamic_joint_states` | 4% | `/joint_states`와 중복 |

→ **71 MB/분 → 10 MB/분.** 27점 스윕(약 21분)이 **239 MB**.

**남긴 것 중 중요한 것:**

| 토픽 | 왜 필요한가 |
|---|---|
| `/joint_states` | 실제 관절 궤적 100 Hz — **J1 스윙의 출처** |
| `/arm_group_controller/controller_state` | 명령값과 실제값을 같이 담음 → 추종 오차 |
| `/rosout` | 노드 로그 전체. 타임스탬프·레벨·노드명이 필드로 분리 |
| `/tf`, `/tf_static` | flange 실제 위치 추적, RViz 재생에 필수 |
| `/robot_description(_semantic)` | transient_local QoS. 빼면 재생 시 RViz에 로봇 모델이 안 뜬다 |
| `/monitored_planning_scene` | **octomap 상태가 담기는 유일한 토픽** (아래 함정 2 참고) |

`[시각화 제안]` **용량 파이차트 + 전후 막대**. 파이는 제외 전 구성(statistics 32% / filtered_cloud 26% / introspection 15% / 기타), 옆에 "71 → 10 MB/분" 막대 두 개. 남긴 토픽은 별도 리스트 박스로.

---

## 2. 재생 방법과 세 가지 함정

### 명령

```bash
# 1) 실제 스택·sync_plan 모두 종료 확인 (함정 1)
ps aux | grep -E "move_group|ros2_control_node" | grep -v grep   # 비어야 함

# 2) 시각화만
rviz2

# 3) 재생
ros2 bag play bags/sweep_clean --clock
ros2 bag play bags/sweep_clean --rate 0.2          # 5배 느리게
ros2 bag play bags/sweep_clean --start-offset 300  # 특정 사이클로 점프
```

RViz에서 Fixed Frame을 `g_base`로, **RobotModel**의 Description Topic을 `/robot_description`으로
지정하면 팔이 움직인다. `TF`와 `MarkerArray`(`/display_planned_path`)를 추가하면 계획 경로도 보인다.

### 함정 1 — 스택이 떠 있는 채로 재생하면 발행자가 둘이 된다

`ros2 bag play`가 `/joint_states`를 발행하는데 실제 스택도 발행 중이면 **서로 다른 값이 교대로**
나간다. 이건 가상의 위험이 아니라 2026-07-31에 실제로 겪은 사고다 — 스택이 두 벌 돌면서
`/joint_states`에 두 값이 섞였고(942개 수신 중 526 대 416), `sync_plan`이 둘을 구분하지 못하고
전부 실물로 중계해 팔이 엉뚱하게 움직였다.

**재생 전에 스택과 `sync_plan`을 반드시 모두 끌 것.**

### 함정 2 — `enable_octomap`의 기본값은 `true`다

플래그 없이 `demo_octomap.launch.py`를 띄우면 octomap이 켜지고, 카메라가 실제 장면을 장애물로
채운다. **eye-in-hand라 팔 자신의 링크가 장애물로 찍힌다:**

```
Found a contact between '<octomap>' (type 'Object') and 'joint5' (type 'Robot link')
Found a contact between '<octomap>' (type 'Object') and 'gripper_right1' (type 'Robot link')
Trajectory component 'plan' is invalid for waypoint 73 out of 102
```

실행 중 경로가 무효화되어 `STATUS_ABORTED`가 나고, 한 스윕이 **24건 중 16건 실패**로 전멸했다.
코드 문제가 아니라 런치 설정 차이였다. FakeSystem 스윕에는 `enable_octomap:=false`를 쓸 것.

### 함정 3 — 진단에 필요한 토픽을 용량 때문에 빼면 원인을 못 찾는다

위 octomap 사고를 진단할 때, 처음에 `/monitored_planning_scene`을 6% 용량 때문에 제외했었다.
그게 **octomap 상태가 담기는 유일한 토픽**이라 "planning scene에 octomap 0개"라는 잘못된 결론을
냈고, `/rosout`의 충돌 메시지를 뒤져서야 원인을 찾았다. 지금은 제외 목록에서 빼두었다.

`[시각화 제안]` **함정 3개를 경고 카드 3장**으로. 각 카드에 아이콘 + 증상 + 원인 + 대처. 함정 1은 "발행자 2개 → 값이 교대로" 도식(두 화살표가 하나의 토픽으로 합류), 함정 2는 octomap voxel이 팔 자신을 덮는 단면도가 있으면 좋다.

---

## 3. 분석 방법

### 명령

```bash
python3 scripts/analyze_sweep_bag.py bags/sweep_clean --csv result.csv
```

`/rosout`에서 사이클 경계·채택 고도각·예상 이동량을 뽑고, `/joint_states`와 시간축에서 겹쳐
사이클별 관절 거동을 계산한다.

### 측정 구간을 잘못 잡으면 숫자가 무의미해진다

첫 분석에서 "J1 평균 101.7°"라는 값을 얻었는데, 이건 **사이클 전체(나갔다 돌아오는 왕복 포함)의
max−min**이었다. 예측치(정렬 자세 하나에 필요한 J1 도달각)와 비교 대상이 아니다.

**단계별로 잘라야 한다.** `/rosout`의 단계 전환 로그를 경계로 쓴다:

| 구간 | 시작 로그 | 끝 로그 |
|---|---|---|
| 1/5 정렬 | `[1/5 정렬] 목표 위치로 플래닝` | `[1/5 정렬] 완료` |
| 3/5 접근 | `[3/5 직진 접근] 목표 지점으로` | `[3/5 직진 접근] 완료` |
| 5/5 후퇴 | `[5/5 후퇴] 정렬 위치로` | `[5/5 후퇴] 완료` |
| 복귀 | `look pose로 복귀 중` | `look pose 복귀 완료` |

`[시각화 제안]` **잘못된 측정 vs 올바른 측정 대조도**. 위: 사이클 전체를 한 덩어리로 보고 max−min을 재는 그림(값 101.7°). 아래: 4구간으로 잘라 정렬 구간만 보는 그림(값 40.1° / 127.1°). "무엇을 재느냐가 결론을 바꾼다"가 메시지.

---

## 4. 첫 측정 결과 — 예측이 절반만 맞았다

**조건**: FakeSystem, `enable_octomap:=false`, 목표 (0.24, −0.02, z), z = 0.40 → 0.14
**결과**: 발행 27개 중 24개 수신(3개는 발행 단계 유실), **24/24 전부 성공**, 정렬 재시도 0건
**bag**: 239 MB / 1295초 / 421,911 메시지

### 단계별 관절 거동

| 구간 | J1 변화폭 | J1 도달각 | 6축 이동량 합 |
|---|---|---|---|
| **1/5 정렬** | 39.8 ~ 146.4° (평균 94.7) | 32.3 ~ 139.0° | 평균 **520°** |
| 3/5 접근 | 0.3 ~ 15.8° (평균 9.9) | — | 평균 **77°** |
| 5/5 후퇴 | 7.1 ~ 11.4° (평균 10.3) | — | 평균 **55°** |
| 복귀 | 43.0 ~ 139.6° (평균 95.7) | −7.5° (look pose) | 평균 **542°** |

**접근·후퇴는 매우 안정적이다.** J1이 거의 안 돌고(평균 10°) 6축 이동량도 77°/55°로 작다.
접근축 리팩터가 노린 "짧고 곧은 직선 이동"이 실제로 그렇게 되고 있다.

### 핵심 발견 — J1 도달각이 두 무리로 완전히 갈린다

정렬(1/5)이 끝난 시점의 J1 실제 각도를 보면 중간값이 없다:

| 무리 | J1 도달각 | 사이클 | 정렬 6축 이동량 | 예측(34°) 대비 |
|---|---|---|---|---|
| **A — 정상 분기** | **32.3 ~ 46.9°** (평균 40.1) | **11개** | 평균 453° | **일치** ✅ |
| **B — 뒤로 감는 분기** | **121.9 ~ 139.0°** (평균 127.1) | **13개** | 평균 577° | 약 3배 ❌ |

look pose의 J1이 −7.5°이므로 A무리의 스윙은 40~54°다. **예측한 도달각 34°와 맞는다 — 손목
오프셋 관계식은 유효하다.**

문제는 B무리다. 팔을 **뒤로 감아 도달하는 IK 분기**를 골랐고, **전체의 54%(13/24)**가 여기 속한다.

목표 높이와 무관하게 섞여 있다:

```
A(정상):  0.19 0.20 0.24 0.25 0.26 0.31 0.33 0.35 0.36 0.37 0.39
B(뒤로):  0.14 0.15 0.16 0.17 0.18 0.22 0.23 0.28 0.29 0.30 0.32 0.34 0.38
```

### 왜 걸러지지 않았나

`compute_ik`가 예상한 관절 이동량은 A·B가 거의 같다(380~650°). 즉 **현재 선택 기준이 이 차이를
구분하지 못한다.** 두 가지 가설이 있다:

1. **`compute_ik`가 반환한 해와 OMPL이 실제로 실행한 해가 다르다.** 정렬 구간 6축 이동량 실측
   평균 520°가 예상치 435°와 어긋나고, 무엇보다 J1 도달각이 예상과 무관하게 갈린다
2. **선택 기준이 `_angle_delta`(−π~π wrap)를 쓴다.** J1 +130°와 +35°의 차이를 제대로 반영하지
   못할 수 있다

이 코드에는 이미 *"성급한 채택 시 J1이 150° 도는 분기를 집는 함정"*이라는 주석이 있다.
**그 함정이 실제로 절반 넘게 발생하고 있었고, 이번 측정으로 처음 드러났다.**

### 개선 여지

B무리를 없애면 정렬 구간 6축 이동량이 **577° → 453°(약 21% 감소)**한다. 사이클 시간에서
정렬 20% + 복귀 42% = **62%가 이 구간**이므로 체감 효과가 크다.

`[시각화 제안]` 이 절이 가장 중요하다.
(a) **J1 도달각 산점도** (핵심). x축 목표 z(0.14~0.39), y축 J1 도달각(0~150°). 점 24개를 찍으면 32~47°와 122~139° 두 띠로 완전히 갈리고 **가운데가 텅 빈다**. 예측선 34°를 수평 점선으로 그으면 A무리가 그 위에 놓인다. 색은 A=파랑, B=빨강.
(b) **두 IK 분기 개념도**: 위에서 본 평면도 두 장. 왼쪽 A무리(J1 +40°로 정상적으로 목표를 향함), 오른쪽 B무리(J1 +127°로 팔을 뒤로 감아 반대편에서 도달). 같은 목표점에 도달하는 두 자세임을 강조.
(c) **구간별 관절 이동량 막대**: 정렬 520 / 접근 77 / 후퇴 55 / 복귀 542. 정렬+복귀가 압도적임이 보이게. 정렬 막대는 A(453)/B(577)로 쪼개서 개선 여지를 표시.
(d) **예측 vs 실측 대조**: "예측 34°" 하나에 대해 A무리(40.1° ✅)와 B무리(127.1° ❌)를 나란히.

---

## 5. 앞으로의 활용 계획 (아직 미구현)

이번에 한 것은 **1단계**다. 로봇과 토마토 베드를 항상 점유할 수 없으므로, 노트북만으로
튜닝할 수 있는 범위를 넓히는 것이 목표다.

| 단계 | 내용 | 상태 |
|---|---|---|
| **1** | FakeSystem 스윕을 기록·분석 | ✅ 0~4절 |
| **2** | 실물 세션에서 토마토 베드 장면(카메라·octomap)을 기록 → 노트북에서 재생 | ✅ **6절** (2026-07-31) |
| **3** | 재생된 장면에서 YOLO가 검출한 실제 토마토를 목표로 1번과 같은 분석 | 부분 — 검출 좌표 확보 완료, 스윕 미구현 |

3단계의 입력이 될 검출 데이터는 이미 파일로 굳혀 두었다(`bags/lab_bed_detections.json`,
6.6절). 남은 것은 그 좌표 목록을 목표로 삼아 도는 스윕 스크립트다 —
`scripts/sweep_bed_offline.py`가 격자를 스스로 생성하는 구조라, 목표 목록을 파일에서
읽는 옵션을 붙이면 된다.

### 2·3단계의 근본 제약 — 카메라가 eye-in-hand다

`camera_link`는 `joint6`에 얹힌 static TF(핸드-아이 캘리브레이션)다. 즉 **녹화된 카메라 데이터는
녹화 당시의 팔 자세에서만 유효하다.** 재생 중 시뮬 팔이 움직이면 카메라 TF도 따라 움직이는데
영상은 옛 자세의 것이라 좌표 정합이 깨진다.

다행히 설계가 절반을 해결해 둔다 — YOLO는 팔이 look pose에 있을 때만 판단한다(허용 0.08 rad).
팔이 움직이는 동안의 검출은 자동으로 버려진다.

**남는 문제는 octomap이다.** 포인트클라우드는 게이팅 없이 계속 들어가므로, 팔이 움직인 뒤에도
옛 영상이 새 TF로 변환되어 엉뚱한 위치에 voxel이 쌓인다. 해법은 셋 중 하나다:

1. `enable_octomap:=false`로 두고 장애물 회피를 튜닝 대상에서 제외 (가장 간단)
2. octomap이 형성되면 bag 재생을 일시정지 (수동)
3. `pointcloud_tomato_filter_node`에 YOLO와 같은 look pose 게이팅을 추가 (근본 해법,
   **실물에서도 이득** — 지금은 실물에서도 이동 중 클라우드가 octomap을 오염시키고 있을 것)

`[시각화 제안]` **3단계 로드맵 다이어그램**. 1단계는 완료 체크, 2·3단계는 점선. 아래에 "eye-in-hand 제약" 설명도 — 녹화 시점의 카메라 위치(팔이 look pose)와 재생 중 팔이 움직였을 때의 카메라 위치를 겹쳐 그려서, 같은 영상이 다른 TF로 변환되면 voxel이 어긋난다는 것을 보이기.

---

## 6. 장면 rosbag — 실물 베드를 노트북으로 가져오기 (트랙 A, 2026-07-31 구현)

0~4절이 **팔의 움직임**을 남긴 것이라면, 이 절은 반대로 **카메라가 본 장면**을 남긴다.
목적은 로봇도 베드도 없는 노트북에서 (a) 보고서용 그림을 만들고 (b) 수확 경로를
시뮬레이션하는 것이다.

### 6.0 왜 두 트랙으로 나뉘는가

처음엔 "장면을 재생하면서 그 위에서 팔을 스윕한다"를 생각했는데, 그건 안 된다 —
5절의 eye-in-hand 제약 때문이다. 재생 중 시뮬 팔이 움직이면 카메라 TF가 따라 움직이는데
영상은 옛 자세의 것이라 좌표가 깨진다. 그래서 목적을 둘로 쪼갠다:

| 트랙 | 목적 | 팔 | 카메라 데이터 |
|---|---|---|---|
| **A** | 보고서용 그림·영상 | look pose 고정 | bag 재생 |
| **B** | 대량 스윕·히트맵 | 자유롭게 이동 | 안 씀 (검출 좌표 JSON만) |

**다만 이 제약은 생각보다 약하다.** 이 로봇은 look-then-move다 — look pose에서 한 번 보고
그 목록으로 끝까지 수확한다. 즉 look pose 이후 카메라는 할 일이 없다. `points_filtered`
발행을 look pose 구간에만 허용하면(5절 해법 3) octomap이 그 시점 스냅샷으로 얼어붙고,
그 뒤로는 팔이 움직여도 오염되지 않는다. **입력을 끊는 것만으로 스냅샷이 된다** —
MoveIt의 octomap은 새 클라우드가 광선으로 지우기 전에는 voxel을 유지하기 때문이다.
이건 재생용 편의가 아니라 **실물 신뢰도 개선**이기도 하다. 지금 실물에서도 이동 중
클라우드가 octomap을 오염시키고 있고, `enable_octomap:=false` 플래그를 만든 원인이었던
"그리퍼 근접 self-filter 잔여 voxel → `START_STATE_IN_COLLISION`"의 발생 경로가
바로 이것이다. (아직 미구현.)

### 6.1 무엇을 담고 무엇을 빼는가 — 다시 실측으로 정했다

20초 시범 기록(`PROFILE=full`)의 토픽별 **실제 직렬화 바이트**를 합산했다. 합계 87.4 MB/s:

| 토픽 | MB | 비중 | Hz | KB/장 | |
|---|---|---|---|---|---|
| `color/image_raw` | 482 | 28.8% | 27.3 | 921.7 | |
| `depth/color/points_filtered` | 428 | 25.6% | 4.5 | 4915.4 | **재생성 가능** |
| `tomato_detections_image` | 352 | 21.1% | 20.0 | 921.7 | **재생성 가능** |
| `aligned_depth_to_color/image_raw` | 320 | 19.2% | 27.2 | 614.5 | |
| `color/…/compressed` | 50 | 3.0% | 26.7 | 97.9 | raw의 **1/9.4** |
| `aligned_…/compressedDepth` | 37 | 2.2% | 26.9 | 72.0 | raw의 **1/8.5** |

**핵심 설계: 클라우드가 아니라 그 입력을 담는다.** 클라우드와 검출영상이 46.7%인데 둘 다
원재료(depth + color + camera_info + bbox)만 있으면 재생 시 노드를 다시 돌려 만들 수 있다.
용량이 줄 뿐 아니라 **`bbox_padding_ratio` 같은 파라미터를 오프라인에서 다시 튜닝**할 수
있다 — 녹화된 클라우드로는 불가능한 일이다.

| 프로파일 | 속도 | 분당 |
|---|---|---|
| full (raw + 클라우드 + 검출영상) | 87.4 MB/s | 5.2 GB |
| **slim (압축본만)** | **5.2 MB/s** | **312 MB** |

**17배.** `compressedDepth`는 16UC1을 PNG로 담아 **무손실**이라 depth 값이 그대로 보존된다
(포인트클라우드 좌표가 정확히 재현됨). 컬러는 JPEG이라 손실이 있지만 YOLO 입력으로는
충분하다 — 실제 카메라들도 JPEG로 스트리밍한다.

`[시각화 제안]` **full vs slim 막대 2개(87.4 / 5.2 MB/s)** + 옆에 full의 구성 파이차트.
"재생성 가능"인 두 조각(46.7%)을 다른 색으로 칠하고 화살표로 빼내는 그림이면 설계 의도가 바로 보인다.

### 6.2 기록 방법

**개별 명령** (터미널 4개, 실물 카메라 필요)

```bash
# A — 스택 (카메라·octomap·필터 노드 포함)
ros2 launch mycobot_280_moveit2 demo_octomap.launch.py enable_octomap:=true

# B — YOLO. target_point 리맵이 중요하다(아래 참고)
ros2 run mycobot_280_pick yolo_d435_detector_node --ros-args -r target_point:=target_point_dryrun

# C — 모든 노드가 뜬 뒤 octomap 한 번 비우기 (함정 8)
ros2 service call /clear_octomap std_srvs/srv/Empty

# D — 기록
./scripts/record_scene.sh bed_0801 30      # slim, 30초
PROFILE=full ./scripts/record_scene.sh cmp 20   # 용량 비교용
```

**`coord_to_goal_node`를 띄우면 안 된다.** 그게 있으면 YOLO가 look pose에서 발행한
`target_point`를 받아 팔이 자동으로 움직이고, eye-in-hand라 녹화 장면이 깨진다. YOLO의
출력을 죽은 토픽으로 리맵하면 `tomato_boxes`/`tomato_candidates`는 그대로 나오면서
움직임만 차단된다.

**런치 한 줄** — 위 A~D를 전부 대신한다:

```bash
ros2 launch mycobot_280_pick scene_record.launch.py record:=true bag_name:=bed_0801 duration:=30
```

### 6.3 재생 방법

**개별 명령** (터미널 6개)

```bash
# 1 — 스택. 카메라·필터 노드·RViz를 전부 끈다
ros2 launch mycobot_280_moveit2 demo_octomap.launch.py \
    enable_octomap:=true enable_camera:=false enable_pointcloud_filter:=false use_rviz:=false

# 2,3 — 압축본을 raw로 복원. in_transport는 반드시 파라미터로(함정 4)
ros2 run image_transport republish --ros-args \
    -p in_transport:=compressed -p out_transport:=raw \
    -r in/compressed:=/camera/camera/color/image_raw/compressed \
    -r out:=/camera/camera/color/image_raw
ros2 run image_transport republish --ros-args \
    -p in_transport:=compressedDepth -p out_transport:=raw \
    -r in/compressedDepth:=/camera/camera/aligned_depth_to_color/image_raw/compressedDepth \
    -r out:=/camera/camera/aligned_depth_to_color/image_raw

# 4 — 클라우드 재생성 2개. restamp_now가 필수다(함정 6)
ros2 run mycobot_280_pick pointcloud_tomato_filter_node --ros-args -p restamp_now:=true
ros2 run mycobot_280_pick pointcloud_tomato_filter_node --ros-args \
    -r __node:=pointcloud_unmasked_node \
    -p output_topic:=/camera/camera/depth/color/points_unmasked \
    -p boxes_topic:=__no_boxes__ -p publish_rate_hz:=2.0 -p restamp_now:=true

# 5 — YOLO
ros2 run mycobot_280_pick yolo_d435_detector_node --ros-args -r target_point:=target_point_dryrun

# 6 — 재생. /tf_static 포함, /tomato_boxes·/joint_states·/tf 제외 (함정 5)
ros2 bag play bags/bed_look_slim --loop --topics \
    /camera/camera/color/image_raw/compressed \
    /camera/camera/aligned_depth_to_color/image_raw/compressedDepth \
    /camera/camera/color/camera_info \
    /camera/camera/aligned_depth_to_color/camera_info \
    /tf_static

# 다 뜬 뒤 한 번 (함정 8)
ros2 service call /clear_octomap std_srvs/srv/Empty
rviz2 -d src/mycobot_280_pick/config/scene_replay.rviz
```

**런치 한 줄:**

```bash
ros2 launch mycobot_280_pick scene_replay.launch.py
ros2 launch mycobot_280_pick scene_replay.launch.py bag:=bags/bed_0801 rate:=0.5
ros2 launch mycobot_280_pick scene_replay.launch.py rate:=0.25
```

실측 검증: 이 런치 하나로 `color/image_raw` 29.5Hz, `aligned_depth` 29.1Hz,
`tomato_boxes` 29.0Hz, `points_filtered` 4.5Hz, `points_unmasked` 2.0Hz가 모두 흐르고,
`points_filtered` 발행자는 1개, 23초 시점에 "Octomap cleared."가 찍힌다.

### 6.4 재생 함정 4~8 (전부 실제로 밟았다)

**함정 4 — `republish`의 `in_transport`는 위치 인자가 아니라 파라미터다.**
`ros2 run image_transport republish compressed --ros-args ...`처럼 주면 그 값이 무시되고
기본값 `raw`가 된다. 그러면 압축 토픽이 아니라 raw 토픽을 구독하는데 거기엔 아무도 발행하지
않으니 **에러 한 줄 없이 조용히 아무 일도 안 일어난다.** 기동 로그의
`The 'in_transport' parameter is set to: compressed`를 확인할 것.

**함정 5 — `/tf_static`을 재생하지 않으면 클라우드가 안 그려진다.**
`camera_link → camera_*_optical_frame`은 realsense 드라이버가 발행하는 static TF다.
재생하려고 드라이버를 끄면 그 TF가 같이 사라진다. 반대로 `/tomato_boxes`는 **빼야** 한다
(살아있는 YOLO와 발행자 충돌), `/joint_states`·`/tf`도 빼야 한다(2절 함정 1과 같은 사고;
녹화 중 팔은 정지해 있었으므로 재생할 이유도 없다).

**함정 6 — bag은 `header.stamp`를 녹화 당시 값 그대로 내보낸다.** 실측 **1788.8초**(약 30분)
과거였다. TF는 살아있는 스택이 현재 시각으로 발행하므로 tf2 버퍼(기본 10초)를 벗어난다.
증상이 고약하다:

- RViz PointCloud2가 **Error**가 되고 **Position/Color Transformer가 둘 다 빈칸**으로 남는다
  → "Color Transformer 문제"로 오해하기 쉽다. 실제로 그렇게 오진했다.
- occupancy_map_monitor도 octomap을 못 쌓는다. 그런데 이전 라이브 세션의 voxel이 남아 있으면
  **쌓이는 것처럼 보인다** — `/clear_octomap`을 해봐야 드러난다.
- Image 디스플레이는 TF가 필요 없어 **멀쩡히 보인다** → 더 헷갈린다.

정석은 `ros2 bag play --clock` + 모든 노드 `use_sim_time`이지만 move_group과 FakeSystem
컨트롤러까지 전부 재시작해야 해서 과하다. 장면이 정지해 있고 팔이 녹화 당시 자세에
그대로 있는 재생이라면 `pointcloud_tomato_filter_node`의 **`restamp_now:=true`**로 충분하다
(출력 stamp를 현재로 바꾼다). 적용 후 지연 **0.42초**. 단 팔이 움직이는 중에 켜면 안 된다.

**함정 7 — `demo_octomap.launch.py`가 필터 노드를 자동으로 띄운다.**
`enable_octomap:=true`면 기본 파라미터(=`restamp_now` 꺼짐)로 하나가 뜬다. 재생용으로 직접
띄운 것과 합쳐 `points_filtered`에 **발행자가 둘**이 되고, 런치가 띄운 쪽은 옛 타임스탬프를
내보낸다. `enable_pointcloud_filter:=false`로 끌 것(octomap 본체는 유지된다).

**함정 8 — 마스킹은 사전 차단이지 사후 삭제가 아니다.**
octomap voxel은 **광선이 그 자리를 통과할 때만** 지워진다. 그런데 마스킹은 토마토 픽셀의
포인트를 아예 없애므로 그 방향 광선도 사라진다. 즉:

> 토마토 voxel이 **한 번이라도 들어가면** 이후 마스킹이 완벽해도 영원히 안 지워진다.

필터 노드는 YOLO보다 먼저 뜨고 그동안 `_boxes`가 비어 있어 마스킹 없는 클라우드를 먼저
내보내므로, 이 일은 거의 항상 일어난다. **"토마토 자리에 구멍이 안 난다"의 실제 원인이
이것이었다** — 클라우드는 정상적으로 26.3%(80,818개)를 지우고 있었다.
**모든 노드가 뜨고 `tomato_boxes`가 흐른 뒤 `/clear_octomap`을 한 번 부를 것.**

`[시각화 제안]` **함정 4~8을 경고 카드 5장**으로(2절 카드 3장의 속편). 함정 6은 특히
"증상(색 문제처럼 보임) → 실제 원인(타임스탬프)" 화살표 도식이 좋다. 함정 8은 광선이
토마토를 통과하지 못해 voxel이 남는 단면도.

### 6.5 마스킹 검증과 `bbox_padding_ratio`

마스킹이 실제로 되는지는 두 클라우드를 비교하면 즉시 나온다(같은 프레임 기준):

| 클라우드 | 유효 포인트 |
|---|---|
| `points_unmasked` | 234,456 / 307,200 (76.3%) |
| `points_filtered` | 158,501 / 307,200 (51.6%) |
| **마스킹으로 지워진 것** | **80,818개 (26.3%)** |

**`points_filtered`로는 토마토 위치를 못 찾는다.** 정의상 토마토가 지워진 클라우드라
octomap과 똑같이 그 자리가 비어 있다. 그래서 필터 노드를 `boxes_topic=__no_boxes__`로
하나 더 띄워 마스킹 안 된 컬러 클라우드를 만들고, RViz에서 octomap과 토글해 본다.

그리고 현재 `bbox_padding_ratio = 0.5`는 **과하다**. 같은 19개 bbox로 잰 마스킹 면적:

| padding | 마스킹 면적 |
|---|---|
| 0.0 | 11.8% |
| 0.2 (원래 값) | 21.7% |
| **0.5 (현재 값)** | **31.5%** |

토마토뿐 아니라 **줄기·지지대까지 octomap에서 사라진다** — 그것들은 실제로 피해야 할
장애물이다. "눈에 잘 안 보여서" 0.2에서 0.5로 올린 값인데, 대가가 이 정도였다는 것이
이번에 처음 수치로 나왔다. slim bag이 있으니 오프라인에서 다시 튜닝할 수 있다.

`[시각화 제안]` **마스킹 면적 3단 비교**(0.0/0.2/0.5의 자홍색 오버레이 이미지 3장 나란히).
0.5에서 화면 중앙이 통째로 지워지는 게 한눈에 보인다.

### 6.6 검출 좌표를 파일로 뽑기 — 트랙 B의 입력

```bash
python3 scripts/export_detections.py --out bags/lab_bed_detections
```

`tomato_candidates`를 받아 TF로 `g_base`로 변환해 JSON/CSV로 저장한다. **ROS 없이 읽힌다.**
담기는 것: 클래스(ripe/unripe/rotten/disease), 카메라·g_base 3D 좌표, 추정 반지름,
confidence, 관측 횟수. 이를 위해 `tomato_candidates`의 stride를 5 → **7**로 늘렸다
(`radius_m`, `count` 추가). `harvest_sequence_node`의 파서도 같이 갱신했다.

**실험실 실측(2026-07-31)**: 토마토 15개(ripe 10, disease 5), 반경 0.248~0.293 m,
방위각 −14.2°~+19.5°, 높이 0.161~0.361 m, 추정 반지름 13~18 mm.

**함정 — YOLO 워밍업.** `tomato_candidates`는 look pose 도착 후 5초 누적창이 끝나는 순간
**1회만** 발행된다. 그런데 노드 기동 시 팔이 이미 look pose에 있으면 누적창이 즉시 시작되고,
ultralytics **첫 추론 워밍업**이 그 창을 다 먹는다 — 실측으로 검출 **16개(프레임 1장),
관측 1회**만 잡혔고 타이머도 5초가 아니라 12.5초 뒤에 울렸다. 워밍업 후 재유발하면
**검출 905개, 관측 57회**가 된다.

> 다행히 좌표 자체는 두 경우가 **1 mm 이내로 일치**했다(정지된 장면이라). 관측 횟수가
> 한 자리면 재수집하되, 급하면 그대로 써도 된다.

팔을 움직이지 않고 재판단을 유발하려면 YOLO를 `-r joint_states:=joint_states_warm`으로
띄우고, 워밍업 후 별도 노드가 `joint_states_warm`에 **look pose 이탈 값 → 실제 값** 순으로
흘려 주면 된다. 이때 **전 관절을 틀어야 한다** — 인덱스 0번만 틀었더니 그 자리가 그리퍼
관절이라 `_is_near_look_pose`가 보는 6개 팔 관절은 그대로여서 잠금이 안 풀렸다.
이탈 값 발행은 DDS 디스커버리보다 길어야 한다(2.5초는 짧아서 한 장도 전달되지 않았다).

---

## 7. 런치 파일 구성 방법

6.2·6.3의 수동 절차를 그대로 옮긴 것이다. **`scene_record.launch.py`(기록)**,
**`scene_replay.launch.py`(재생)** 둘 다 `src/mycobot_280_pick/launch/`에 있다.

### 7.1 전제 — `demo_octomap.launch.py`에 인자 3개를 추가해야 했다

이 런치가 무조건 하던 일 셋을 끌 수 있어야 한다. 셋 다 재생 세션에서만 문제가 된다.

| 인자 | 기본값 | 왜 필요한가 |
|---|---|---|
| `use_rviz` | `true` | MoveIt 기본 RViz를 띄운다. 끄지 않으면 `scene_replay.rviz`와 **창이 두 개** 뜬다 |
| `enable_camera` | `true` | realsense를 띄운다. 재생 시 발행자 이중화, 집에서는 D435가 없어 오류 |
| `enable_pointcloud_filter` | `true` | 필터 노드를 띄운다 → 함정 7 |

`use_rviz`는 `moveit_configs_utils`의 `generate_demo_launch`가 이미 선언하지만, 그건
`OpaqueFunction` **안쪽**에서 만들어지는 `LaunchDescription`이라 최상위 인자 목록에 안 잡힌다.
명령줄에 `use_rviz:=false`를 주면 `Parameter 'use_rviz' is not supported` 경고만 뜨고 RViz가
그대로 떴다(실측 확인). **같은 이름으로 최상위에 한 번 더 선언**하면 CLI 값이 컨텍스트에 먼저
들어가고, 안쪽 `DeclareLaunchArgument`는 이미 설정된 값을 덮어쓰지 않으므로 의도대로 동작한다.

`enable_octomap`과 `enable_pointcloud_filter`를 **분리한 이유**는, 재생 세션에서
"octomap 본체(`occupancy_map_monitor`)는 켜고 필터 노드만 끄는" 조합이 필요하기 때문이다.
하나의 플래그로 묶여 있으면 만들 수 없는 조합이다.

### 7.2 `scene_replay.launch.py` 액션 구성

| # | 액션 | 요점 |
|---|---|---|
| 1 | `IncludeLaunchDescription(demo_octomap)` | `enable_octomap:=true`, 나머지 3개는 `false` |
| 2 | `Node` × 2 — `image_transport/republish` | `parameters=[{'in_transport': ...}]` — 함정 4가 구조적으로 불가능해진다 |
| 3 | `Node` × 2 — 필터 | 둘 다 `restamp_now: True`. 하나는 `output_topic`/`boxes_topic`/`publish_rate_hz`를 바꿔 비마스킹본 |
| 4 | `Node` — YOLO | `remappings=[('target_point', 'target_point_dryrun')]` |
| 5 | `ExecuteProcess` — `ros2 bag play` | 토픽 목록이 파일에 박혀 함정 5가 봉쇄된다 |
| 6 | `TimerAction(clear_delay)` → `ExecuteProcess` | `/clear_octomap` 1회 — 함정 8 |
| 7 | `Node` — `rviz2` | `FindPackageShare`로 `config/scene_replay.rviz` |

`coord_to_goal_node`는 **포함하지 않는다**. 팔 자동 이동 차단이 목적이고, 목표 지점에 등록되는
반지름 0.05 m 초록색 구가 토마토를 가리는 문제도 같이 없어진다.

### 7.3 구현 시 걸린 것 셋

**`IncludeLaunchDescription`의 `launch_arguments`는 부모 스코프를 오염시킨다.**
가장 오래 걸린 함정이다. `scene_replay.launch.py`는 자기 `use_rviz`(기본 `true`)로 RViz를
띄우면서, 자식 `demo_octomap.launch.py`에는 `use_rviz:=false`를 넘겨 MoveIt 기본 RViz를
막는다. 그런데 **include는 스코프를 만들지 않아서** 그 `false`가 최상위 `use_rviz`까지
덮어썼고, 뒤이어 평가되는 rviz2 노드의 `IfCondition`이 거짓이 되어 **RViz가 전혀 뜨지 않았다
— 에러 한 줄 없이.** 노드가 죽은 게 아니라 애초에 생성되지 않으니 로그에 아무 흔적이 없다.

```python
ld.add_action(GroupAction(scoped=True, actions=[
    IncludeLaunchDescription(..., launch_arguments={'use_rviz': 'false'}.items())
]))
```

`GroupAction(scoped=True)`로 감싸면 격리된다. **같은 이름의 인자를 자식에게 다른 값으로
넘길 때 항상 생기는 문제**이므로, 이름 충돌이 없더라도 습관적으로 감쌀 것.



**`IfCondition(['not ', LaunchConfiguration('loop')])`은 동작하지 않는다.** launch가
`"not true"`를 불린으로 해석하지 못해 `InvalidConditionExpressionError`가 난다.
`UnlessCondition`을 쓸 것.

**`setup.py`에 `config/`가 없었다.** `data_files`에 `('share/<pkg>/config', glob('config/*.rviz'))`를
추가해야 `FindPackageShare`로 RViz 설정을 찾을 수 있다. 상대경로로 `rviz2 -d src/...`를
직접 실행할 때는 드러나지 않던 문제다.

### 7.4 설계상 한계 — `clear_delay`가 고정 시간이다

"`tomato_boxes`가 흐르기 시작하면"이라는 조건을 런치가 직접 감시할 수 없어 타이머로 근사한다.
기본 25초는 YOLO 모델 로드(0.4초) + 첫 추론 워밍업(수초) + 노드 기동에 여유를 얹은 값이고,
느린 장비면 늘리면 된다. 근본 해법은 필터 노드가 "bbox를 한 번도 못 받았으면 발행하지 않음"
옵션을 갖는 것인데, 토마토가 없는 장면에서 영원히 발행하지 않는 부작용이 있어 기본값으로 두긴
어렵다.

또 `tomato_candidates`는 이 런치로 받을 수 없다(6.6절 워밍업 함정). 좌표 데이터가 다시 필요하면
별도 절차가 필요하다.

### 7.5 `bag` 기본값은 상대경로다

`bags/bed_look_slim`은 `ros2 launch`를 실행한 **현재 디렉터리** 기준으로 풀린다(패키지 기준이
아니다). 저장소 루트에서 실행할 것. 다른 곳에서 치면 `ros2 bag play`만 실패하고 **런치 전체는
계속 돌아가서**(스택도 RViz도 정상으로 뜨고 클라우드만 안 나옴) 함정 6과 증상이 비슷하다.
어디서든 쓰려면 `bag:=/절대/경로`를 줄 것.

`[시각화 제안]` **수동 6터미널 vs 런치 1줄 대비도**. 왼쪽에 터미널 6개와 그 사이를 잇는
함정 5개(4~8) 표시, 오른쪽에 런치 파일 1개 상자와 그 안에 액션 7개. "함정이 파일 안으로
들어갔다"가 메시지.

---

## 8. 그림 요청 요약

| 우선순위 | 그림 | 절 |
|---|---|---|
| ★★★ | **J1 도달각 산점도** (두 무리로 갈림) | 4-(a) |
| ★★★ | 두 IK 분기 개념도 (정상 vs 뒤로 감기) | 4-(b) |
| ★★ | 구간별 관절 이동량 막대 | 4-(c) |
| ★★ | 재생 함정 3장 경고 카드 | 2 |
| ★★ | 예측 vs 실측 대조 | 4-(d) |
| ★★★ | **마스킹 면적 3단 비교** (padding 0.0/0.2/0.5) | 6.5 |
| ★★ | full vs slim 용량 + "재생성 가능" 46.7% 도식 | 6.1 |
| ★★ | 재생 함정 5장 경고 카드 (함정 4~8) | 6.4 |
| ★★ | 두 트랙 분기도 (A=그림용 / B=스윕용) | 6.0 |
| ★ | 텍스트 로그 vs rosbag 대조표 | 0 |
| ★ | 용량 파이차트 + 전후 막대 | 1 |
| ★ | 잘못된 측정 vs 올바른 측정 | 3 |
| ★ | 3단계 로드맵 | 5 |
| ★ | 수동 6터미널 vs 런치 1줄 | 7.4 |

**작성 시 유의**

- **J1 도달각과 J1 변화폭은 다르다.** 도달각은 정렬이 끝난 시점의 절대 각도, 변화폭은 구간
  내 max−min이다. 산점도는 **도달각**을 쓸 것.
- look pose의 J1은 **−7.5°**다. 스윙 = 도달각 − (−7.5°).
- 고도각 부호: **음수 = 위에서 아래로 접근**, 양수 = 아래에서 위로 접근.
- **0~4절의 측정은 RViz FakeSystem이다. 6절은 실물 D435 + 실제 토마토 베드다** — 둘을
  섞어 쓰지 말 것. 6절에서 팔은 FakeSystem이지만 카메라·검출·octomap은 전부 실물이다.
- 6.5절의 마스킹 수치(26.3%, 31.5%)는 bbox 19개가 잡힌 **특정 프레임** 기준이다. 검출 개수가
  달라지면 면적도 달라진다.
- 관련 문서: 접근축 리팩터 자체는 `docs/VISUALIZATION_HANDOFF.md`, 속도 튜닝 이력은
  `docs/SPEED_TUNING_HANDOFF.md`.
