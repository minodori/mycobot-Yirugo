# 속도/부드러움 튜닝 — mycobot 280 토마토 수확

**2026-07-30 전면 개정.** 이전 판의 3·4절(정량 관계, speed 불일치 진단)은 잘못된 전제
위에 세워져 있었음이 실측으로 드러나 폐기했다. 무엇이 왜 틀렸는지는 5절에 남겨뒀다 —
같은 오류를 반복하지 않기 위함.

대상 파일 3개:

- `rpi/sync_plan.py` (RPi5, pymycobot relay) — **이 저장소가 정본**, RPi는 복사본
- `src/mycobot_280_pick/mycobot_280_pick/coord_to_goal_node.py` (PC, MoveIt2)
- `src/mycobot_ros2/mycobot_280/mycobot_280_moveit2/config/{ompl_planning,joint_limits}.yaml`
  — ⚠️ gitignore된 중첩 저장소 안. 바깥 저장소 커밋에 안 담김, 재클론 시 유실 주의

---

## 1. 결론 요약 (바쁘면 이것만)

**끊김의 근본 원인은 `send_angles()`의 블로킹이었다.** pymycobot이 응답 없는 명령에
0.5초씩 3번 재시도해서 호출당 1.5초를 잡고 있었고, 그 결과 relay가 20Hz가 아니라
**0.64Hz**로 돌면서 명령당 이동량이 0.9°가 아니라 **28°**였다. `_async=True`로
write-only 전환한 것이 이 프로젝트에서 가장 큰 개선이었다(사용자 평가 "차원이 다르게
부드러워짐").

**남은 미세 떨림은 jerk 제한으로 잡았다.** 기본 파이프라인에 Ruckig이 빠져 있어서
궤적 가속도가 불연속이었다. `AddRuckigTrajectorySmoothing` + `max_jerk: 5.0`.

**그 사이에 세운 가설 5개는 전부 실측으로 배제됐다** — 4절. 다시 시도하지 말 것.

### 현재 확정값 (2026-07-30)

| 파일 | 변수 | 값 |
|---|---|---|
| coord_to_goal_node | `VELOCITY_SCALING` / `ACCELERATION_SCALING` | **0.5** / 0.25 (리팩터 후 재조정, 6절 참고) |
| coord_to_goal_node | `APPROACH_*` / `RETREAT_*` | 0.2 / 0.2 |
| coord_to_goal_node | `tolerance_orientation` (정렬) | 0.2 |
| sync_plan | `SPEED_GAIN_K` / `SPEED_HYSTERESIS` | 1.7 / 4 |
| sync_plan | `ANGLE_EPSILON_DEG` / `THROTTLE_PERIOD_SEC` | 0.1 / 0.05 |
| sync_plan | `USE_DYNAMIC_SPEED` / `FIXED_SPEED` | True / 40 |
| sync_plan | `fresh_mode` | 1 (잠정 — 4절 참고) |
| joint_limits.yaml | `max_jerk` | **5.0** |

---

## 2. 아키텍처 — 속도가 어디서 결정되는가

```
[PC] MoveIt 플래닝
       → AddTimeOptimalParameterization  ← VELOCITY/ACCELERATION_SCALING 개입
       → AddRuckigTrajectorySmoothing    ← max_jerk 개입 (2026-07-30 추가)
       → controller 실행 → /joint_states (100Hz)
                        │  ROS2 (RPi에도 100Hz로 정상 도착 — 실측)
                        ▼
[RPi] sync_plan.listener_callback
        ├─ THROTTLE_PERIOD_SEC 로 게이트 (20Hz)
        ├─ ANGLE_EPSILON_DEG 미만 변화는 전송 생략
        ├─ speed = SPEED_GAIN_K × 실제 각속도  (+ SPEED_HYSTERESIS)
        └─ mc.send_angles(..., _async=True)   ← 논블로킹 필수
                        ▼
             서보 펌웨어: point-to-point, 명령마다 가감속 램프
```

**불변식:**

- 팔의 실제 속도는 `VELOCITY_SCALING`이 단독 결정. `/joint_states`가 이미 "언제 어느
  각도"를 담고 나온다.
- sync_plan의 `speed`는 속도가 아니라 **추종 능력**만 정한다.
- 서보는 스트리밍용이 아니다 → 명령마다 가감속 램프가 붙는다.
- **`fresh_mode=1`이 진행 중 동작을 가로채므로**, 명령이 촘촘하면 램프가 완결되지 않고
  이어져 연속 운동이 된다. 이것이 `_async` 수정이 효과를 낸 메커니즘이다.

---

## 3. 근본 원인: send_angles 블로킹 (해결됨)

`mycobot280.py`의 `_res()`는 `has_reply=False`인 명령에도 응답을 기다린다:

```python
while try_count < 3:
    self._serial_port.reset_input_buffer()
    self._write(...)
    data = self._read(genre)      # wait_time = 0.5s (Linux 기본)
    if not data or len(data) < 4:
        try_count += 1; continue   # 응답이 없으므로 3번 모두 소진
```

`MyCobot280`은 `crc_robot_class`가 아니므로 `wait_time=0.5`가 적용된다 →
**3 × 0.5 = 1.5초**. 실측 `dt=1.563s`와 일치(나머지 63ms는 write/직렬화).

| | 의도 | 실제(수정 전) | 수정 후 |
|---|---|---|---|
| relay 주기 | 20Hz | **0.64Hz** | 16.9Hz |
| 명령당 이동량 | 0.9° | **28°** | 1.08° |

**`set_color`도 같은 경로였다** — 단계 전환마다(사이클당 6회) relay가 1.5초 멈추고
있었다. 공개 API에 `_async`를 넘길 구멍이 없어 `_mesg(ProtocolCode.SET_COLOR, ...,
_async=True)`를 직접 호출한다(`sync_plan._set_color_async`). 이 수정 없이는 LED 점멸이
불가능하다(초당 여러 번 1.5초를 잡아 팔이 멈춤).

> **교훈**: pymycobot에서 새 명령을 relay 경로에 넣을 때는 그 명령이 `_async`를
> 지원하는지 먼저 확인할 것. 아니면 1.5초 블로킹이 조용히 들어온다.

---

## 4. 배제된 가설 — 다시 시도하지 말 것

`_async` 수정 후에도 남은 미세 떨림("건들건들")을 두고 세운 가설들. **전부 실측으로
배제**됐고, 각 상수의 주석에 근거가 남아 있다.

| 가설 | 검증 방법 | 결과 |
|---|---|---|
| 명령 주기가 부족 | `ANGLE_EPSILON_DEG` 0.7→0.1 (5.9Hz→17.2Hz) | ❌ 변화 없음 |
| 저속 자체가 원인 | 정렬·복귀는 speed 40, ω 18°/s로 저속이 아닌데도 떨림 | ❌ |
| speed 노이즈 | `SPEED_HYSTERESIS`=4 (변화율 50%→11.9%) | ❌ 변화 없음 |
| 명령 스트림이 진동 | 관절별 방향 반전 0%, dmax 변동계수 9% | ❌ 명령은 완전히 매끄러움 |
| 도착 후 정지 (t≈T) | `SPEED_GAIN_K` 2.22→1.7 (t/T 1.00→1.31) | ❌ 변화 없음 |

배제된 것들도 값은 유지하고 있다 — 그 자체로는 개선(전송률 균일화, speed 안정화)이고
해가 없기 때문. 되돌릴 이유는 없다.

### fresh_mode — 결론 없음

문서상 `0`=큐(순차 완결), `1`=최신 명령 우선(진행 중 중단). upstream 예제는 1,
2026-07-29에 0으로 실험, 이후 다시 1로 돌아와 현재 1. "1이 더 나빴다"는 관측이 한 번
있었으나 speed가 하드코딩이던 시절의 비통제 비교라 근거로 삼지 않는다.
**`_async`와 동적 speed가 들어온 뒤로는 이전 관측이 그대로 적용되지 않는다** —
변수를 하나씩 고정한 재비교가 필요하고, 그 전에는 양방향 모두 뒤집지 말 것.

---

## 5. 이전 판이 왜 틀렸나

**(1) 정량 관계의 전제가 30배 틀렸다.** 이전 판은 `Δθ = ω × THROTTLE_PERIOD` =
18 × 0.05 = **0.9°**로 계산했지만, 실제 `dt`는 1.563초여서 진짜 Δθ는 **27°**였다.
`THROTTLE_PERIOD_SEC`는 그때 아무 역할도 못 하고 있었다(블로킹이 실효 주기를 지배).

**(2) "부드러움 조건 t ≈ T"가 반대였다.** `t ≈ T`는 "다음 명령이 오는 순간 정확히
도착" = **매번 완주하고 멈춤**이다. 연속 운동에는 `t > T`가 필요하다. 실측에서
`SPEED_GAIN_K=2.22`일 때 t/T가 정확히 1.00으로 나온 것이 이 오류의 증거다.
(다만 1.7로 낮춰 t/T=1.31을 만들어도 떨림은 안 사라졌다 — 4절.)

**(3) "0.2/40이 왜 개선됐는지"의 램프 희석 해석은 방향만 맞았다.** Δθ가 실질
인자라는 결론은 맞지만, 당시 계산한 Δθ 값(0.45→0.9°)은 틀렸고(13.5→27°),
지금 부드러운 이유는 Δθ가 **작아서 명령이 뭉개져 이어지는** 것이다(크게 해서 램프를
희석하는 것이 아니라).

**(4) `speed ≈ 200 × VELOCITY_SCALING`은 우연히 맞았다.** 해석적으로도 같은 값이
나온다(`joint_limits.yaml` max_velocity 1.57 rad/s ≈ 90°/s, 40/18 ≈ 2.22 →
2.22 × 90 = 200). 지금은 `SPEED_GAIN_K=1.7`을 쓰므로 `speed ≈ 153 × scaling`이다.

---

## 6. 실제로 효과가 있었던 것

### (1) `_async=True` — 압도적 1위

3절 참고. 유일하게 "차원이 다르게" 수준의 개선.

### (2) jerk 제한 (Ruckig)

기본 파이프라인에 `AddRuckigTrajectorySmoothing`이 **빠져 있었다**. TOTG의 플러그인
설명이 명시적으로 `"Not jerk limited"`이므로, 궤적 가속도가 불연속이고 waypoint마다
jerk가 튄다 — 가벼운 팔에서 기계적 울림이 된다.

- `ompl_planning.yaml`을 새로 만들어 TOTG **뒤에** Ruckig 추가
- `joint_limits.yaml`에 `has_jerk_limits: true` / `max_jerk` 추가.
  **없으면 Ruckig이 경고 후 하드코딩 기본값으로 폴백하고, 그 값이 크면 아무것도
  안 깎인다** — 실제로 `max_jerk: 20.0`에서는 궤적이 거의 안 바뀌었다.
  **5.0으로 낮추자 실물에서 뚜렷한 개선.**

> ⚠️ 5.0의 효과는 **실물 육안 관측**이다. relay 로그의 가속 램프 지표로는 20.0과
> 5.0의 차이가 확인되지 않았다(17Hz 샘플링으로 2차 미분 차이가 묻히는 것으로 추정).
> 값 조정 시 로그보다 실물 체감을 기준으로 볼 것.

검증: `ros2 param get /move_group robot_description_planning.joint_limits.<관절>.max_jerk`,
그리고 `move_group` 터미널에 `Joint jerk limits are not defined` 경고가 **없어야** 한다.

### (3) `VELOCITY_SCALING` — U자 곡선, 최적점이 이동했다 (0.35 → 0.5)

**접근축 리팩터 이전** (정렬 위치 반지름 0.102m, J1 스윙 60~120°):

| 값 | 결과 |
|---|---|
| 0.2 | 건들건들 (도착-정지가 지각됨) |
| **0.35** | **최선** — 많이 줄어듦 |
| 0.5 | **전 구간**에서 흔들림 — 구조 진동 |

0.5가 나쁘다는 판단의 근거: 그때 접근·후퇴(0.2 유지) 구간의 relay 명령이 **완전히
동일**했는데도(ω 17.9, dmax 1.077, dt 표준편차 0.0046s) 흔들렸다. 명령이 그대로인데
흔들리면 원인은 명령 스트림 밖 = 기계 진동이다.

**접근축 리팩터 이후** (정렬 위치 반지름 0.131~0.146m, J1 스윙 약 34°) — **0.5가 최선**
(2026-07-30 실물 검증, 사용자 평가 "0.35는 너무 느려서. 이렇게 하니 좀 더 부드럽게
빨리 움직여").

같은 실물 조건에서 결론이 뒤집힌 것이므로, **U자 곡선 자체가 오른쪽으로 이동했다**고
읽어야 한다. 위 3차 진단이 지목한 원인이 바로 "정렬·복귀의 빠른 **대각도 스윙**이 팔
구조를 울린다"였는데, 리팩터가 그 스윙을 60~120°에서 34°로 줄여 **진동의 입력 자체를
작게 만들었다**(원인: 손목 측면 오프셋 7.3cm 때문에 정렬 위치 반지름이 작을수록 J1이
크게 돌아야 함 — `docs/VISUALIZATION_HANDOFF.md` 1절 참고).

즉 **"0.5는 과했음"은 그 시절 기하에 한정된 결론**이다. 기하가 바뀌면 이 값을 다시
시험해볼 가치가 있다. 상한 0.65(서보 speed 상한)는 기하와 무관하므로 그대로다.

`ACCELERATION_SCALING`이 0.1로 다른 구간(0.2)의 절반이었던 것도 함께 올렸다 —
정렬·복귀만 램프가 두 배로 길었고, 떨림이 그 두 구간에 몰린 것과 방향이 맞았다.

### (4) `tolerance_orientation` 0.5 → 0.2 (파지 정확도 쪽 개선)

정렬이 orientation을 최대 28.6° 어긋난 채 끝낼 수 있었고, **그 오차를 다음 단계인
직진 접근(Cartesian, 정확한 자세 요구)이 대신 갚고 있었다.** 결과:

- 4cm 이동에 관절 총 154°(J4 혼자 67.7°)
- `Computed Cartesian path ... followed 44.8% / 75%` — 직선을 절반도 못 따라감
- 사용자 관측 "3/5부터 자세가 많이 바뀐다"

0.2로 조인 뒤 **J4 67.7° → 5.1°, J6 31.1° → 7.9°**, 총 162.8° → 103.9°.

`0.1`은 무리였다 — 서로 다른 두 좌표에서 정렬이 모두 `STATUS_ABORTED`. 이 로봇/솔버는
정확한 orientation 목표를 잘 못 푼다(roll 후보 탐색 개발 때
`compute_ik` + 정확한 orientation + `avoid_collisions` 성공률이 사실상 0이었던 것과
같은 원인). 정렬 실패가 잦아지면 0.3으로 완화 가능 — 그만큼 접근이 갚을 오차는 늘어난다.

남은 103.9°는 성격이 다르다(J2 32.5°, J3 25.6°, J1 18.4° — 어깨/팔꿈치/베이스).
이 위치에서 4cm 전진에 팔 전체가 크게 움직여야 하는 것으로, tolerance로는 더 줄지 않는다.

---

## 7. 변수 목록

### coord_to_goal_node.py

| 변수 | 값 | 의미 | 적용 구간 |
|---|---|---|---|
| `VELOCITY_SCALING` | **0.5** | joint_limits 최대속도(1.57rad/s)에 곱하는 비율 | 1/5 정렬, look pose 복귀 |
| `ACCELERATION_SCALING` | 0.25 | 가감속 완만함 | 동일 |
| `APPROACH_VELOCITY/ACCELERATION_SCALING` | 0.2 | 3/5 직진 접근 | 3/5 |
| `RETREAT_VELOCITY/ACCELERATION_SCALING` | 0.2 | 열매 쥔 구간 | 5/5 + 복귀까지 |
| `tolerance_orientation` (`_move_arm` 내부) | 0.2 | 정렬 방향 허용오차(rad) | 1/5, 복귀 |
| `CARTESIAN_MAX_STEP_M` | 0.0025 | 속도 아님. Cartesian 공간 해상도 | 3/5, 5/5 |

`RETREAT_*`는 `_check_return_complete()`에서 `VELOCITY_SCALING`으로 원복된다 →
**후퇴~복귀 전 구간이 0.2**다. 복귀는 별도 `grasp_step` 번호가 없어 LED가 후퇴와
같은 초록이다(9절 참고).

### sync_plan.py

| 변수 | 값 | 의미 |
|---|---|---|
| `SPEED_GAIN_K` | 1.7 | `speed = K × 실제 각속도(°/s)`. 하드웨어 환산계수는 2.22 |
| `SPEED_HYSTERESIS` | 4 | 계산값이 이 미만으로 바뀌면 직전 speed 유지 (노이즈 억제) |
| `SPEED_MIN` / `SPEED_MAX` / `SPEED_FALLBACK` | 5 / 100 / 20 | speed=0은 펌웨어 미정의 |
| `USE_DYNAMIC_SPEED` / `FIXED_SPEED` | True / 40 | False면 고정 speed (A/B용) |
| `THROTTLE_PERIOD_SEC` | 0.05 | 전송 게이트 (20Hz) |
| `ANGLE_EPSILON_DEG` | 0.1 | 이하 변화는 전송 생략 |
| `BLINK_TOGGLE_PERIOD_SEC` / `SOLID_STEPS` | 0.25 / {4} | LED 2Hz 점멸, 파지만 상시 점등 |
| `GRIPPER_SPEED` | 50 | 그리퍼 개폐만. 팔과 무관 |

**동적 speed의 시간 기준은 `THROTTLE_PERIOD_SEC` 상수가 아니라 실제 경과 시간
(`_last_sent_time`)이어야 한다.** `ANGLE_EPSILON_DEG`로 생략된 프레임이 있으면 그만큼
시간이 더 흘렀으므로, 상수로 나누면 저속 구간에서 **3배 과대평가**된다(실측 확인:
저속 구간 dt가 0.169s였는데 상수 0.05로 나누면 speed 34, 실제로는 10이 맞는 값).
`_last_send_time`(게이트용, 전송 생략 시에도 갱신)과 혼동하지 말 것.

### joint_limits.yaml / ompl_planning.yaml

| 변수 | 값 | 비고 |
|---|---|---|
| `max_velocity` | 1.57 rad/s (≈90°/s) | 6축 공통 |
| `max_acceleration` | 2.0 rad/s² | 6축 공통 |
| `max_jerk` | **5.0 rad/s³** | 2026-07-30 추가. Ruckig이 이 값을 씀 |
| `response_adapters` | TOTG → **Ruckig** → Validate → Display | 순서 중요 |

URDF는 모든 관절이 `velocity="0"`이라 쓸 수 없다 — `joint_limits.yaml`이 유일한
속도/가속/jerk 출처다.

---

## 8. 조정 치트시트

| 목적 | 조작 |
|---|---|
| 팔을 더 빠르게 | `VELOCITY_SCALING` ↑ (동적 speed가 자동 추종). 단 **0.65가 구조 상한** |
| 미세 떨림 줄이기 | `max_jerk` ↓ (5.0 → 2.0). 궤적이 굼떠지면 ↑ |
| 접촉 구간만 느리게 | `APPROACH_*` ↓ — 단 0.05/0.1에서는 떨림이 관찰됐음(6-(3) U자 곡선) |
| 정렬 실패가 잦음 | `tolerance_orientation` ↑ (0.2 → 0.3). 접근이 갚을 오차는 늘어남 |
| 동적/고정 speed 비교 | `USE_DYNAMIC_SPEED` 토글 (다른 변수 불변) |

**`VELOCITY_SCALING` 상한 0.65**: 동적 speed가 `1.7 × ω`이므로 ω 58.8°/s
(=scaling 0.65)에서 `SPEED_MAX`(100)에 걸린다. 그 이상은 팔만 빨라지고 서보는 100에
고정돼 뒤처진다. 더 올리려면 `SPEED_GAIN_K`를 낮춰 천장을 미뤄야 한다.

**이전 판의 "절대 원칙"은 폐기됐다** — 동적 speed 도입으로 `VELOCITY_SCALING`과 servo
speed를 짝으로 조정할 필요가 없어졌다. 스케일만 바꾸면 speed는 자동으로 따라온다.

---

## 9. 실물 관찰 도구

LED가 단계를 표시한다. **파지(4)만 상시 점등, 나머지는 2Hz 점멸**(구간 구분용).

| 단계 | 색 | 표시 |
|---|---|---|
| 1/5 정렬 | 파랑 | 점멸 |
| 2/5 그리퍼 열기 | 시안 | 점멸 |
| 3/5 직진 접근 | 노랑 | 점멸 |
| 4/5 파지 | 빨강 | **상시** |
| 5/5 후퇴 + look pose 복귀 | 초록 | 점멸 |

후퇴와 복귀는 같은 초록이다(복귀에 별도 번호가 없음). **빨강이 꺼지고 초록이 시작되는
시점이 후퇴 시작**이고, 그 뒤 긴 이동이 복귀다. 구분이 필요하면 양쪽 파일의
`GRASP_STEP_*` / `GRASP_STEP_COLORS`를 함께 고쳐야 한다(두 프로세스가 값을 공유하지 않음).

20Hz 점멸은 의미 없다 — 사람 눈은 10Hz를 넘으면 연속광으로 인지한다.

### 로그 확보

```bash
# RPi: relay 로그. PYTHONUNBUFFERED/stdbuf 없으면 파이썬이 버퍼링해 빈 파일로 보임
PYTHONUNBUFFERED=1 stdbuf -oL ros2 run mycobot_280_moveit2_control sync_plan 2>&1 \
  | tee ~/smh_ws/sync_plan.log
```

`sync_plan` 로그 한 줄에 `speed`, `dmax`(명령당 이동량), `dt`(실제 경과), `°/s`(각속도)가
같이 찍힌다 — `SPEED_GAIN_K` 재캘리브레이션과 구간 식별에 쓴다. `move_group`의
플래닝 로그(어댑터 실행 순서, Cartesian 추종률, jerk 경고)는 **launch.log에 안 담기고
터미널로만 나간다**.

---

## 10. 검증 상태

| 항목 | 상태 |
|---|---|
| `send_angles` 1.5초 블로킹이 끊김의 근본 원인 | **실물+소스 확인.** dt 1.563s = 3×0.5s |
| `_async=True`가 relay를 0.64Hz→16.9Hz로 정상화 | **실물 확인** |
| `set_color`도 같은 블로킹 경로 | **소스 확인.** 단계 전환마다 1.5초 정지였음 |
| `max_jerk` 5.0이 미세 떨림을 줄임 | **실물 확인** (로그 지표로는 재현 못 함 — 6-(2)) |
| `max_jerk` 20.0은 효과 없음(제한선이 느슨) | **실물+로그 확인** |
| `VELOCITY_SCALING` U자 곡선 | **실물 확인** — 리팩터 전 0.35 최적(0.2/0.35/0.5 비교), 리팩터 후 **0.5** 최적. 최적점이 이동함 |
| 0.5의 악화 원인이 기계 진동 | **강한 정황.** 명령 불변인데 흔들림. 진동 자체는 미계측 |
| `tolerance_orientation` 0.2가 접근 구간 재배치를 줄임 | **로그 확인** (J4 67.7→5.1°) |
| 배제된 5개 가설 | **전부 실물 확인** (4절) |
| fresh_mode 0 vs 1 | **미해결.** 통제된 비교 없음 |
| 서보 가속 레지스터(`set_servo_data`) | **미확인.** 레지스터 맵 없음, 아래 참고 |

---

## 11. 남은 과제

### (선택) 서보 레지스터 직접 접근

램프를 근본적으로 없애려면 서보 가속 레지스터를 써야 한다. `set_servo_data(servo_id,
data_id, value, mode=None)`가 있으나 **엘리펀트 문서에 레지스터 맵이 없다.**
mycobot 280은 Feetech STS 계열(12비트 인코더 = 0.088°/스텝으로 추정)이라 가속도
레지스터가 있을 가능성이 높다.

**미검증 / 위험**: 주소를 잘못 쓰면 서보 캘리브레이션이 깨진다. 반드시 (1) 데이터시트로
주소 확인, (2) `get_servo_data`로 원래 값 백업, (3) 1축부터 단독 시험. 확인 전에는
실행하지 말 것.

### 접근 구간의 남은 관절 이동량

`tolerance_orientation`을 조인 뒤에도 4cm 접근에 관절 103.9°가 쓰인다(어깨/팔꿈치 위주).
이 위치의 Jacobian이 그 방향으로 나쁜 것으로, 목표 위치나 접근 방향 자체를 다시
설계해야 줄어든다. 파지 정확도와 직결되므로 다음 우선순위 후보.

### `enable_octomap:=false`가 안 먹는 경우

2026-07-30 관측: `false`로 넘겼는데 `move_group`의 `sensors` 파라미터에
`d435_pointcloud`가 들어가고 octomap updater가 구독을 시작했다. 다만
`points_filtered` 발행자가 0개(필터 노드 미실행)라 실질 영향은 없었다. launch 인자
전달 경로 점검 필요.
