# Look pose (확정, 2026-07-23, 2026-07-24 값 갱신)

토마토 베드가 D435(eye-in-hand, joint6 마운트) 시야에 들어오는 관절각.
실물 팔이 릴리즈(무동력) 상태에서 손으로 맞춘 뒤 `get_angles()`로 읽어 확정함.
카메라 프레임으로 실제 토마토(열매/줄기)가 보이는 것 확인 완료.

**2026-07-24 갱신**: 최초 확정값(`[-6.5, 92.9, -24.96, -65.83, 16.61, 2.02]`)은
그리퍼가 실제 물체(줄기/지지대)에 너무 가까워서(`START_STATE_IN_COLLISION`
유발) `send_coords`로 카메라 시야는 유지한 채 팔만 뒤로/위로 살짝 뺀 값으로
교체함 — 아래 값이 현재 `initial_positions.yaml`/실물이 실제로 쓰는
authoritative 값.

## 관절각 (degrees)

순서: `joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6`

```
[-7.47, 104.06, -56.68, -50.0, 15.99, 1.4]
```

이 자세에서 `g_base` 기준 `joint6_flange` 위치/자세(2026-07-24 확인):
- Translation: `[-0.136, -0.035, 0.241]` (m)
- Rotation (xyzw): `[-0.538, 0.482, -0.442, 0.532]`

## 확인 방법

```bash
ssh jetcobot_126b 'python3 -c "
from pymycobot import MyCobot280
mc = MyCobot280(\"/dev/ttyUSB0\", 1000000)
print(mc.get_angles())
"'
python3 -c "
from pymycobot import MyCobot280
mc = MyCobot280(\"/dev/ttyUSB0\", 1000000)
print(mc.focus_all_servos())
"

python3 -c "
from pymycobot import MyCobot280
mc = MyCobot280(\"/dev/ttyUSB0\", 1000000)
print(mc.focus_all_servos())
"

```
(토크 없이 읽기만 하므로 릴리즈 상태에서도 안전)

## 참고

- 시뮬레이션(FakeSystem) 관절값과 실물 관절값이 이 순간 기준으로 일치해야
  Octomap의 camera_link TF(joint6 기준 handeye 고정 변환)가 실제 카메라 위치와
  맞음 — 실물이 이후에 움직이면(릴리즈 상태라 손으로 건드리면) 다시 어긋남.
- (해결됨, 2026-07-24) `gripper_base - target_object`/`<octomap> - gripper_base`
  충돌이 30cm+ 거리에서도 계속 발생하던 문제의 근본 원인은 그리퍼 콜리전
  메쉬(.dae, mm 단위)가 MoveIt에 m 단위로 잘못 해석되어 실제보다 약 1000배
  크게 취급된 것 — `mycobot_280_m5_adaptive_gripper.urdf`의 그리퍼 링크
  콜리전 mesh 태그에 `scale="0.001 0.001 0.001"` 추가로 수정. 상세는
  `obstacle_avoidance_manual_test.md` "알려진 문제" 참고.
- Self-filter(`sensors_3d.yaml`의 `padding_scale`/`padding_offset` = 0.92/0.0)는
  그리퍼 근처 stray voxel 완화를 위한 실용적 타협이며 완전한 근본 해결은
  아님 — `coord_to_goal_node.py`가 플래닝 직전 `/clear_octomap`을 자동 호출해
  보완 중.
