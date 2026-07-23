# Look pose (확정, 2026-07-23)

토마토 베드가 D435(eye-in-hand, joint6 마운트) 시야에 들어오는 관절각.
실물 팔이 릴리즈(무동력) 상태에서 손으로 맞춘 뒤 `get_angles()`로 읽어 확정함.
카메라 프레임으로 실제 토마토(열매/줄기)가 보이는 것 확인 완료.

## 관절각 (degrees)

순서: `joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6`

```
[-6.5, 92.9, -24.96, -65.83, 16.61, 2.02]
```

## 확인 방법

```bash
ssh jetcobot_126b 'python3 -c "
from pymycobot import MyCobot280
mc = MyCobot280(\"/dev/ttyUSB0\", 1000000)
print(mc.get_angles())
"'
```
(토크 없이 읽기만 하므로 릴리즈 상태에서도 안전)

## 참고

- 시뮬레이션(FakeSystem) 관절값과 실물 관절값이 이 순간 기준으로 일치해야
  Octomap의 camera_link TF(joint6 기준 handeye 고정 변환)가 실제 카메라 위치와
  맞음 — 실물이 이후에 움직이면(릴리즈 상태라 손으로 건드리면) 다시 어긋남.
- 그리퍼 바로 앞 노이즈 복셀(근접 거리 depth 노이즈로 추정)이 START_STATE_IN_COLLISION을
  유발하는 별개 이슈 있음 — 근접거리 필터링으로 후속 처리 필요.
