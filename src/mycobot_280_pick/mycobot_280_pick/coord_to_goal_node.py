#!/usr/bin/env python3
"""
좌표 입력(PointStamped) -> TF2 변환(g_base 기준) -> MoveIt2 목표 플래닝 노드.

로드맵 1단계: tomato_scene_test.py(scripts/)에서 검증한 pymoveit2 좌표->플래닝
로직을 TF2 변환을 포함한 정식 ROS2 노드로 정리한 것. JOINT_NAMES/BASE_LINK_NAME/
END_EFFECTOR_NAME/GROUP_NAME 값은 tomato_scene_test.py에서 이미 확인된 값을
그대로 사용함.

입력: /target_point (geometry_msgs/msg/PointStamped)
  - header.frame_id: 좌표 기준 프레임. 핸드-아이 캘리브레이션 완료 전에는
    카메라 프레임이 TF 트리에 없으므로, frame_id를 BASE_LINK_NAME("g_base")로
    맞춰 발행하면 TF2 변환 없이(identity) 바로 테스트 가능. 캘리브레이션 이후
    카메라 프레임 이름으로 그대로 바꿔 발행하면 코드 수정 없이 실제 연동됨.
  - point: 목표 좌표 (m 단위)

동작: [2026-07-27, 그리퍼 actuation 1단계] 정렬 -> 그리퍼 열기 -> 직진 접근
-> 파지(닫기) -> 후퇴 5단계로 실행함(so101-ros-physical-ai 자매 프로젝트가
"그리퍼 열기 단계 누락"으로 전체 파지가 실패했던 교훈을 처음부터 반영,
docs/obstacle_avoidance_manual_test.md "[Tier5]" 절 참고). 정렬/후퇴는
목표보다 APPROACH_OFFSET_X만큼 로봇 쪽(x축 음의 방향)으로 당긴 위치, 직진
접근은 목표 지점 그 자체(같은 orientation)로 이동함. 어느 한 단계라도
실패하면 그 자리에서 중단하고 look pose 복귀를 시도함(나머지 단계는
건너뜀). 그리퍼 열림/닫힘 방향(GRIPPER_OPEN_POSITION/
GRIPPER_CLOSED_POSITION)은 아직 실물/RViz 육안 확인 전 — 반대로 보이면
그 두 상수만 맞바꿀 것.

[2026-07-27, 손목 회전 최소화 / 2026-07-28, 전체 관절로 확장] 정렬 orientation을
정하는 roll 후보 탐색(ROLL_CANDIDATES_RAD)이 "IK가 풀리는 첫 후보"를 그대로
채택하던 것을, 현재 관절 자세에서 6축 관절 이동량 합이 가장 적은 후보를
고르도록 바꿈 — 처음엔 손목만 비교했는데, 실물 로그에서 joint1(베이스)이
140도 가까이 스윙하는 사례가 확인돼(사용자 피드백, "몸을 틀면서 이동")
전체 관절로 비교 범위를 넓힘. 토마토는 거의 구형이라 roll 값 자체는 파지
성공에 영향이 없으므로, 이 변경은 안전 여유를 줄이지 않으면서 불필요한
움직임만 줄임(아래 _try_next_roll_candidate/_on_roll_candidate_result 참고).

[2026-07-27, 그리퍼 폭 동적화] 그리퍼 열림 폭을 항상 GRIPPER_OPEN_POSITION
(근사 최대 개방)으로 고정하지 않고, YOLO가 추정한 대상 반지름(target_radius_m
토픽, yolo_d435_detector_node가 target_point와 함께 발행)에 맞춰 조정함 —
실제 토마토(사용자 실측 "둘레 35~40mm")보다 훨씬 넓게 열던 것을 줄여 옆
줄기/다른 토마토가 같이 잡힐 여지를 낮추려는 의도(GRIPPER_TARGET_RADIUS_*/
_gripper_open_position_for_radius 참고). URDF 순기구학 근사치라 실물 육안
확인 전 — 다음 세션에 재확인 필요.

Octomap 클리어 로직: 목표(토마토) 자체가 실제 물체라서 D435 point cloud를
통해 Octomap에 장애물로 찍혀 있음 — 그대로 두면 목표 근처로 접근하는 자세가
항상 "충돌"로 판정되어 플래닝이 매번 실패함 (실제 end-to-end 테스트에서
"Unable to sample any valid states for goal tree"로 확인됨). 그래서 목표
좌표에 작은 구(SPHERE) CollisionObject를 등록해서, MoveIt의
PlanningSceneMonitor가 이 구와 겹치는 Octomap voxel을 자동으로 걸러내도록 함
(로봇 자기 몸 필터와 같은 메커니즘 — world 안의 collision object도 제외
대상임). 실제 Octomap이 새 point cloud로 이 필터를 반영하기까지 한 주기
걸리므로(sensors_3d.yaml의 max_update_rate=1.0Hz), 구를 등록한 뒤
OCTOMAP_CLEAR_DELAY_SEC만큼 기다렸다가 플래닝을 시작함.

주의: `MoveIt2.wait_until_executed()`와 (use_move_group_action=False일 때의)
`plan()`은 내부적으로 `rclpy.spin_once(self._node, ...)`를 호출하는데, 이 노드는
이미 자체 MultiThreadedExecutor로 이 노드를 spin 중이라 두 스핀이 충돌해서 첫
성공 실행 이후 후속 콜백이 더 이상 디스패치되지 않는 문제가 있었음. 그래서
(1) MoveIt2를 `use_move_group_action=True`로 생성해 완전히 콜백 기반인
MoveGroup 액션 경로를 쓰고(=spin_once 호출 코드 경로 자체를 안 탐),
(2) 완료 대기는 blocking 호출 대신 타이머로 `query_state()`를 폴링하는 방식으로
처리함 (실행 중인 executor 하나만 스핀 담당).
"""

import functools
import math

from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    AttachedCollisionObject,
    CollisionObject,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
    PositionIKRequest,
)
from moveit_msgs.srv import GetPlanningScene, GetPositionIK
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.moveit2_gripper import MoveIt2Gripper
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float32, Int32
from std_srvs.srv import Empty, Trigger
from tf2_geometry_msgs import do_transform_point
import tf2_ros

# 실행 완료 여부를 폴링하는 주기 (초)
COMPLETION_POLL_PERIOD = 0.2

# [2026-07-24] 목표 지점 접근 완료 후 이 시간만큼 머문 뒤(향후 그리퍼 동작이
# 들어갈 자리) look pose로 자동 복귀. 실물 테스트에서 팔이 목표 지점에 계속
# 머물러 있으면 카메라(eye-in-hand)가 다음 토마토를 넓게 못 보고, 방금 접근한
# 토마토조차 화면 가장자리에 bbox가 걸려 depth가 무효해지는 문제를 확인함 —
# harvest_sequence_node처럼 다음 목표를 순차로 보내는 상위 로직이 항상 look
# pose 프레이밍에서 다음 검출을 받도록 보장하기 위함.
RETURN_TO_LOOK_POSE_DELAY_SEC = 1.5

# SRDF(firefighter.srdf)의 look_pose group_state와 동일 (docs/look_pose.md).
LOOK_POSE_JOINT_POSITIONS = [
    -0.130376,
    1.816190,
    -0.989253,
    -0.872665,
    0.279078,
    0.024435,
]

# ---- 로봇 설정 (tomato_scene_test.py에서 확인된 값과 동일) ----
JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]
BASE_LINK_NAME = 'g_base'
END_EFFECTOR_NAME = 'joint6_flange'
GROUP_NAME = 'arm_group'

# 그리퍼 MoveIt 플래닝 그룹 이름(SRDF firefighter.srdf의 "gripper" 그룹)과
# 그 그룹의 실제(mimic 아닌) 구동 관절 이름. 나머지 5개 손가락 관절은
# URDF에서 이 관절을 mimic하므로 코드에서 따로 다룰 필요 없음.
GRIPPER_GROUP_NAME = 'gripper'
GRIPPER_JOINT_NAME = 'gripper_controller'
# gripper_controller 관절(rad)의 완전 개방/완전 폐쇄 목표값. URDF 물리
# 한계는 -0.74~0.15rad — 정확히 한계값을 목표로 주면 OMPL이 목표 상태를
# 못 찾아 플래닝이 실패하므로 양 끝에서 여유를 둠. 실물 파지 성공 사례로
# 열림/닫힘 방향 확인됨(반대면 이 두 값만 맞바꾸면 됨).
GRIPPER_OPEN_POSITION = 0.12
GRIPPER_CLOSED_POSITION = -0.6

# 대상 반지름(target_radius_m, YOLO 추정치)을 그리퍼 개방 관절값으로 선형
# 보간할 때 쓰는 입력 구간(m) — 실측 토마토 두 종류(지름 25mm/36mm)의
# 반지름과 동일해야 함. yolo_d435_detector_node.py의 MIN/MAX_ESTIMATED_
# RADIUS_M과 반드시 같은 값으로 유지(두 노드가 독립 실행 파일이라 값을
# import로 공유하지 않음).
GRIPPER_TARGET_RADIUS_MIN_M = 0.0125  # 25mm 지름 토마토의 반지름
GRIPPER_TARGET_RADIUS_MAX_M = 0.018   # 36mm 지름 토마토의 반지름

# 위 반지름 구간에 대응하는 그리퍼 개방 관절값 구간(rad) — 큰 쪽
# (GRIPPER_OPEN_POSITION)은 실측 개방폭 약 50mm(사용자 실측 "5cm 정도
# 열었을 때"), 작은 쪽(GRIPPER_OPEN_POSITION_SMALL)은 그 실측점을 기준으로
# URDF 순기구학 근사 곡선을 스케일링해 25mm 대상용 개방폭(~35mm)에
# 대응하도록 역산한 값 — 완전한 실측은 아니므로 실물 확인 필요.
GRIPPER_OPEN_POSITION_SMALL = -0.49

# [방식 B, 2026-07-24] 목표 위치마다 "그리퍼가 목표 방향을 바라보는"
# orientation을 매번 동적으로 계산함(look-at 방식). 고정 orientation(identity,
# look-pose 값, `GRIPPER_ORIENTATION` 등)은 모두 위치별로 성공/실패가 갈려서
# 작업공간 전체를 커버하지 못함이 확인됐음(6축 팔의 IK 특성상 자연스러운
# 결과, docs/obstacle_avoidance_manual_test.md "알려진 문제" 참고).
#
# 그리퍼 "정면"(그리퍼가 물체를 향해 뻗어나가는 방향) 축 확정: `joint6_flange`의
# 로컬 **+Z축**. 실물 팔을 움직여 실측하는 대신, URDF 링크체인(joint6_flange ->
# gripper_base(고정 조인트, origin z=0.034) -> 손가락 링크들)으로 FK를 계산해
# 손가락 중점이 flange 원점 기준 거의 순수한 로컬 +Z 방향(정규화 벡터
# [0, -0.003, 0.99999], 거리 ~5.5cm)에 있음을 확인함(2026-07-24, 스크립트로
# 검증). look pose 실측 쿼터니언(위 docstring, xyzw=[-0.538,0.482,-0.442,0.532])
# 으로 교차검증해도 이 +Z축이 g_base 기준 [0.988,0.146,-0.043] 방향(대략
# 로봇 앞쪽)을 가리켜 물리적으로도 합당함 — 기존
# `GRIPPER_BOX_Z_OFFSET`(아래) 계산이 가정했던 축과 일치.
#
# `_compute_look_at_quat_xyzw()`가 이 forward(+Z)축을 목표 방향으로 정렬하는
# 회전을 계산함. TF 조회 실패 등 예외 상황에서만 아래 고정값(look pose에서
# 실측한 orientation)으로 폴백함.
FALLBACK_APPROACH_QUAT_XYZW = [-0.538, 0.482, -0.442, 0.532]

# look-at 계산에서 roll(정면 축 둘레 회전)을 고정하기 위한 기준 "up" 벡터
# (g_base 기준 world +Z). 향후 AI Service가 토마토 기울기 정보를 주면 이
# 값을 그 기울기 벡터로 교체할 예정(docs/obstacle_avoidance_manual_test.md
# "향후 계획" 참고).
WORLD_UP = (0.0, 0.0, 1.0)

# 정렬(1/5)/후퇴(5/5) 위치와 직진 접근(3/5) 최종 목표(grasp_target_x,
# 아래 GRIPPER_LENGTH_OFFSET_M 반영 후 값) 사이의 g_base X축 방향 거리(m).
# g_base 원점 기준이 아니라 이 두 웨이포인트 간의 상대 간격임. 방위각이
# ±75도 이내로 제한돼 있어 forward 방향이 대체로 g_base -X에 가까우므로,
# 실제 3차원 forward축 대신 X축 단일 성분으로 근사함(look-at 방향 그대로
# 쓰면 더 정확하지만 계산이 복잡해짐).
# [2026-07-28] 0.08 -> 0.04로 축소 — GRIPPER_LENGTH_OFFSET_M과 합쳐 정렬
# 위치를 당기는 총량이 MIN_TARGET_RADIUS_M 체크 기준(정렬 위치 반지름
# ≥0.08m)을 결정하는데, 실측 토마토 베드 거리(g_base 원점 기준 약 0.25m)
# 근처의 정상적인 목표까지 이 체크에 걸려 거부되는 걸 확인함(0.08일 땐
# 반지름 0.255m 이상만 통과, 0.04로 줄이면 0.215m 이상까지 커버). 순수
# 여유 마진이라(GRIPPER_LENGTH_OFFSET_M과 달리 실측 그리퍼 길이에 묶여
# 있지 않음) 줄여도 물리적 근거가 깨지지 않음.
APPROACH_OFFSET_X = 0.05 # 0.08

# 직진 접근(3/5)이 이동하는 flange 목표 = 검출된 토마토 위치(target.x)에서
# 이 값을 g_base X축으로 뺀 지점. 그리퍼는 flange 원점에 있지 않고 그보다
# 앞으로 뻗어나간 손가락 위치에서 물체를 물기 때문에, flange를 토마토
# 위치 자체로 보내면 손가락(파지점)은 이미 그만큼 더 깊이 들어간 상태가
# 됨 — 이 값은 그 차이를 미리 빼서 파지점이 실제 토마토 위치에 오도록
# 맞추는 보정. 접근은 그리퍼가 열린 상태로 진행되므로(2/5 이후), 닫았을
# 때가 아니라 **연 상태의 flange-파지점 거리**를 써야 함(사용자 실측:
# 닫았을 때 110mm, 5cm 정도 열었을 때 90mm — 열수록 파지점이 flange
# 쪽으로 가까워짐). Octomap 클리어용 CollisionObject(target_object)는
# 이 보정과 무관하게 실제 토마토 위치를 그대로 씀(_on_target_point 참고).
# 그리퍼가 목표를 지나쳐서(overshoot, 너무 깊이 들어감) → 이 값을 키움(flange를 더 뒤로 당김)
# 그리퍼가 목표에 못 미침(undershoot, 너무 짧게 감) → 이 값을 줄임(flange를 덜 당김, 목표에 더 가깝게)
# [2026-07-28] 실측 그리퍼 길이(90mm)보다 크게 키우지 말 것 — 이 값 +
# APPROACH_OFFSET_X가 정렬 위치를 베이스 쪽으로 당기는 총량이라, 너무 크면
# 가까운 목표(반지름 20cm대)의 정렬 위치가 베이스 근처 도달 불가 사각지대
# (반지름 <8cm)에 빠짐 — 0.15로 테스트 중 재현 확인(_on_target_point의
# 정렬 위치 반지름 체크가 이제 이 경우를 미리 거름). overshoot이 남아있다면
# 이 값이 아니라 depth 추정 자체(DEPTH_SAFETY_MARGIN_M,
# yolo_d435_detector_node.py)를 의심할 것.
# [2026-07-28, 실물 검증] 0.095로 실물 테스트 — g_base (0.32, 0.039, 0.307)
# 목표에 flange가 실제 토마토 위치에 정확히 도착함(육안 확인, sync_plan
# 경유 실물 이동). 90mm 실측치에서 5mm만 더 보정한 값.
GRIPPER_LENGTH_OFFSET_M = 0.09 # 0.09 # 실측

# Octomap 자기 필터용으로 목표 지점(실제 토마토 위치)에 등록하는
# CollisionObject(구)의 반지름(m) — GRIPPER_LENGTH_OFFSET_M과는 무관한
# 별개 상수(우연히 비슷한 값이었던 적이 있었을 뿐). 실측 토마토 반지름
# (최대 18mm, 위 GRIPPER_TARGET_RADIUS_MAX_M 참고)보다 넉넉히 큰 이유는
# depth 카메라가 재는 지점이 물체의 "카메라 쪽 표면"이라 노이즈 섞인
# point cloud가 그보다 튀어나오는 경우를 감안한 여유임.
TARGET_OBJECT_RADIUS = 0.05
TARGET_OBJECT_ID = 'target_object'

# 그리퍼 URDF 링크 7개(mycobot_280_m5_adaptive_gripper.urdf) 이름 목록 —
# target_object(위 TARGET_OBJECT_RADIUS 구)와의 충돌을 ACM에서 허용하는 데
# 씀(그리퍼가 접근하며 이 구 안으로 들어가는 건 정상 동작이라 충돌 판정에서
# 제외해야 함, Octomap/다른 world 장애물과의 충돌 검사는 그대로 유지).
GRIPPER_LINK_NAMES = [
    'gripper_base',
    'gripper_left1',
    'gripper_left2',
    'gripper_left3',
    'gripper_right1',
    'gripper_right2',
    'gripper_right3',
]

# joint6_flange 바로 위 두 링크(joint5, joint6) 이름 — target_object 구
# 안으로 그리퍼가 깊이 들어갈 때 이 두 링크까지 구와 겹치므로,
# GRIPPER_LINK_NAMES와 같은 이유로 ACM 충돌 허용에 같이 포함해야 함(빠지면
# 목표 지점 근방에서 항상 self-collision 판정되어 플래닝이 실패함).
WRIST_LINK_NAMES = ['joint5', 'joint6']

# 구를 planning scene에 등록한 뒤, Octomap이 다음 point cloud로 자기 필터를
# 반영할 때까지 기다리는 시간 (sensors_3d.yaml의 max_update_rate=1.0Hz보다
# 살짝 길게 잡음).
OCTOMAP_CLEAR_DELAY_SEC = 1.2

# [2026-07-27] "직진 접근"/"후퇴"(3/5, 5/5단계)는 이미 유효하다고 검증된
# 정렬 위치에서 같은 orientation을 유지한 채 직선으로 짧게(APPROACH_OFFSET_X)
# 이동하는 것뿐인데, OMPL 조인트공간 샘플링(RRTConnect)은 매번 완전히 새로운
# 목표를 무작위로 찾으려 해서 "Unable to sample any valid states for goal
# tree"로 반복 실패함(mock 스택으로 여러 좌표에서 재현 확인, compute_ik
# 단발 호출은 즉시 성공하는데 OMPL은 3초/10회 시도에도 전부 실패). 이런
# "이미 유효한 상태에서 짧은 직선 이동"은 Cartesian 경로 계획
# (`compute_cartesian_path`, 직전 관절해를 시드로 한 스텝씩 IK를 풀어나감)이
# 훨씬 안정적이라 이 두 단계만 Cartesian으로 전환함(정렬/그리퍼 열기/파지는
# 기존 OMPL 방식 유지 — 이미 안정적으로 동작 확인됨).
CARTESIAN_MAX_STEP_M = 0.0025
# Cartesian 경로가 전체 거리의 이 비율 이상 풀렸을 때만 성공으로 인정
# (완전히 100%를 요구하면 사소한 수치 오차로도 거부될 수 있어 약간 여유를 둠).
CARTESIAN_FRACTION_THRESHOLD = 0.95

# [2026-07-27] 정렬(1/5) 완료 직후 직진 접근 Cartesian 경로가 dry-run으로
# 안 이어지면(위 "직진 접근 reachability" 설명 참고) 정렬을 다시 풀어서
# 재시도하는 최대 횟수. OMPL은 확률적이라 재시도마다 다른 관절해를 고르므로
# 몇 차례 재시도하면 이어지는 해가 나올 가능성이 있음 — 다만 위치 자체가
# 구조적으로 어려운 경우(관절 한계 근접 등)엔 재시도로도 안 풀릴 수 있어
# 상한을 둠.
MAX_ALIGN_RETRIES = 5

# pymoveit2 기본값(allowed_planning_time=0.5s, num_planning_attempts=5)은 OMPL
# 목표 샘플링이 확률적이라(같은 위치/orientation이어도 시도마다 성공/실패가
# 갈림, 2026-07-24 실물 스택 테스트로 확인) IK 여유가 좁은 목표에서 자주
# "Unable to sample any valid states for goal tree"로 실패함. 여유를 늘려
# 재시도 확률을 높임.
PLANNING_TIME_SEC = 3.0
PLANNING_ATTEMPTS = 10

# [Tier1, so101-ros-physical-ai 교훈 이식] pymoveit2 MoveIt2의
# max_velocity/max_acceleration은 기본값 0.0인데, 이는 MoveIt에 "스케일링
# 없음"으로 전달되어 사실상 최대 속도로 실행됨 — 지금까지 이 노드는 이
# 값을 한 번도 설정한 적이 없었음. 자매 프로젝트(so101)가 실물에서 "파지
# 후 후퇴가 위험할 정도로 빠르다"를 반복 피드백받고서야 뒤늦게 낮춘 전례가
# 있어(최종 15%), 같은 사고를 mycobot에서 먼저 겪지 않도록 선제적으로
# 보수적인 값을 명시함. 체감 속도 확인 후 조정 가능.
# [2026-07-30] 0.2/0.1 -> 0.35/0.25. 정렬(1/5)과 look pose 복귀 구간이 "느려서
# 건들건들"하다는 실물 관측에 대응. 이 두 구간은 이동 거리가 400°대로 길어서
# 같은 관절속도로도 수 초가 걸리고, 그 사이 미세 떨림이 계속 보임(반면 접근·
# 후퇴는 4cm/약 1초라 짧아서 지각되지 않음 — 같은 scaling 0.2인데 후퇴는
# 부드럽고 복귀는 떨린다는 관측이 이를 뒷받침).
#
# 남은 유일한 미검증 가설인 "램프 희석"을 시험하는 것: 명령당 이동량이 커지면
# (0.9° -> 1.8°) 서보 내부 가감속 램프가 전체 이동에서 차지하는 비율이 줄어
# 개별 움직임이 매끄러워짐. relay 쪽 레버(명령 주기, speed 수준, speed 노이즈,
# 명령 단조성, 도착 타이밍 t/T)는 모두 실측으로 배제됨 — sync_plan.py의
# SPEED_GAIN_K / SPEED_HYSTERESIS 주석 참고.
#
# ACCELERATION_SCALING은 0.1로 다른 모든 구간(APPROACH/RETREAT = 0.2)의 절반
# 이었음 — 정렬·복귀만 램프가 두 배로 길었다는 뜻이라 같이 올림.
#
# 동적 speed(sync_plan)가 켜져 있으면 ω가 오르면 서보 speed도 자동으로 따라
# 오르므로(0.35 -> ω 31.5°/s -> speed 54) 짝을 따로 맞출 필요 없음.
# 접근(3/5)·후퇴(5/5)는 이미 "빨라서 위험"하다는 관측이 있어 0.2 유지.
# [2026-07-30, 2차] 0.35 -> 0.5. 0.35에서 "많이 줄었지만 여전히 건들"이라는
# 실물 관측 — 램프 희석이 실제로 듣는 메커니즘임이 확인됐으므로(배제된 다른
# 가설들과 달리 유일하게 개선을 낸 변경) 같은 방향으로 더 밀어봄.
# 0.35 실측: ω 31.4°/s, 명령당 1.887°, 동적 speed 53, 상한 클램프 0건.
#
# ⚠️ 이 구조의 상한은 scaling 0.65임 — 동적 speed가 SPEED_GAIN_K(1.7) × ω이므로
# ω 58.8°/s(=scaling 0.65)에서 speed가 SPEED_MAX(100)에 걸린다. 그 이상 올리면
# 서보 speed는 100에 고정된 채 팔 목표만 빨라져 서보가 뒤처지고(짝이 깨짐)
# 오히려 나빠진다. 더 올리려면 SPEED_GAIN_K를 낮춰 천장을 미루거나, 서보
# 자체의 speed 상한을 넘는 문제로 받아들여야 함.
# [2026-07-30, 3차 — 0.5는 과했음. 0.35로 되돌림]
# 0.5(ω 45°/s)에서는 "전 구간에서 흔들린다"가 됨. 결정적 단서: 그때 접근·후퇴
# (여전히 0.2)의 명령은 이전과 완전히 동일했음(ω 17.9, dmax 1.077, speed 29,
# dt 표준편차 0.0046s로 안정) — 즉 그 구간 명령이 나빠진 게 아니라, 정렬·복귀의
# 빠른 대각도 스윙(400°+)이 팔 구조를 기계적으로 울리게 만들고 그 진동이
# 이어지는 구간까지 남은 것으로 판단됨.
# 결론: 떨림은 scaling에 대해 U자 곡선 — 너무 느리면 도착-정지가 보이고,
# 너무 빠르면 구조 진동이 생김. 실측상 0.35가 최선(0.2보다 "많이 줄었다").
VELOCITY_SCALING = 0.35 #0.5 #0.2 #0.1
ACCELERATION_SCALING = 0.25 #0.35 #0.1

# 후퇴(5/5) + look pose 복귀 구간(그리퍼가 물체를 쥔 채 움직이는 유일한
# 구간)의 속도/가속 스케일 — 일반 VELOCITY_SCALING(0.1)보다 낮춰서 파지한
# 물체가 떨어지거나 흔들리지 않게 함.
RETREAT_VELOCITY_SCALING = 0.2 #0.05
RETREAT_ACCELERATION_SCALING = 0.2 #0.05

# [2026-07-28] 직진 접근(3/5)의 속도/가속 스케일 — 목표(토마토)에 실제로
# 닿는 구간이라 후퇴와 같은 이유로 낮춤(접촉 직전 감속, 부드러운 접근).
# 정렬(1/5)까지는 빈 그리퍼로 멀리서 움직이므로 기존 VELOCITY_SCALING
# 유지, 이 구간만 낮춤.
# [2026-07-30, 2차] 0.05 -> 0.1. 0.05에서 실물이 흔들리는(떨리는) 현상이 남았고,
# 원인이 "명령 주기"가 아님을 실측으로 배제했음: sync_plan의 ANGLE_EPSILON_DEG를
# 0.7 -> 0.1로 낮춰 접근 구간 전송률을 5.9Hz -> 17.2Hz(순항과 동일)까지 올렸는데도
# 떨림이 남았다. 남은 원인은 서보 자체의 저속 특성으로 판단 —
#   (a) 저속 코깅/stick-slip: speed=10은 0~100 스케일에서 상당히 낮음
#   (b) 위치 양자화: Feetech 계열 12비트 인코더(4096스텝/360° = 0.088°/스텝)
#       기준으로 0.05 스케일의 명령당 이동량 0.269°는 3스텝 정도뿐이라 목표
#       주변에서 헌팅하기 쉬움
# 둘 다 소프트웨어로 없앨 수 없으므로 실제 이동 속도를 올려 회피한다
# (0.1 -> ω 9°/s, 명령당 0.45°≈5스텝, sync_plan의 동적 speed는 자동으로 20).
#
# ⚠️ 올릴 값은 이 스케일이지 sync_plan의 SPEED_GAIN_K가 아니다. 후자를 올리면
# 팔의 속도는 그대로인 채 서보만 각 목표로 전력질주한 뒤 대기하게 되어
# 덜컹거림이 오히려 악화된다.
#
# 접촉 구간 저속이라는 원래 설계 의도와의 타협점 — 순항(0.2)의 절반은 유지함.
# 0.1에서도 떨리면 0.15까지 올려볼 수 있으나, 토마토에 실제로 닿는 구간이므로
# 파지 정확도/안전과 맞바꾸는 것임을 인지할 것.
#
# velocity와 acceleration은 반드시 두 줄을 같이 확인할 것 — 2026-07-30에
# acceleration만 0.2로 되돌아가 두 값이 엇갈린 채(velocity 0.05 + acceleration
# 0.2) 실물 테스트가 한 번 돌아갔음(편집기 vim 모드에서 우발적 undo로 추정).
# 이 구간 값을 바꾼 뒤에는 실행 전에 디스크 내용을 직접 확인하는 게 안전함:
#   grep -n "^APPROACH_.*SCALING" coord_to_goal_node.py
# [2026-07-30, 3차 — 0.2로 되돌림] 위 (a)/(b) 가설로 0.05->0.1까지 올려봤으나
# 떨림이 남았고, 시간순 증거가 "저속 자체가 원인"을 가리킴:
#   - _async 수정 직후 이 값이 0.2였을 때 사용자 평가가 "차원이 다르게
#     부드러워졌다"였음
#   - 떨림은 그 뒤 이 값을 0.05로 낮춘 다음부터 나타났고, 0.1로 올려도 잔존
#   - 사용자 관측 "3/5는 천천히 가니까 흔들흔들하면서 간다" = 저속 구간 특정
#   - _async 이후에 0.2를 재시험한 적이 없어 이 가능성을 오래 놓치고 있었음
# 부수 효과로 RETREAT(0.2)와 값이 같아져, 파지 후 후퇴 시작 시 speed가 20->40
# 으로 2배 점프하던 불연속(사용자 관측 "물러설 때 속도가 갑자기 빨라져")도
# 사라짐 — 이 점프는 APPROACH만 낮춘 데서 생긴 것이었음.
#
# 접촉 구간을 의도적으로 저속으로 두려는 원래 설계 의도는 포기함. 근거: 접촉
# 직전에 그리퍼가 떨리는 것이 조금 빠르게 매끄럽게 접근하는 것보다 파지
# 정확도에 더 해로움. 저속이 필요하다면 speed/주기 튜닝이 아니라 서보 자체의
# 저속 특성을 개선할 방법을 찾아야 함(현재 수단 없음).
APPROACH_VELOCITY_SCALING = 0.2 #0.1 #0.05
APPROACH_ACCELERATION_SCALING = 0.2 #0.1 #0.05

# 파지(4/5) 완료 후 후퇴 시작 전 이 자리에 머무는 시간(초) — 실제로 물체를
# 잡았는지 육안으로 확인할 시간을 줌.
POST_GRASP_DWELL_SEC = 2.0

# [2026-07-28, 그리퍼 단계 LED 표시] /grasp_step 토픽에 발행하는 단계 번호.
# RPi sync_plan.py의 GRASP_STEP_COLORS와 반드시 같은 매핑을 유지할 것(두
# 프로세스가 독립 실행 파일이라 값을 import로 공유하지 않음).
GRASP_STEP_IDLE = 0
GRASP_STEP_ALIGN = 1
GRASP_STEP_GRIPPER_OPEN = 2
GRASP_STEP_APPROACH = 3
GRASP_STEP_GRASP = 4
GRASP_STEP_RETREAT = 5

# TF 변환된 목표(target, g_base 기준)를 플래닝하기 전에 검증하는 안전
# 범위. `_target_radius_azimuth()`가 계산하는 두 값과 비교함:
#   - radius = hypot(target.x, target.y) — g_base 원점(팔의 회전축)에서
#     목표까지의 수평 거리(z는 무시). MIN/MAX_TARGET_RADIUS_M 밖이면 거부
#     (너무 가까우면 그리퍼 코앞이라 접근 자체가 위험, 너무 멀면 작업공간
#     밖이라 애초에 도달 불가).
#   - azimuth_deg = atan2(target.y, target.x) — g_base +X축 기준 목표의
#     수평 회전각(도). |azimuth| > MAX_TARGET_AZIMUTH_DEG면 거부(팔이 봐야
#     할 정상 작업 영역 밖, TF 오류 등으로 좌표가 잘못 계산된 경우 이 값이
#     비정상적으로 커짐).
# 즉 이 셋은 "목표가 g_base 기준으로 말이 되는 위치에 있는지"를 실행 전에
# 거르는 최소 안전망 — TF/검출 오류로 엉뚱한 좌표가 들어와도 팔이 그
# 방향으로 크게 움직이는 사고를 막음. 실측 기준(g_base 원점~토마토 실제
# 거리 약 0.3m)에 앞뒤로 넉넉한 여유를 두고 잡은 값(0.08~0.45m) — SO-ARM101
# 등 기구학이 다른 로봇의 값을 가져온 게 아니라 mycobot 280 자체 실측
# 기준. 이런 종류의 검증 자체를 넣게 된 동기는 so101-ros-physical-ai
# 자매 프로젝트가 겪은 TF 타이밍 버그(변환에 쓰인 로봇 자세가 실제와 안
# 맞아 좌표가 엉뚱하게 계산된 사고)인데, 이건 로봇 구조와 무관하게 TF
# 기반 좌표 파이프라인이면 어디서나 날 수 있는 소프트웨어 버그 패턴이라
# 참고한 것뿐 — 값 자체와는 무관함.
MIN_TARGET_RADIUS_M = 0.08
MAX_TARGET_RADIUS_M = 0.45
MAX_TARGET_AZIMUTH_DEG = 75.0

# [2026-07-24, roll 후보 탐색] look-at 방식(방식 B)이 WORLD_UP 기준으로 고정하는
# roll(정면 축 둘레 회전)은 목표 위치에 따라 IK가 아예 안 풀리는 "데드존"에 걸릴
# 수 있음이 실측으로 확인됨(g_base [0.219,0.054,0.311] 근처에서 재현, 같은
# forward 방향이라도 roll을 15도만 바꾸면 IK가 풀리는 경우가 다수 있었음 —
# compute_ik 서비스로 24단계 스윕해 확인, docs/obstacle_avoidance_manual_test.md
# 참고). 6축 팔이라 이 roll이 자유도가 아니라 위치별 도달가능한 값이 정해져
# 있어서, 후보를 여러 개 순차 시도해 실제 compute_ik로 풀리는 값을 고름.
# 30도 간격 12개 후보(0도=기존 WORLD_UP 단일값과 동일해 하위호환 유지).
ROLL_CANDIDATES_RAD = [i * (2.0 * math.pi / 12.0) for i in range(12)]
# 후보 하나당 IK 서비스 타임아웃. kinematics.yaml의 KDL 솔버 타임아웃(0.005s)
# 기준으로 몇 차례 랜덤 재시작이 가능하도록 여유를 둠(compute_ik 실측 확인).
ROLL_CANDIDATE_IK_TIMEOUT_SEC = 0.05
# KDL(수치해석) IK는 같은 위치/orientation/시드에도 매 호출마다 결과가 바뀌는
# 확률적 솔버임이 실측으로 확인됨(2026-07-24, 같은 조건 10회 호출에 성공 7~9회
# 정도로 편차 있었음). 후보 하나를 한 번만 시도하면 실제로 풀리는 후보를
# 운 나쁘게 놓칠 수 있어, 후보마다 몇 번 재시도한 뒤 다음 후보로 넘어감.
ROLL_CANDIDATE_IK_RETRIES = 3

# [2026-07-29, 손목 특이점 회피] joint6_to_joint5(J5, "wrist pitch")가 0
# 근처면 이 손목 구조(J4·J6 회전축이 J5=0에서 거의 겹치는 spherical wrist)
# 특성상 목표가 아주 조금만 바뀌어도 J4/J6가 수십~백 도 단위로 크게 튀는
# 현상이 실물 로그로 확인됨(2026-07-29, 사용자 피드백 — 직진접근/후퇴/
# look pose 복귀 전부에서 발생). roll 후보(ROLL_CANDIDATES_RAD)는 순수하게
# forward축 둘레 회전이라 J6만 바꾸고 J5엔 거의 영향이 없어 이 문제를 못
# 잡음 — 대신 forward(목표 지향) 벡터의 고도각을 살짝 틀어서 J5 자체를
# 특이점에서 떨어뜨림(_tilt_forward_vector, _on_roll_candidate_result 참고).
WRIST_PITCH_JOINT_NAME = 'joint6_to_joint5'
WRIST_SINGULARITY_MARGIN_RAD = math.radians(8.0)
# 0도(원래 방향)부터 시도하고, 안 되면 점점 더 크게 기울여봄. 토마토는 거의
# 구형이라 접근 고도각이 이 정도 틀어져도 파지 성공에는 영향이 없다고 판단
# (기존 roll 무관 판단과 같은 근거).
FORWARD_TILT_CANDIDATES_RAD = [
    0.0,
    math.radians(6.0), math.radians(-6.0),
    math.radians(12.0), math.radians(-12.0),
]

# ---- 그리퍼 임시 충돌 형상 ----
# [2026-07-29 정정] 이 코드가 처음 작성될 때는 URDF에 그리퍼가 없었지만,
# 지금 로드되는 firefighter.urdf.xacro는 mycobot_280_m5_adaptive_gripper.urdf를
# include해서 실제 그리퍼 mesh+collision(gripper_base/left1~3/right1~3)이
# 있음. 그런데도 이 박스가 남아있는 이유는 목적이 달라서임 — 실제 그리퍼
# (폭 8~10cm 정도)보다 훨씬 크게(15x15x25cm) 잡은 여유 형상으로, Octomap
# self-filter(sensors_3d.yaml의 self_mask 패딩)가 카메라 바로 앞(~13~21cm)의
# 그리퍼를 충분히 못 덮어서 자기 몸을 장애물로 오인하는 문제의 여유 마진용.
# 지금은 enable_octomap:=false가 기본값이라 이 박스가 실제로 뭘 막아주고
# 있는지 최근에 검증된 적은 없음 — Octomap을 다시 켜고 쓸 일이 생기면
# 이 박스가 여전히 필요한지(패딩만으로 충분한지) 재검증할 것.
# joint6_flange의 회전축(=그리퍼가 붙는 정면 방향)이 flange 로컬 Z축이라서
# (joint6output_to_joint6의 axis="0 0 1"), 그리퍼는 flange 원점에서 로컬
# Z 방향으로 뻗어있다고 가정. 원점에 대칭으로 박스를 두면 절반만 덮여서
# (실측 안 하고 대략치) 로컬 Z 방향으로 오프셋을 줘서 원점~10cm 구간을 덮음.
# 실제와 다르면(반대 방향이면) GRIPPER_BOX_Z_OFFSET 부호를 뒤집으면 됨.
GRIPPER_BOX_DIMENSIONS = [0.15, 0.15, 0.25]  # x, y(단면) / z(길이, flange 로컬 축)
GRIPPER_BOX_Z_OFFSET = GRIPPER_BOX_DIMENSIONS[2] / 2.0
GRIPPER_OBJECT_ID = 'gripper'
# 시작 직후엔 move_group의 planning_scene 구독이 아직 안 됐을 수 있어서
# (디스커버리 지연), 몇 번 반복 발행함.
GRIPPER_PUBLISH_RETRY_COUNT = 5
GRIPPER_PUBLISH_RETRY_PERIOD_SEC = 2.0


def _normalize(vec):
    length = math.sqrt(sum(c * c for c in vec))
    return [c / length for c in vec]


def _cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _gripper_open_position_for_radius(radius_m: float) -> float:
    """대상 추정 반지름을 [GRIPPER_TARGET_RADIUS_MIN_M, MAX_M] 구간에서
    [GRIPPER_OPEN_POSITION_SMALL, GRIPPER_OPEN_POSITION] 관절값으로 선형
    보간함(위 상수 설명 참고). 범위 밖 값은 clamp."""
    clamped = min(
        max(radius_m, GRIPPER_TARGET_RADIUS_MIN_M), GRIPPER_TARGET_RADIUS_MAX_M
    )
    span = GRIPPER_TARGET_RADIUS_MAX_M - GRIPPER_TARGET_RADIUS_MIN_M
    fraction = (clamped - GRIPPER_TARGET_RADIUS_MIN_M) / span
    return GRIPPER_OPEN_POSITION_SMALL + fraction * (
        GRIPPER_OPEN_POSITION - GRIPPER_OPEN_POSITION_SMALL
    )


def _angle_delta(a_rad: float, b_rad: float) -> float:
    """두 각도(rad) 차이를 [-pi, pi]로 wrap한 뒤 절대값."""
    diff = a_rad - b_rad
    return abs(math.atan2(math.sin(diff), math.cos(diff)))


def _tilt_forward_vector(forward, tilt_rad):
    """forward 벡터의 고도각(elevation)만 tilt_rad만큼 더하고 방위각
    (azimuth)과 크기는 그대로 유지함. [2026-07-29, 손목 특이점 회피] 참고 —
    순수 roll 회전(forward축 둘레)과 달리 이건 forward 자체의 방향을 바꿔
    IK가 요구하는 J5(wrist pitch) 값을 실제로 움직이기 위한 것."""
    if tilt_rad == 0.0:
        return list(forward)
    dx, dy, dz = forward
    horizontal = math.hypot(dx, dy)
    magnitude = math.sqrt(dx * dx + dy * dy + dz * dz)
    if magnitude < 1e-9 or horizontal < 1e-9:
        return list(forward)
    azimuth = math.atan2(dy, dx)
    elevation = math.atan2(dz, horizontal)
    new_elevation = elevation + tilt_rad
    new_horizontal = magnitude * math.cos(new_elevation)
    new_dz = magnitude * math.sin(new_elevation)
    return [
        new_horizontal * math.cos(azimuth),
        new_horizontal * math.sin(azimuth),
        new_dz,
    ]


def _target_radius_azimuth(target: Point):
    """g_base 기준 목표 좌표의 (수평 반지름, 방위각 degree)를 계산."""
    radius = math.hypot(target.x, target.y)
    azimuth_deg = math.degrees(math.atan2(target.y, target.x))
    return radius, azimuth_deg


def _rotation_matrix_to_quat_xyzw(m):
    """3x3 회전행렬(행 단위 리스트) -> 쿼터니언(xyzw). 표준 Shepperd's method."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m[2][1] - m[1][2]) * s
        y = (m[0][2] - m[2][0]) * s
        z = (m[1][0] - m[0][1]) * s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return [x, y, z, w]


def _compute_look_at_quat_xyzw(forward_raw, world_up=WORLD_UP, roll_rad=0.0):
    """그리퍼 로컬 +Z축(정면, 위 상수 설명 참고)이 `forward_raw` 방향을
    향하도록 하는 orientation을 쿼터니언(xyzw)으로 계산 (그래픽스 look-at
    행렬과 같은 원리). `world_up`은 정면 축 둘레 회전(roll)만 고정하는 기준
    벡터 — forward와 거의 평행하면(90도 인근 목표 등) 대체 기준축으로
    degenerate를 피함. `roll_rad`만큼 정면(forward)축 둘레로 추가 회전시켜
    다른 roll 후보를 생성할 수 있음(기본값 0 = 기존 WORLD_UP 단일값과 동일).
    """
    forward = _normalize(forward_raw)

    up_ref = list(world_up)
    if abs(_dot(forward, up_ref)) > 0.99:
        up_ref = [1.0, 0.0, 0.0]

    right0 = _normalize(_cross(up_ref, forward))
    up0 = _cross(forward, right0)

    cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)
    right = [right0[i] * cos_r + up0[i] * sin_r for i in range(3)]
    up = [-right0[i] * sin_r + up0[i] * cos_r for i in range(3)]

    # 회전행렬의 각 열(column) = 로컬 X/Y/Z축이 base(g_base) 좌표계에서
    # 가리키는 방향. 로컬 X=right, 로컬 Y=up, 로컬 Z=forward(그리퍼 정면)로 배정.
    rotation_matrix = [
        [right[0], up[0], forward[0]],
        [right[1], up[1], forward[1]],
        [right[2], up[2], forward[2]],
    ]
    return _rotation_matrix_to_quat_xyzw(rotation_matrix)


class CoordToGoalNode(Node):

    def __init__(self):
        super().__init__('coord_to_goal_node')

        # [2026-07-28, 테스트 모드] True면 target_point를 받아도 자동으로
        # 5단계를 실행하지 않고 /confirm_grasp 서비스 호출을 기다림 — 파지
        # 정확도 튜닝 중 실패한 대상을 매번 수동으로 치워야 재시도를 막을 수
        # 있던 문제(look pose 복귀가 곧 새 방문으로 잡혀 자동 재시도됨) 해결용.
        # 런치 인자 없이 기본은 꺼짐(기존 자동 동작 유지):
        #   ros2 run mycobot_280_pick coord_to_goal_node --ros-args -p require_grasp_confirmation:=true
        self.declare_parameter('require_grasp_confirmation', False)
        self._require_grasp_confirmation = (
            self.get_parameter('require_grasp_confirmation')
            .get_parameter_value()
            .bool_value
        )
        # 테스트 모드에서 target_point는 등록됐지만 아직 /confirm_grasp을
        # 못 받은 상태인지 표시. /emergency_stop이 이 상태의 대기 중인 목표도
        # 그대로 취소함(기존 로직 그대로 재사용, 아래 참고).
        self._awaiting_grasp_confirmation = False

        callback_group = ReentrantCallbackGroup()

        self._moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK_NAME,
            end_effector_name=END_EFFECTOR_NAME,
            group_name=GROUP_NAME,
            callback_group=callback_group,
            # MoveGroup 액션(콜백 기반) 경로를 쓰기 위함. False(기본값)면
            # plan()/wait_until_executed()가 내부에서 rclpy.spin_once()를 호출해서
            # 이 노드를 이미 spin 중인 MultiThreadedExecutor와 충돌함.
            use_move_group_action=True,
        )
        self._moveit2.allowed_planning_time = PLANNING_TIME_SEC
        self._moveit2.num_planning_attempts = PLANNING_ATTEMPTS
        self._moveit2.max_velocity = VELOCITY_SCALING
        self._moveit2.max_acceleration = ACCELERATION_SCALING

        # [2026-07-27, 그리퍼 actuation 1단계] arm_group과 별도 플래닝
        # 그룹이라 별도 MoveIt2(정확히는 pose 관련 메서드가 빠진 하위 클래스
        # MoveIt2Gripper) 인스턴스가 필요함.
        self._gripper_moveit2 = MoveIt2Gripper(
            node=self,
            gripper_joint_names=[GRIPPER_JOINT_NAME],
            open_gripper_joint_positions=[GRIPPER_OPEN_POSITION],
            closed_gripper_joint_positions=[GRIPPER_CLOSED_POSITION],
            gripper_group_name=GRIPPER_GROUP_NAME,
            callback_group=callback_group,
            use_move_group_action=True,
        )
        self._gripper_moveit2.allowed_planning_time = PLANNING_TIME_SEC
        self._gripper_moveit2.num_planning_attempts = PLANNING_ATTEMPTS
        self._gripper_moveit2.max_velocity = VELOCITY_SCALING
        self._gripper_moveit2.max_acceleration = ACCELERATION_SCALING

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # move_to_pose() 실행 중 새 목표가 들어오면 무시하기 위한 플래그.
        self._busy = False
        self._completion_timer = None
        self._clear_timer = None
        self._pending_approach_position = None
        self._pending_target_position = None
        # [2026-07-27, 그리퍼 폭 동적화] target_radius_m 구독으로 받은 최신값을
        # 캐시해뒀다가, target_point 수신 시점에 스냅샷해서 그 사이클 동안은
        # 고정(다음 검출 사이클의 반지름이 끼어들지 않도록). 아직 한 번도
        # 못 받았으면 안전하게 최대(가장 넓게 열기)로 기본값을 둠.
        self._latest_target_radius_m = GRIPPER_TARGET_RADIUS_MAX_M
        self._pending_target_radius_m = GRIPPER_TARGET_RADIUS_MAX_M
        self._return_dwell_timer = None
        self._return_completion_timer = None
        self._pending_result_succeeded = None
        self._look_pose_completion_timer = None
        self._gripper_completion_timer = None
        self._pending_quat = None
        self._align_retry_count = 0
        self._post_grasp_dwell_timer = None

        # 목표 지점에 CollisionObject(구)를 등록해 Octomap이 그 자리 실제
        # 물체(토마토)를 자기 필터로 걸러내도록 함 (위 모듈 docstring 참고).
        self._planning_scene_publisher = self.create_publisher(
            PlanningScene, 'planning_scene', 10
        )

        # 목표 하나의 플래닝/실행 결과(성공/실패)를 발행 — harvest_sequence_node처럼
        # 여러 목표를 순차 발행하는 상위 로직이 다음 목표를 언제 보낼지 판단하는
        # 데 씀(아래 _check_retreat_complete 참고).
        self._plan_result_publisher = self.create_publisher(Bool, 'plan_result', 10)

        # [2026-07-28, 그리퍼 단계 LED 표시] 5단계 중 현재 어느 단계인지를
        # 실물 상단 RGB LED 색으로 보여주기 위한 상태 발행. 이 노드는 값만
        # 발행하고 실제 mc.set_color() 호출은 RPi sync_plan이 이 토픽을
        # 구독해서 처리함(시리얼 포트를 쥔 프로세스가 sync_plan뿐이라 여기서
        # 직접 호출 불가, GRASP_STEP_COLORS 매핑은 sync_plan.py에 동일하게
        # 유지할 것).
        self._grasp_step_publisher = self.create_publisher(Int32, 'grasp_step', 10)

        # 그리퍼 근접거리 depth 노이즈로 self-filter(padding)가 못 걸러내는 잔여
        # voxel이 START_STATE_IN_COLLISION을 유발하는 경우가 있어(카메라 min-range
        # 노이즈로 추정, 2026-07-23 확인), 플래닝 직전 octomap 전체를 한 번 비움.
        self._clear_octomap_client = self.create_client(Empty, '/clear_octomap')

        # roll 후보 탐색(위 ROLL_CANDIDATES_RAD 설명 참고)에 쓰는 IK 서비스.
        self._ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=callback_group
        )
        self._roll_search_forward = None
        self._roll_search_index = 0
        self._roll_search_retry = 0
        # [2026-07-28, 전체 관절 이동량 최소화] 후보 전수 조사 중 "6축 관절
        # 이동량 합이 가장 적은" 후보를 기억해두는 상태(아래
        # _try_next_roll_candidate/_on_roll_candidate_result 참고).
        self._roll_search_current_joint_positions = None
        self._roll_search_best_delta = None
        self._roll_search_best_quat = None
        self._roll_search_best_roll_rad = None

        # target_object <-> 그리퍼 링크 충돌 허용(위 GRIPPER_LINK_NAMES 설명
        # 참고). 이 시점(__init__)은 아직 executor가 이 노드를 spin하기 전이라
        # rclpy.spin_until_future_complete()를 안전하게 blocking 호출할 수 있음
        # (파일 상단 docstring이 경고하는 "이중 spin 충돌"은 executor가 이미
        # spin 중인 상태에서만 발생함).
        self._get_planning_scene_client = self.create_client(
            GetPlanningScene, '/get_planning_scene', callback_group=callback_group
        )
        self._allow_gripper_target_object_collision()

        # 이 워크스페이스의 URDF는 이미 실제 그리퍼(mycobot_280_m5_adaptive_gripper)를
        # 포함하고 있어 아래 임시 박스 콜리전(그리퍼 미포함 URDF 대응용)이 필요 없음 —
        # 오히려 실제 그리퍼 메시와 겹쳐 자기충돌(START_STATE_IN_COLLISION)을 일으켜서
        # 비활성화함. 그리퍼 없는 URDF로 되돌아가면 아래 두 줄을 복원할 것.
        self._gripper_publish_count = 0
        self._gripper_publish_timer = None

        # YOLO 실시간 검출(yolo_d435_detector_node)이 카메라 프레임마다 계속
        # /target_point를 재발행하는데, ReentrantCallbackGroup을 쓰면 이전
        # 메시지가 처리 중이어도 이 콜백이 다른 스레드에서 동시에 또 실행돼
        # `_busy` 체크-후-설정 사이에 race condition이 생겨 두 메시지가 동시에
        # 통과해버림(2026-07-24 실물 재현 — self._clear_timer가 두 번 생성돼
        # 그 중 하나가 취소된 뒤 콜백이 None인 타이머를 참조해 노드가 죽음).
        # 이 구독 콜백만 별도 MutuallyExclusiveCallbackGroup으로 분리해 항상
        # 한 번에 하나씩만 실행되도록 함(다른 콜백들은 기존 ReentrantCallbackGroup
        # 유지 — MoveIt2 액션 응답/완료 폴링 등은 서로 겹쳐 돌아야 함).
        self._target_point_callback_group = MutuallyExclusiveCallbackGroup()
        self._subscription = self.create_subscription(
            PointStamped,
            'target_point',
            self._on_target_point,
            10,
            callback_group=self._target_point_callback_group,
        )

        # [2026-07-27, 그리퍼 폭 동적화] target_point와 함께 발행되는 대상
        # 추정 반지름 구독(위 GRIPPER_TARGET_RADIUS_* 설명 참고). target_point와
        # 같은 그룹에 둬서 target_point 콜백과 겹쳐 돌지 않도록 함(발행 순서상
        # target_radius_m이 먼저 오므로, target_point 처리 시점엔 이미 최신값이
        # 캐시돼 있을 가능성이 높음 — 완벽한 원자성 보장은 아니지만 참고용
        # 정보라 무해함).
        self._target_radius_sub = self.create_subscription(
            Float32,
            'target_radius_m',
            self._on_target_radius,
            10,
            callback_group=self._target_point_callback_group,
        )

        # [Tier4, so101 rviz_control_panel_node 이식용] look pose로 즉시 이동/
        # 소프트 정지를 외부(RViz 인터랙티브 마커 등)에서 트리거할 수 있는
        # 독립 서비스. 기존엔 목표 접근 완료 후 내부적으로만 look pose 복귀
        # 로직(_start_return_to_look_pose)이 있었고, 그와 무관하게 언제든
        # 호출 가능한 서비스가 없었음.
        self._go_to_look_pose_service = self.create_service(
            Trigger, 'go_to_look_pose', self._on_go_to_look_pose_request
        )
        self._emergency_stop_service = self.create_service(
            Trigger, 'emergency_stop', self._on_emergency_stop_request
        )
        # [2026-07-28, 테스트 모드] require_grasp_confirmation:=true일 때만
        # 의미 있음 — 대기 중인 목표를 실제로 실행하도록 확인.
        self._confirm_grasp_service = self.create_service(
            Trigger, 'confirm_grasp', self._on_confirm_grasp_request
        )

        self.get_logger().info(
            'coord_to_goal_node 준비 완료. /target_point 구독 대기 중... '
            f'(require_grasp_confirmation={self._require_grasp_confirmation})'
        )

    def _on_go_to_look_pose_request(self, request, response) -> Trigger.Response:
        if self._busy:
            response.success = False
            response.message = '다른 목표 실행 중이라 거부함'
            return response

        self._busy = True
        self.get_logger().info('go_to_look_pose 요청 수신, look pose로 이동 시작')
        self._moveit2.move_to_configuration(
            LOOK_POSE_JOINT_POSITIONS, joint_names=JOINT_NAMES
        )
        self._look_pose_completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_go_to_look_pose_complete
        )
        response.success = True
        response.message = 'look pose 이동 시작함(완료 여부는 로그로 확인)'
        return response

    def _check_go_to_look_pose_complete(self) -> None:
        if self._moveit2.query_state() != MoveIt2State.IDLE:
            return

        self._look_pose_completion_timer.cancel()
        self._look_pose_completion_timer = None

        if self._moveit2.motion_suceeded:
            self.get_logger().info('go_to_look_pose 이동 완료.')
        else:
            self.get_logger().warn('go_to_look_pose 이동 실패 — 팔 상태 수동 확인 필요.')

        self._busy = False

    def _on_emergency_stop_request(self, request, response) -> Trigger.Response:
        """MoveIt이 실행 중인 궤적을 취소함(소프트 정지).

        주의: 이건 ros2_control/서보 토크를 직접 끊는 하드웨어 e-stop이
        아니라 MoveIt trajectory_execution_manager에 취소를 요청하는
        소프트웨어 정지임 — mycobot 280의 실물 실행은 이 워크스페이스의
        ros2_control(mock_components/GenericSystem)이 아니라 RPi
        (jetcobot_126b)의 별도 브릿지 프로세스를 거쳐 pymycobot으로 나가서
        (docs/look_pose.md "확인 방법" 절 참고), 이 노드가 직접 서보 토크를
        끌 수 있는 표준 ros2_control 경로가 없음. 진짜 하드웨어 정지가
        필요하면 RPi 쪽에서 `mc.release_all_servos()`를 직접 호출할 것.
        """
        self._moveit2.cancel_execution()
        self._gripper_moveit2.cancel_execution()

        for timer_attr in (
            '_completion_timer',
            '_return_completion_timer',
            '_return_dwell_timer',
            '_clear_timer',
            '_look_pose_completion_timer',
            '_gripper_completion_timer',
            '_post_grasp_dwell_timer',
        ):
            timer = getattr(self, timer_attr)
            if timer is not None:
                timer.cancel()
                setattr(self, timer_attr, None)

        self._busy = False
        self._awaiting_grasp_confirmation = False
        self.get_logger().warn(
            'emergency_stop 요청 수신 — 현재 궤적 실행 취소함(소프트 정지)'
        )
        response.success = True
        response.message = (
            '궤적 실행 취소함(소프트 정지) — 하드웨어 토크 차단이 아님, '
            '실물 토크까지 끊으려면 RPi에서 release_all_servos() 직접 호출 필요'
        )
        return response

    def _on_confirm_grasp_request(self, request, response) -> Trigger.Response:
        """require_grasp_confirmation:=true일 때 대기 중인 목표를 실제로
        실행함. 대기 중인 목표가 없으면(확인 대기 상태가 아니면) 거부."""
        if not self._awaiting_grasp_confirmation:
            response.success = False
            response.message = '확인 대기 중인 목표 없음'
            return response

        self._awaiting_grasp_confirmation = False
        self.get_logger().info('grasp 확인됨 — 플래닝 시작')
        self._clear_timer = self.create_timer(
            OCTOMAP_CLEAR_DELAY_SEC, self._start_planning
        )
        response.success = True
        response.message = '확인 완료, 플래닝 시작함'
        return response

    def _lookup_target_transform(self, msg: PointStamped):
        """`msg.header.stamp`(검출이 실제로 유효했던 시점) 기준으로 TF를
        조회함. tf2 버퍼에 그 시점 데이터가 아직 없으면(디스커버리/네트워크
        지연 등) 최신 TF로 폴백하되, 그 경우 검출 시점과 로봇 자세가 다를 수
        있음을 경고함. [Tier1, so101 교훈] 기존엔 항상 `Time()`(최신)으로만
        조회해서, 콜백 처리 시점의 "최신" TF가 실제로 검출이 유효했던 순간의
        로봇 자세를 반영하고 있었는지 보장이 없었음.
        """
        try:
            return self._tf_buffer.lookup_transform(
                BASE_LINK_NAME,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warn(
                f'검출 시점 기준 TF 조회 실패, 최신 TF로 폴백함(검출 시점과 '
                f'현재 로봇 자세가 다를 수 있음): {exc}'
            )
            return self._tf_buffer.lookup_transform(
                BASE_LINK_NAME,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=1.0),
            )

    def _is_target_within_safe_range(self, target: Point) -> bool:
        """[Tier1, so101 교훈] 목표 반지름/방위각이 정상 작업 범위를 크게
        벗어나면 플래닝/실행 전에 거부함(상수 정의부 설명 참고)."""
        radius, azimuth_deg = _target_radius_azimuth(target)
        if not (MIN_TARGET_RADIUS_M <= radius <= MAX_TARGET_RADIUS_M):
            self.get_logger().error(
                f'목표 반지름({radius:.3f}m)이 안전 범위 '
                f'[{MIN_TARGET_RADIUS_M}, {MAX_TARGET_RADIUS_M}]를 벗어나 '
                f'실행 거부: ({target.x:.3f}, {target.y:.3f}, {target.z:.3f})'
            )
            return False
        if abs(azimuth_deg) > MAX_TARGET_AZIMUTH_DEG:
            self.get_logger().error(
                f'목표 방위각({azimuth_deg:.1f}도)이 안전 범위 '
                f'±{MAX_TARGET_AZIMUTH_DEG}도를 벗어나 실행 거부: '
                f'({target.x:.3f}, {target.y:.3f}, {target.z:.3f})'
            )
            return False
        return True

    def _on_target_radius(self, msg: Float32) -> None:
        self._latest_target_radius_m = msg.data

    def _on_target_point(self, msg: PointStamped) -> None:
        if self._busy:
            self.get_logger().warn('이전 목표 실행 중이라 새 좌표는 무시함')
            return

        try:
            if msg.header.frame_id and msg.header.frame_id != BASE_LINK_NAME:
                transform = self._lookup_target_transform(msg)
                point_in_base = do_transform_point(msg, transform)
            else:
                point_in_base = msg
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().error(
                f'TF2 변환 실패 ({msg.header.frame_id} -> {BASE_LINK_NAME}): {exc}'
            )
            return

        target = point_in_base.point

        if not self._is_target_within_safe_range(target):
            return

        # [2026-07-27, 실물 파지 정확도] flange가 실제로 이동할 목표(파지
        # 지점)만 그리퍼 길이만큼 당김(위 GRIPPER_LENGTH_OFFSET_M 설명 참고).
        # target(원본, 실제 토마토 위치)은 안전 범위 검증/Octomap 클리어용
        # CollisionObject에 그대로 쓰고, 이 보정된 값만 정렬/직진 접근 목표로 씀.
        grasp_target_x = target.x - GRIPPER_LENGTH_OFFSET_M
        approach_position = [grasp_target_x - APPROACH_OFFSET_X, target.y, target.z]

        # [2026-07-28] 원본 target은 MIN_TARGET_RADIUS_M 범위 안이어도,
        # GRIPPER_LENGTH_OFFSET_M+APPROACH_OFFSET_X만큼 베이스 쪽으로 당긴
        # 정렬 위치는 그 범위 밖(베이스 근처 도달 불가 사각지대)으로 빠질 수
        # 있음 — 가까운 목표(반지름 20cm대)에서 실측 재현 확인(정렬이 매번
        # STATUS_ABORTED). target과 같은 최소 반지름 기준으로 정렬 위치도
        # 검증.
        approach_radius = math.hypot(approach_position[0], approach_position[1])
        if approach_radius < MIN_TARGET_RADIUS_M:
            self.get_logger().error(
                f'정렬 위치 반지름({approach_radius:.3f}m)이 최소 안전 반지름'
                f'({MIN_TARGET_RADIUS_M}m) 미만 — GRIPPER_LENGTH_OFFSET_M+'
                f'APPROACH_OFFSET_X가 목표를 베이스 근처 도달 불가 지점까지 '
                f'당김. 실행 거부: target=({target.x:.3f}, {target.y:.3f}, '
                f'{target.z:.3f}), approach={[round(c, 3) for c in approach_position]}'
            )
            return

        self._busy = True
        self._align_retry_count = 0
        self._publish_target_collision_object(target)
        self._pending_approach_position = approach_position
        self._pending_target_position = [grasp_target_x, target.y, target.z]
        self._pending_target_radius_m = self._latest_target_radius_m
        self.get_logger().info(
            f'목표 지점({target.x:.3f}, {target.y:.3f}, {target.z:.3f}), '
            f'그리퍼 길이 보정 후 flange 목표 x={grasp_target_x:.3f}에 '
            f'Octomap 클리어용 구 등록'
        )

        if self._require_grasp_confirmation:
            self._awaiting_grasp_confirmation = True
            self.get_logger().warn(
                '[테스트 모드] 실행 대기 — /confirm_grasp 서비스를 호출해야 '
                '플래닝이 시작됨(취소하려면 /emergency_stop).'
            )
            return

        # Octomap이 다음 point cloud로 자기 필터 갱신을 반영할 때까지 기다린
        # 뒤에 플래닝을 시작함 (바로 플래닝하면 아직 예전 voxel이 남아있어서
        # 여전히 충돌로 판정될 수 있음).
        self.get_logger().info(f'{OCTOMAP_CLEAR_DELAY_SEC}초 대기 후 플래닝 시작')
        self._clear_timer = self.create_timer(
            OCTOMAP_CLEAR_DELAY_SEC, self._start_planning
        )

    def _allow_gripper_target_object_collision(self) -> None:
        """target_object(위 상수 설명 참고)와 그리퍼 링크 간 충돌을 Allowed
        Collision Matrix(ACM)에 미리 허용해둠. Octomap이나 다른 world 장애물과의
        충돌 검사는 그대로 유지되고, 오직 우리가 등록한 이 특정 구와 그리퍼
        사이만 예외로 둠.

        주의: PlanningScene diff로 ACM을 발행하면 기존 전체 매트릭스가 아니라
        발행한 내용으로 통째로 대체됨(2026-07-24 직접 실험으로 확인 — 부분
        ACM만 발행했다가 SRDF의 self-collision-disable 항목이 전부 날아가는
        사고가 남, move_group 재시작으로 복구함). 그래서 반드시 현재 전체 ACM을
        먼저 조회해 기존 항목을 보존한 채로 target_object 행/열만 추가해서
        다시 발행해야 함.
        """
        if not self._get_planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                'get_planning_scene 서비스 준비 안 됨, target_object ACM 허용 '
                '설정을 건너뜀(그리퍼가 target_object와 충돌 판정될 수 있음)'
            )
            return

        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self._get_planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        current_acm = future.result().scene.allowed_collision_matrix

        # [2026-07-27, 그리퍼 actuation 1단계] "직진 접근" 단계를 추가해 실제로
        # end effector(joint6_flange)를 목표 지점 그 자체까지 이동시켜보니,
        # 그 자리에 등록된 target_object 구가 (그리퍼 링크는 이미 허용돼
        # 있었지만) flange 자신과는 여전히 충돌로 잡혀서 "Unable to sample any
        # valid states for goal tree"로 매번 플래닝이 실패하는 것을 확인함
        # (mock 스택 실측) — flange도 같이 허용 목록에 추가. [같은 날 후속
        # 조사] joint5/joint6도 같은 이유로 필요함이 추가로 확인됨(위
        # WRIST_LINK_NAMES 설명 참고).
        allowed_link_names = GRIPPER_LINK_NAMES + WRIST_LINK_NAMES + [END_EFFECTOR_NAME]

        entry_names = list(current_acm.entry_names)
        entry_rows = [list(entry.enabled) for entry in current_acm.entry_values]

        if TARGET_OBJECT_ID not in entry_names:
            for row in entry_rows:
                row.append(False)
            for i, name in enumerate(entry_names):
                entry_rows[i][-1] = name in allowed_link_names

            new_row = [name in allowed_link_names for name in entry_names]
            new_row.append(False)  # target_object 자기 자신과의 항목은 무의미
            entry_names.append(TARGET_OBJECT_ID)
            entry_rows.append(new_row)

        updated_acm = AllowedCollisionMatrix()
        updated_acm.entry_names = entry_names
        updated_acm.default_entry_names = list(current_acm.default_entry_names)
        updated_acm.default_entry_values = list(current_acm.default_entry_values)
        for row in entry_rows:
            entry = AllowedCollisionEntry()
            entry.enabled = row
            updated_acm.entry_values.append(entry)

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = updated_acm
        self._planning_scene_publisher.publish(scene)
        self.get_logger().info(
            f'target_object <-> 그리퍼 링크+flange({len(allowed_link_names)}개) 충돌 '
            '허용 ACM 등록 완료'
        )

    def _publish_gripper_collision_object(self) -> None:
        self._gripper_publish_count += 1
        if self._gripper_publish_count >= GRIPPER_PUBLISH_RETRY_COUNT:
            self._gripper_publish_timer.cancel()
            self._gripper_publish_timer = None

        collision_object = CollisionObject()
        collision_object.header.frame_id = END_EFFECTOR_NAME
        collision_object.id = GRIPPER_OBJECT_ID
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = GRIPPER_BOX_DIMENSIONS
        collision_object.primitives = [primitive]

        pose = Pose()
        pose.position.z = GRIPPER_BOX_Z_OFFSET
        pose.orientation.w = 1.0
        collision_object.primitive_poses = [pose]

        attached_object = AttachedCollisionObject()
        attached_object.link_name = END_EFFECTOR_NAME
        attached_object.object = collision_object
        # 팔 자신이 이 형상에 닿아도 충돌로 안 잡히게 함.
        attached_object.touch_links = [END_EFFECTOR_NAME, 'joint6']

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached_object]
        self._planning_scene_publisher.publish(scene)
        self.get_logger().info(
            f'그리퍼 임시 충돌 형상 등록 ({self._gripper_publish_count}/'
            f'{GRIPPER_PUBLISH_RETRY_COUNT})'
        )

    def _publish_target_collision_object(self, target: Point) -> None:
        collision_object = CollisionObject()
        collision_object.header.frame_id = BASE_LINK_NAME
        collision_object.id = TARGET_OBJECT_ID
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [TARGET_OBJECT_RADIUS]
        collision_object.primitives = [primitive]

        pose = Pose()
        pose.position = target
        pose.orientation.w = 1.0
        collision_object.primitive_poses = [pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [collision_object]
        self._planning_scene_publisher.publish(scene)

    def _compute_forward_vector(self, target_position):
        """현재 end-effector 위치 -> 목표 위치 방향 벡터(정규화 안 됨)를
        계산. TF 조회 실패나 목표가 현재 위치와 거의 같은 경우(degenerate)엔
        None을 반환함(호출부에서 고정 orientation으로 폴백).
        """
        try:
            ee_transform = self._tf_buffer.lookup_transform(
                BASE_LINK_NAME,
                END_EFFECTOR_NAME,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warn(
                f'현재 EE 위치 TF 조회 실패, 고정 orientation으로 폴백: {exc}'
            )
            return None

        ee_translation = ee_transform.transform.translation
        ee_position = [ee_translation.x, ee_translation.y, ee_translation.z]
        forward = [t - e for t, e in zip(target_position, ee_position)]

        if math.sqrt(sum(c * c for c in forward)) < 1e-6:
            self.get_logger().warn(
                '목표가 현재 EE 위치와 거의 같아 forward 벡터가 degenerate함, '
                '고정 orientation으로 폴백'
            )
            return None

        return forward

    def _start_planning(self) -> None:
        self._clear_timer.cancel()
        self._clear_timer = None

        if self._clear_octomap_client.service_is_ready():
            self._clear_octomap_client.call_async(Empty.Request())

        forward = self._compute_forward_vector(self._pending_target_position)
        if forward is None:
            self._finalize_planning(FALLBACK_APPROACH_QUAT_XYZW)
            return

        # [2026-07-29, 손목 특이점 회피] 재탐색 시 기준이 되는 원래(안 기운)
        # forward — FORWARD_TILT_CANDIDATES_RAD는 항상 이 값에서부터 상대
        # 틸트를 적용함(누적 틸트 방지).
        self._roll_search_base_forward = forward
        self._roll_search_tilt_index = 0
        self._roll_search_forward = forward
        self._roll_search_index = 0
        self._roll_search_retry = 0
        # [2026-07-28, 전체 관절 이동량 최소화] 정렬 시작 시점의 6축 관절각을
        # 스냅샷 — 후보들이 이 자세에서 얼마나 움직여야 하는지 비교하는
        # 기준점. joint_states를 아직 못 받았으면 None(이 경우 아래에서
        # 기존처럼 "첫 feasible 후보 즉시 채택" 동작으로 폴백).
        self._roll_search_current_joint_positions = self._get_current_arm_joint_positions()
        self._roll_search_best_delta = None
        self._roll_search_best_quat = None
        self._roll_search_best_roll_rad = None
        self._roll_search_best_j5_rad = None
        self._try_next_roll_candidate()

    def _get_current_arm_joint_positions(self):
        """JOINT_NAMES(6축) 각각의 현재 관절각(rad) 딕셔너리. joint_states를
        아직 못 받았으면(예: 노드 기동 직후) None."""
        joint_state = self._moveit2.joint_state
        if joint_state is None:
            return None
        names = list(joint_state.name)
        positions = {}
        for joint_name in JOINT_NAMES:
            try:
                idx = names.index(joint_name)
            except ValueError:
                return None
            positions[joint_name] = joint_state.position[idx]
        return positions

    def _try_next_roll_candidate(self) -> None:
        """ROLL_CANDIDATES_RAD를 순서대로 compute_ik로 사전 체크함(위
        ROLL_CANDIDATES_RAD 설명 참고). 후보마다 ROLL_CANDIDATE_IK_RETRIES번
        재시도함(IK가 확률적이라 한 번만 시도하면 실제로 유효한 후보를 놓칠 수
        있음). [2026-07-27, 손목 회전 최소화 / 2026-07-28, 전체 관절로 확장]
        예전엔 "IK가 풀리는 첫 후보"를 그 자리에서 바로 채택했는데, 그
        후보가 현재 관절 자세와 가까운지는 전혀 고려하지 않아 정렬 실행 시
        관절이 크게 도는 경우가 관찰됨(처음엔 손목만 봐서 손목 회전은
        줄였지만, joint1(베이스)이 140도 가까이 스윙하는 사례가 실물 로그로
        확인됨 — 사용자 피드백, "몸을 틀면서 이동"). 토마토는 거의 구형이라
        roll 값 자체는 파지 성공에 영향이 없으므로(순수 IK-feasibility
        목적), feasible한 후보를 즉시 채택하지 않고 전수 조사해서 "6축 관절
        이동량 합이 가장 적은" 후보를 고름(아래 _on_roll_candidate_result).
        현재 관절 자세를 모르면(위 _get_current_arm_joint_positions 참고)
        비교 기준이 없으므로 기존처럼 첫 feasible 후보를 즉시 채택. 전체
        후보가 다 실패하면 0번 후보(기존 WORLD_UP 단일값과 동일)로 진행 —
        compute_ik는 사전 필터일 뿐이라 실제 move_to_pose의 OMPL 플래닝(더
        넉넉한 시간/재시도)이 최종 판단임.
        """
        if self._roll_search_index >= len(ROLL_CANDIDATES_RAD):
            if self._roll_search_best_quat is not None:
                # [2026-07-29, 손목 특이점 회피] roll 후보 전수 조사로 고른
                # 최적해가 여전히 J5 특이점 근처면, roll을 아무리 바꿔봐야
                # 소용없으므로(위 WRIST_SINGULARITY_MARGIN_RAD 설명 참고)
                # forward 벡터 자체를 기울여 재탐색. 남은 틸트 후보가 있을
                # 때만 재시도 — 다 써버리면(FORWARD_TILT_CANDIDATES_RAD 소진)
                # 특이점 근처라도 그냥 진행(그리퍼는 못 뻗는 것보단 도는 게
                # 나음).
                near_singularity = (
                    self._roll_search_best_j5_rad is not None
                    and abs(self._roll_search_best_j5_rad) < WRIST_SINGULARITY_MARGIN_RAD
                )
                if near_singularity and (
                    self._roll_search_tilt_index + 1 < len(FORWARD_TILT_CANDIDATES_RAD)
                ):
                    self._roll_search_tilt_index += 1
                    tilt_rad = FORWARD_TILT_CANDIDATES_RAD[self._roll_search_tilt_index]
                    self.get_logger().warn(
                        f'roll 후보 최적해가 J5 특이점 근처'
                        f'({math.degrees(self._roll_search_best_j5_rad):.1f}도) — '
                        f'forward 벡터를 {math.degrees(tilt_rad):.0f}도 기울여 재탐색'
                    )
                    self._roll_search_forward = _tilt_forward_vector(
                        self._roll_search_base_forward, tilt_rad
                    )
                    self._roll_search_index = 0
                    self._roll_search_retry = 0
                    self._roll_search_best_delta = None
                    self._roll_search_best_quat = None
                    self._roll_search_best_roll_rad = None
                    self._roll_search_best_j5_rad = None
                    self._try_next_roll_candidate()
                    return

                if near_singularity:
                    self.get_logger().warn(
                        f'forward 기울임(틸트 후보 전부 소진)에도 J5가 여전히 '
                        f'특이점 근처({math.degrees(self._roll_search_best_j5_rad):.1f}도)'
                        ' — 그대로 진행'
                    )

                self.get_logger().info(
                    f'roll 후보 전수 조사 완료 — 전체 관절 이동량 최소(roll '
                    f'{math.degrees(self._roll_search_best_roll_rad):.0f}도, '
                    f'예상 이동량 합 {math.degrees(self._roll_search_best_delta):.0f}도) '
                    '후보 채택'
                )
                self._finalize_planning(self._roll_search_best_quat)
                return

            fallback_quat = _compute_look_at_quat_xyzw(
                self._roll_search_forward, roll_rad=ROLL_CANDIDATES_RAD[0]
            )
            self._finalize_planning(fallback_quat)
            return

        roll_rad = ROLL_CANDIDATES_RAD[self._roll_search_index]
        quat = _compute_look_at_quat_xyzw(self._roll_search_forward, roll_rad=roll_rad)

        # [2026-07-27 정정] 정렬 위치(APPROACH_OFFSET_X만큼 로봇 쪽으로 당긴,
        # 즉 더 가까운 지점)가 아니라 실제 목표 지점(_pending_target_position,
        # 더 멀리 뻗어야 하는 쪽)으로 사전 IK를 검증함 — 그리퍼 actuation
        # 1단계에서 "직진 접근"(목표 지점 그 자체로 이동) 단계를 추가해보니,
        # 정렬 위치에서는 풀리는 orientation이 그보다 8cm 더 먼 목표 지점에서는
        # "Unable to sample any valid states for goal tree"로 실패하는 경우를
        # mock 스택 실측으로 확인함(더 먼 지점이 관절 한계에 더 가까워 항상
        # 더 어려운 제약이므로, 이쪽으로 사전 검증하면 더 가까운 정렬 위치는
        # 사실상 항상 따라옴).
        request = GetPositionIK.Request()
        ik_request = PositionIKRequest()
        ik_request.group_name = GROUP_NAME
        ik_request.ik_link_name = END_EFFECTOR_NAME
        ik_request.avoid_collisions = True
        ik_request.timeout = Duration(
            seconds=ROLL_CANDIDATE_IK_TIMEOUT_SEC
        ).to_msg()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = BASE_LINK_NAME
        position = self._pending_target_position
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]
        ik_request.pose_stamped = pose_stamped
        request.ik_request = ik_request

        future = self._ik_client.call_async(request)
        future.add_done_callback(
            functools.partial(
                self._on_roll_candidate_result, roll_rad=roll_rad, quat=quat
            )
        )

    def _on_roll_candidate_result(self, future, roll_rad, quat) -> None:
        """[2026-07-27 되돌림] 목표 지점 IK 해를 시드로 정렬 위치 IK까지 같이
        요구하는 방식을 이번 세션에 시도했다가 되돌림 — 스크래치패드
        스크립트(`/tmp`, 세션 한정)로 직접 재현해보니 `avoid_collisions=True`
        + 정확한 orientation(tolerance 없음) 조합 자체가 이 그리퍼 자세에서는
        1초 타임아웃을 줘도 성공률이 사실상 0에 가까움(목표 지점 단독
        IK조차도 마찬가지) — 즉 이 사전 필터는 애초에 "가끔 맞으면 좋고 아니면
        말고" 수준이었지 신뢰 가능한 검증 수단이 아니었음이 이번에 실측으로
        드러남. 시드 조건까지 얹으면 이미 낮은 성공률이 더 낮아져 거의 항상
        exhaustion으로 빠짐 — 사실상 무의미한 추가 지연이었음. 실제 "직진
        접근" reachability 문제는 아래 `_verify_grasp_approach_reachable`
        (dry-run 재시도 루프)로 대응함 — 문서 "직진 접근 시드 검증, 되돌림"
        절 참고.
        """
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - IK 서비스 호출 실패는 재시도로 넘어감
            self.get_logger().warn(f'compute_ik 호출 실패, 재시도: {exc}')
            response = None

        if response is not None and response.error_code.val == MoveItErrorCodes.SUCCESS:
            if self._roll_search_current_joint_positions is None:
                # 현재 관절 자세를 모르면 비교 기준이 없으므로 기존 동작대로
                # 첫 feasible 후보를 즉시 채택.
                self.get_logger().info(
                    f'roll 후보 {math.degrees(roll_rad):.0f}도에서 IK 확인됨, '
                    '이 orientation 채택(현재 관절 자세 미수신, 이동량 비교 생략)'
                )
                self._finalize_planning(quat)
                return

            # [2026-07-28, 전체 관절 이동량 최소화] 손목 하나만 보면 다른
            # 관절(특히 joint1 베이스 회전)이 큰 값으로 튀어도 못 잡아서
            # 실물에서 베이스가 140도 가까이 스윙하는 사례가 나왔음 — 6축
            # 전체에 대해 현재 자세와의 각도차 합을 계산해 비교.
            try:
                solution_names = list(response.solution.joint_state.name)
                solution_positions = list(response.solution.joint_state.position)
                delta = 0.0
                for joint_name, current_rad in self._roll_search_current_joint_positions.items():
                    idx = solution_names.index(joint_name)
                    target_rad = solution_positions[idx]
                    delta += _angle_delta(target_rad, current_rad)
                # [2026-07-29, 손목 특이점 회피] 이 후보가 요구하는 J5 값도
                # 같이 뽑아둠 — 전수 조사가 끝난 뒤 최적해가 특이점 근처인지
                # 판단하는 데 씀(_try_next_roll_candidate exhaustion 분기).
                j5_rad = None
                if WRIST_PITCH_JOINT_NAME in solution_names:
                    j5_rad = solution_positions[solution_names.index(WRIST_PITCH_JOINT_NAME)]
            except ValueError:
                delta = None
                j5_rad = None

            if delta is not None:
                self.get_logger().info(
                    f'roll 후보 {math.degrees(roll_rad):.0f}도에서 IK 확인됨 '
                    f'(예상 전체 관절 이동량 합 {math.degrees(delta):.0f}도)'
                )
                if self._roll_search_best_delta is None or delta < self._roll_search_best_delta:
                    self._roll_search_best_delta = delta
                    self._roll_search_best_quat = quat
                    self._roll_search_best_roll_rad = roll_rad
                    self._roll_search_best_j5_rad = j5_rad
                # 즉시 채택하지 않고 다음 후보까지 계속 조사(전수 조사 후
                # _try_next_roll_candidate의 exhaustion 분기에서 최소 회전량
                # 후보를 최종 채택함).
                self._roll_search_retry = 0
                self._roll_search_index += 1
                self._try_next_roll_candidate()
                return

            # IK 응답에 손목 관절이 없는 예외적인 경우 — 회전량 비교 불가하니
            # 기존 동작대로 즉시 채택.
            self.get_logger().info(
                f'roll 후보 {math.degrees(roll_rad):.0f}도에서 IK 확인됨, 이 orientation 채택'
            )
            self._finalize_planning(quat)
            return

        self._roll_search_retry += 1
        if self._roll_search_retry >= ROLL_CANDIDATE_IK_RETRIES:
            self._roll_search_retry = 0
            self._roll_search_index += 1
        self._try_next_roll_candidate()

    def _move_arm(self, position, quat, tolerance_position=0.005) -> None:
        # tolerance 기본값(0.001/0.001)은 IK가 이 정확한 위치+방향을 동시에
        # 만족하는 해를 못 찾는 경우가 많아(OMPL이 짧은 시간 안에 못 찾음),
        # orientation을 넉넉히 풀어뒀었음 — "위치는 정확히, 방향은 근사치면
        # 충분한 접근(pregrasp) 자세"라는 판단이었음.
        #
        # [2026-07-30] 0.5 -> 0.1로 조임. 위 판단에 구멍이 있었음: 정렬이 방향
        # 오차를 남기면 그 오차를 다음 단계인 직진 접근(3/5, Cartesian)이 대신
        # 갚아야 함. Cartesian은 정확한 자세를 요구하므로, 접근 구간이 "4cm
        # 전진"이 아니라 "4cm 전진 + 누적 방향오차 보정"을 동시에 하게 되고,
        # 그게 가장 민감한 접촉 구간에서 벌어짐.
        #
        # 실물 로그로 확인한 증상(2026-07-30):
        #   - 접근 구간 4cm 이동에 관절이 총 154° 움직임(J4 혼자 64°,
        #     J5는 -9.5~-21.1°로 손목 특이점 마진 밖이었으므로 특이점 문제가
        #     아니라 순수한 자세 재배치였음)
        #   - move_group이 Cartesian 경로를 44.8% / 75%만 따라가고 중도 포기
        #     ("followed ...% of requested trajectory")
        #   - 사용자 관측: "3/5부터 자세가 많이 바뀐다", 접근 중 떨림
        #   - 명령 스트림 자체는 방향 반전 0%로 완전히 단조로웠으므로(로그 분석)
        #     떨림의 원인은 relay/서보 speed가 아니었음 — APPROACH_VELOCITY_
        #     SCALING을 0.05->0.1로 올려도(speed 10->20) 떨림이 그대로였고,
        #     후퇴는 이미 0.2(speed 40)인데도 떨렸음. 즉 속도는 변수가 아님.
        #
        # [2026-07-30, 2차 — 0.1은 무리였음] 0.1(5.7°)로 조였더니 정렬이 바로
        # STATUS_ABORTED로 실패함. 서로 다른 두 좌표(정렬 위치 반지름 0.095m,
        # 0.109m)에서 모두 실패했으므로 좌표 탓이 아니라 tolerance 탓으로 확인됨.
        # 원인은 이 로봇/솔버 조합의 알려진 약점 — 정확한 orientation을 요구하는
        # 목표는 OMPL 목표 샘플링이 거의 못 풀음(roll 후보 탐색 개발 때
        # compute_ik + 정확한 orientation + avoid_collisions 조합의 성공률이
        # 사실상 0에 가까웠던 것과 같은 원인, _on_roll_candidate_result 주석 참고).
        # 그래서 0.2(11.5°)로 절충함 — 28.6°보다는 접근 구간이 갚을 오차가
        # 절반 이하로 줄고, 정렬 성공률은 유지하는 지점을 찾는 것이 목표.
        # 0.2에서도 정렬이 실패하면 0.3까지 완화할 것. 반대로 0.2가 안정적이면
        # 접근 구간 관절 이동량(이전 실측 154°/4cm)이 얼마나 줄었는지로 효과를
        # 판정할 수 있음.
        self._moveit2.move_to_pose(
            position=position,
            quat_xyzw=quat,
            cartesian=False,
            tolerance_position=tolerance_position,
            tolerance_orientation=0.2,
        )

    def _move_arm_cartesian(self, position, quat, on_success, on_failure) -> None:
        """[2026-07-27] Cartesian 경로로 이동(상수 정의부 설명 참고).

        주의: `MoveIt2.move_to_pose(cartesian=True)`를 그대로 쓰면
        `use_move_group_action` 설정과 무관하게 내부적으로 blocking
        `plan()`(스스로 `rclpy.spin_once()`를 돌림)을 타서, 이 노드를 이미
        spin 중인 MultiThreadedExecutor와 충돌함(파일 상단 docstring이 경고
        하는 문제 그대로 재현됨). 그래서 non-blocking인
        `plan_async()`+`get_trajectory()`+`execute()` 조합을 직접 써서
        완료 확인은 기존과 동일하게 `query_state()` 폴링으로 처리함.
        """
        future = self._moveit2.plan_async(
            position=position,
            quat_xyzw=quat,
            frame_id=BASE_LINK_NAME,
            cartesian=True,
            max_step=CARTESIAN_MAX_STEP_M,
            tolerance_position=0.005,
            tolerance_orientation=0.5,
        )
        if future is None:
            on_failure()
            return
        future.add_done_callback(
            functools.partial(
                self._on_cartesian_plan_done, on_success=on_success, on_failure=on_failure
            )
        )

    def _on_cartesian_plan_done(self, future, on_success, on_failure) -> None:
        trajectory = self._moveit2.get_trajectory(
            future,
            cartesian=True,
            cartesian_fraction_threshold=CARTESIAN_FRACTION_THRESHOLD,
        )
        if trajectory is None:
            on_failure()
            return
        self._moveit2.execute(trajectory)
        on_success()

    def _publish_grasp_step(self, step: int) -> None:
        msg = Int32()
        msg.data = step
        self._grasp_step_publisher.publish(msg)

    def _abort_to_return(self, reason: str) -> None:
        """[2026-07-27] 5단계(정렬/그리퍼 열기/직진 접근/파지/후퇴) 중 어느
        하나라도 실패하면 그 자리에서 중단하고 나머지 단계는 건너뛴 채 바로
        look pose 복귀를 시도함(so101 교훈 — "실패 시 그 자리에서 중단").
        """
        self._publish_grasp_step(GRASP_STEP_IDLE)
        self.get_logger().warn(f'{reason} — 이후 단계 건너뛰고 look pose 복귀 시도.')
        self._pending_result_succeeded = False
        self._return_dwell_timer = self.create_timer(
            RETURN_TO_LOOK_POSE_DELAY_SEC, self._start_return_to_look_pose
        )

    def _finalize_planning(self, approach_quat) -> None:
        self._publish_grasp_step(GRASP_STEP_ALIGN)
        self._pending_quat = approach_quat
        approach_position = self._pending_approach_position

        self.get_logger().info(
            f'[1/5 정렬] 목표 위치로 플래닝: {approach_position}, orientation(xyzw): '
            f'{[round(c, 3) for c in approach_quat]}'
        )
        self._move_arm(approach_position, approach_quat)
        # wait_until_executed()는 내부적으로 rclpy.spin_once()를 호출해서 이미
        # 돌고 있는 MultiThreadedExecutor와 충돌하므로 쓰지 않음. 대신 같은
        # executor가 처리하는 타이머로 완료 여부만 폴링함.
        self._completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_align_complete
        )

    def _check_align_complete(self) -> None:
        if self._moveit2.query_state() != MoveIt2State.IDLE:
            return
        self._completion_timer.cancel()
        self._completion_timer = None

        if not self._moveit2.motion_suceeded:
            self._abort_to_return('[1/5 정렬] 실패')
            return

        self.get_logger().info('[1/5 정렬] 완료.')
        self._verify_grasp_approach_reachable()

    def _verify_grasp_approach_reachable(self) -> None:
        """[2026-07-27, "직진 접근" reachability 수정] 문서 "미해결: 직진 접근
        reachability" 절의 유력 가설 — 정렬(1/5) 단계는 OMPL(RRTConnect)이
        목표 지점과 무관하게 임의의 관절해를 고르므로(orientation tolerance가
        0.5rad로 넉넉해서 후보가 넓음), 그 해가 관절 한계/특이점에 가까우면
        그 다음 "직진 접근"(목표 지점까지 8cm Cartesian)이 거의 못 움직이고
        막힌다는 것 — 을 실제로 막기 위한 조치.

        같은 정렬 위치를 목표로 한 IK를 시드로 써서 두 단계를 한 번에
        묶으려는 시도를 이번 세션에 먼저 해봤으나(스크래치패드로 직접 검증),
        `avoid_collisions=True` + 정확한 orientation 조합 자체의 IK 성공률이
        이 그리퍼 자세에서는 1초를 줘도 사실상 0에 가까워 실효성이 없었음
        (위 `_on_roll_candidate_result` 주석 참고). 그래서 대신, 정렬 실행
        직후 **실행하지 않고** 직진 접근 Cartesian 경로가 실제로 끝까지
        이어지는지 dry-run으로 미리 확인하고, 안 이어지면 정렬을 다시 풀어서
        (OMPL은 같은 목표라도 매 시도마다 다른 관절해를 고르는 확률적 플래너임
        — docs 여러 곳에서 이미 확인됨) 이어지는 해가 나올 때까지 반복함.
        정렬/Cartesian 둘 다 이미 개별적으로 검증된 경로라 새 실패 유형을
        추가하지 않으면서 "하나의 문제로 묶기"를 재시도 루프로 구현한 것.
        """
        self._check_cartesian_reachable(
            self._pending_target_position,
            self._pending_quat,
            self._on_grasp_approach_reachability_checked,
        )

    def _check_cartesian_reachable(self, position, quat, on_result) -> None:
        """`_move_arm_cartesian`과 같은 Cartesian 플래닝이지만 `execute()`를
        호출하지 않고 목표 fraction 이상 풀리는지만 확인함(dry-run)."""
        future = self._moveit2.plan_async(
            position=position,
            quat_xyzw=quat,
            frame_id=BASE_LINK_NAME,
            cartesian=True,
            max_step=CARTESIAN_MAX_STEP_M,
            tolerance_position=0.005,
            tolerance_orientation=0.5,
        )
        if future is None:
            on_result(False)
            return
        future.add_done_callback(
            functools.partial(self._on_cartesian_dry_run_done, on_result=on_result)
        )

    def _on_cartesian_dry_run_done(self, future, on_result) -> None:
        trajectory = self._moveit2.get_trajectory(
            future,
            cartesian=True,
            cartesian_fraction_threshold=CARTESIAN_FRACTION_THRESHOLD,
        )
        on_result(trajectory is not None)

    def _on_grasp_approach_reachability_checked(self, reachable: bool) -> None:
        if reachable:
            if self._align_retry_count > 0:
                self.get_logger().info(
                    f'[1/5 정렬] 재시도 {self._align_retry_count}회 만에 직진 접근이 '
                    '이어지는 정렬 해를 찾음.'
                )
            self._align_retry_count = 0
            self._start_open_gripper()
            return

        self._align_retry_count += 1
        if self._align_retry_count > MAX_ALIGN_RETRIES:
            self._align_retry_count = 0
            self._abort_to_return(
                f'[1/5 정렬] {MAX_ALIGN_RETRIES}회 재시도해도 직진 접근 dry-run이 '
                '계속 막힘(정렬 해가 매번 관절 한계/특이점 근처를 고름)'
            )
            return

        self.get_logger().warn(
            f'[1/5 정렬] 완료했지만 직진 접근 dry-run이 막혀 정렬 재시도 '
            f'({self._align_retry_count}/{MAX_ALIGN_RETRIES})'
        )
        self._move_arm(self._pending_approach_position, self._pending_quat)
        self._completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_align_complete
        )

    def _start_open_gripper(self) -> None:
        self._publish_grasp_step(GRASP_STEP_GRIPPER_OPEN)
        # [2026-07-27, 그리퍼 폭 동적화] 항상 GRIPPER_OPEN_POSITION(근사 최대
        # 개방)까지 열던 것을, 대상 추정 반지름에 맞춘 폭으로 대체함(위
        # GRIPPER_TARGET_RADIUS_*/_gripper_open_position_for_radius 설명 참고).
        # MoveIt2Gripper.open()은 생성자에 고정된 open_gripper_joint_positions만
        # 쓰므로, 대신 move_to_position()으로 매번 계산된 목표를 직접 지정.
        open_position = _gripper_open_position_for_radius(self._pending_target_radius_m)
        self.get_logger().info(
            f'[2/5 그리퍼 열기] 시작 (대상 추정 반지름 '
            f'{self._pending_target_radius_m * 1000:.1f}mm -> 그리퍼 목표 '
            f'{open_position:.3f}rad)'
        )
        self._gripper_moveit2.move_to_position(open_position)
        self._gripper_completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_open_gripper_complete
        )

    def _check_open_gripper_complete(self) -> None:
        if self._gripper_moveit2.query_state() != MoveIt2State.IDLE:
            return
        self._gripper_completion_timer.cancel()
        self._gripper_completion_timer = None

        if not self._gripper_moveit2.motion_suceeded:
            self._abort_to_return('[2/5 그리퍼 열기] 실패')
            return

        self.get_logger().info('[2/5 그리퍼 열기] 완료.')
        self._start_grasp_approach()

    def _start_grasp_approach(self) -> None:
        self._publish_grasp_step(GRASP_STEP_APPROACH)
        # [2026-07-28] 목표에 실제로 닿는 구간이라 감속(위 APPROACH_VELOCITY_
        # SCALING 설명 참고). look pose 복귀 완료 시점(_check_return_complete)
        # 에서 정상 속도로 원상복구됨 — 이 단계가 실패해도(abort) 그 복원
        # 로직이 그대로 적용되므로 별도 처리 불필요.
        self._moveit2.max_velocity = APPROACH_VELOCITY_SCALING
        self._moveit2.max_acceleration = APPROACH_ACCELERATION_SCALING
        target_position = self._pending_target_position
        self.get_logger().info(f'[3/5 직진 접근] 목표 지점으로 Cartesian 플래닝(감속 {APPROACH_VELOCITY_SCALING}): {target_position}')
        self._move_arm_cartesian(
            target_position,
            self._pending_quat,
            on_success=self._on_grasp_approach_execution_started,
            on_failure=functools.partial(
                self._abort_to_return, reason='[3/5 직진 접근] Cartesian 플래닝 실패'
            ),
        )

    def _on_grasp_approach_execution_started(self) -> None:
        self._completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_grasp_approach_complete
        )

    def _check_grasp_approach_complete(self) -> None:
        if self._moveit2.query_state() != MoveIt2State.IDLE:
            return
        self._completion_timer.cancel()
        self._completion_timer = None

        if not self._moveit2.motion_suceeded:
            self._abort_to_return('[3/5 직진 접근] 실패')
            return

        self.get_logger().info('[3/5 직진 접근] 완료.')
        self._start_close_gripper()

    def _start_close_gripper(self) -> None:
        self._publish_grasp_step(GRASP_STEP_GRASP)
        self.get_logger().info('[4/5 파지] 그리퍼 닫는 중...')
        self._gripper_moveit2.close()
        self._gripper_completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_close_gripper_complete
        )

    def _check_close_gripper_complete(self) -> None:
        if self._gripper_moveit2.query_state() != MoveIt2State.IDLE:
            return
        self._gripper_completion_timer.cancel()
        self._gripper_completion_timer = None

        if not self._gripper_moveit2.motion_suceeded:
            self._abort_to_return('[4/5 파지] 실패')
            return

        self.get_logger().info(
            f'[4/5 파지] 완료. {POST_GRASP_DWELL_SEC}초 대기 후 후퇴 시작(육안 확인용).'
        )
        self._post_grasp_dwell_timer = self.create_timer(
            POST_GRASP_DWELL_SEC, self._start_retreat
        )

    def _start_retreat(self) -> None:
        self._publish_grasp_step(GRASP_STEP_RETREAT)
        if self._post_grasp_dwell_timer is not None:
            self._post_grasp_dwell_timer.cancel()
            self._post_grasp_dwell_timer = None

        # [so101 교훈, 위 RETREAT_VELOCITY_SCALING 설명 참고] 그리퍼가 물체를
        # 쥔 채 움직이는 구간이라 일반 속도보다 낮춤. look pose 복귀까지
        # 이 속도 유지(_check_return_complete에서 원래 속도로 복원).
        self._moveit2.max_velocity = RETREAT_VELOCITY_SCALING
        self._moveit2.max_acceleration = RETREAT_ACCELERATION_SCALING

        approach_position = self._pending_approach_position
        self.get_logger().info(f'[5/5 후퇴] 정렬 위치로 Cartesian 복귀(감속 {RETREAT_VELOCITY_SCALING}): {approach_position}')
        self._move_arm_cartesian(
            approach_position,
            self._pending_quat,
            on_success=self._on_retreat_execution_started,
            on_failure=self._start_retreat_ompl_fallback,
        )

    def _start_retreat_ompl_fallback(self) -> None:
        """[2026-07-27] "직진 접근"(3/5)과 같은 이유로 후퇴(5/5)의 Cartesian
        경로도 막힐 수 있음이 실측으로 확인됨(mock 스택, ACM에 joint5/joint6를
        추가해 대부분의 "직진 접근" 실패는 해결했지만 일부 위치는 Cartesian
        직선 경로 자체가 특이점 근처에서 막힘 — 위 CARTESIAN_MAX_STEP_M 설명
        참고). 접근(3/5)은 아직 그리퍼가 비어있어 실패해도 그 자리에서 중단하고
        빈 그리퍼로 복귀하면 되지만, 후퇴(5/5)는 이미 물체를 쥔 상태라 그
        자리에서 바로 포기(abort)하는 것보다 OMPL 자유 경로로라도 정렬
        위치까지 물러나는 게 안전함(후퇴는 "정확한 직선"보다 "일단 안전하게
        빠져나오는 것"이 우선). OMPL도 실패하면 그때는 정말 물러날 방법이
        없는 것이므로 그대로 실패 처리(_check_retreat_complete가 처리).
        """
        self.get_logger().warn(
            '[5/5 후퇴] Cartesian 플래닝 실패 — OMPL 자유 경로로 재시도(그리퍼가 '
            '물체를 쥔 채라 안전하게 물러나는 것이 우선)'
        )
        self._move_arm(self._pending_approach_position, self._pending_quat)
        self._completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_retreat_complete
        )

    def _on_retreat_execution_started(self) -> None:
        self._completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_retreat_complete
        )

    def _check_retreat_complete(self) -> None:
        if self._moveit2.query_state() != MoveIt2State.IDLE:
            return
        self._completion_timer.cancel()
        self._completion_timer = None

        succeeded = self._moveit2.motion_suceeded
        if succeeded:
            self.get_logger().info('[5/5 후퇴] 완료. 5단계 전부 성공.')
        else:
            self.get_logger().warn('[5/5 후퇴] 실패.')

        # plan_result는 look pose 복귀까지 끝난 뒤에 발행함(아래 참고) —
        # 결과 값 자체는 5단계 전체(정렬/그리퍼 열기/직진 접근/파지/후퇴)가
        # 전부 성공했을 때만 True(이 지점에 도달했다는 것 자체가 이전 4단계가
        # 전부 성공했다는 뜻이므로, 여기선 후퇴 결과만 반영하면 됨).
        self._pending_result_succeeded = succeeded
        self._return_dwell_timer = self.create_timer(
            RETURN_TO_LOOK_POSE_DELAY_SEC, self._start_return_to_look_pose
        )

    def _start_return_to_look_pose(self) -> None:
        self._return_dwell_timer.cancel()
        self._return_dwell_timer = None

        self.get_logger().info('look pose로 복귀 중...')
        self._moveit2.move_to_configuration(
            LOOK_POSE_JOINT_POSITIONS, joint_names=JOINT_NAMES
        )
        self._return_completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_return_complete
        )

    def _check_return_complete(self) -> None:
        if self._moveit2.query_state() != MoveIt2State.IDLE:
            return

        self._return_completion_timer.cancel()
        self._return_completion_timer = None
        self._publish_grasp_step(GRASP_STEP_IDLE)

        if self._moveit2.motion_suceeded:
            self.get_logger().info('look pose 복귀 완료.')
        else:
            self.get_logger().warn('look pose 복귀 실패 — 팔 상태 수동 확인 필요.')

        # [2026-07-27] 후퇴(5/5) 시작 시 낮춘 속도(RETREAT_VELOCITY_SCALING)를
        # 다음 사이클(정렬)이 정상 속도로 시작하도록 원상복구. 정상 속도
        # 그대로였던 경로(파지 전 실패로 abort된 경우)에도 no-op이라 안전함.
        self._moveit2.max_velocity = VELOCITY_SCALING
        self._moveit2.max_acceleration = ACCELERATION_SCALING

        # harvest_sequence_node 등 여러 목표를 순차 발행하는 상위 로직이 이
        # 결과를 보고 다음 목표를 보낼 타이밍을 잡을 수 있도록 함(2026-07-24,
        # "수확 순차 처리" 로드맵). look pose 복귀까지 끝난 뒤에 발행해야
        # 다음 목표 검출 시점에 카메라가 이미 look pose 프레이밍으로 돌아와
        # 있음이 보장됨.
        result_msg = Bool()
        result_msg.data = bool(self._pending_result_succeeded)
        self._plan_result_publisher.publish(result_msg)

        self._busy = False


def main():
    rclpy.init()

    node = CoordToGoalNode()

    # 구독 콜백, MoveIt2 액션 응답, 완료 폴링 타이머가 동시에 처리될 수 있어야
    # 하므로 스레드 여러 개 필요.
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
