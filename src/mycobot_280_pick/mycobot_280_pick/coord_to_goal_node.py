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

동작: 그리퍼가 아직 없어서, 목표 지점보다 APPROACH_OFFSET_X 만큼 로봇 쪽(x축
음의 방향)으로 당긴 위치로 팔 끝(flange)이 접근하는 것으로 대체
(tomato_scene_test.py와 동일한 접근 방식).

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
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
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

# 목표 지점보다 이만큼 로봇 쪽(x축 음의 방향)으로 당겨서 접근.
# [2026-07-24 정정] 예전 주석엔 "TARGET_OBJECT_RADIUS보다 크면 접근 위치가 이
# 구와 안 겹침"이라고 돼 있었는데, 이는 flange 원점(점)만 고려한 착각이었음 —
# 그리퍼는 실제 부피가 있는 몸체(손가락이 flange에서 ~5.5cm 더 뻗어나감,
# "목표를 바라보는 orientation 동적 계산" 절 참고)라서, look-at이 정확히
# 목표를 겨냥하면 이 offset - TARGET_OBJECT_RADIUS(=3cm) 여유로는 손가락이
# 항상 구를 뚫고 들어감. compute_ik + check_state_validity로 직접 확인함
# (roll을 바꿔도 전부 동일하게 gripper_* <-> target_object 충돌,
# docs/obstacle_avoidance_manual_test.md 참고). 실제 해결은 아래
# GRIPPER_LINK_NAMES를 target_object와 충돌 허용(ACM)으로 등록하는 것 —
# 이 offset 값 자체는 순수 접근 거리로만 남음.
APPROACH_OFFSET_X = 0.08

# 목표 지점에 등록할 CollisionObject(구)의 반지름 (m). 실측 토마토 지름은
# 4cm(반지름 2cm)지만, 깊이 카메라가 측정하는 지점은 물체의 "카메라 쪽
# 표면"이라 노이즈 섞인 실제 point cloud가 그보다 카메라 쪽으로 튀어나온
# 경우가 있어(육안으로 확인함 — 구 앞쪽에 안 지워진 voxel이 남음), 여유를
# 넉넉히 둠.
TARGET_OBJECT_RADIUS = 0.05
TARGET_OBJECT_ID = 'target_object'

# [2026-07-24] target_object 구가 Octomap 자기 필터용으로만 쓰이길 원했는데,
# 그리퍼 자신의 접근 자세와도 충돌 판정되는 부작용이 있어(위 APPROACH_OFFSET_X
# 주석 참고), 그리퍼 링크들만 이 특정 구와 충돌 허용(ACM)으로 등록함(Octomap/
# 다른 world 장애물과의 충돌은 그대로 유지). URDF의 그리퍼 7개 링크와 동일
# (mycobot_280_m5_adaptive_gripper.urdf).
GRIPPER_LINK_NAMES = [
    'gripper_base',
    'gripper_left1',
    'gripper_left2',
    'gripper_left3',
    'gripper_right1',
    'gripper_right2',
    'gripper_right3',
]

# 구를 planning scene에 등록한 뒤, Octomap이 다음 point cloud로 자기 필터를
# 반영할 때까지 기다리는 시간 (sensors_3d.yaml의 max_update_rate=1.0Hz보다
# 살짝 길게 잡음).
OCTOMAP_CLEAR_DELAY_SEC = 1.2

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
VELOCITY_SCALING = 0.3
ACCELERATION_SCALING = 0.3

# [Tier1, so101-ros-physical-ai 교훈 이식] 목표 좌표를 TF 변환 후 검증 없이
# 바로 플래닝/실행하는 구조였음 — so101에서는 TF 조회 타이밍 문제로 팔이
# 베드와 무관한 방향(방위각 약 -98도)으로 크게 도는 사고가 있었음. mycobot은
# 같은 사고를 아직 겪지 않았지만 구조적으로 동일한 리스크(변환 결과를
# 그대로 신뢰)를 갖고 있어 최소 안전망으로 반지름/방위각 범위를 검증함.
# 아래 경계는 docs/obstacle_avoidance_manual_test.md에 기록된 정상 테스트
# 좌표들(g_base 기준 반지름 0.117~0.253m, 방위각 -50.2~+51.4도, 예:
# (0.219,0.054,0.311), (0.125,-0.15,0.204), (0.158,0.198,0.152))보다 훨씬
# 넉넉하게 잡되, 실제 사고성 좌표(2026-07-24 자동 루프 중 나온
# (-0.025,0.060,0.249) — 반지름 0.065m, 방위각 +112.6도, `obstacle_avoidance_
# manual_test.md:752` 참고)는 확실히 걸러지도록 함.
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

# ---- 그리퍼 임시 충돌 형상 (URDF에 아직 그리퍼가 없어서, 실제 그리퍼가
# 카메라 바로 앞(~13~21cm)에서 자기 몸 필터 없이 그대로 장애물로 잡히는 문제
# 우회용) ----
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

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # move_to_pose() 실행 중 새 목표가 들어오면 무시하기 위한 플래그.
        self._busy = False
        self._completion_timer = None
        self._clear_timer = None
        self._pending_approach_position = None
        self._pending_target_position = None
        self._return_dwell_timer = None
        self._return_completion_timer = None
        self._pending_result_succeeded = None
        self._look_pose_completion_timer = None

        # 목표 지점에 CollisionObject(구)를 등록해 Octomap이 그 자리 실제
        # 물체(토마토)를 자기 필터로 걸러내도록 함 (위 모듈 docstring 참고).
        self._planning_scene_publisher = self.create_publisher(
            PlanningScene, 'planning_scene', 10
        )

        # 목표 하나의 플래닝/실행 결과(성공/실패)를 발행 — harvest_sequence_node처럼
        # 여러 목표를 순차 발행하는 상위 로직이 다음 목표를 언제 보낼지 판단하는
        # 데 씀(위 _check_motion_complete 참고).
        self._plan_result_publisher = self.create_publisher(Bool, 'plan_result', 10)

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

        self.get_logger().info(
            'coord_to_goal_node 준비 완료. /target_point 구독 대기 중...'
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

        for timer_attr in (
            '_completion_timer',
            '_return_completion_timer',
            '_return_dwell_timer',
            '_clear_timer',
            '_look_pose_completion_timer',
        ):
            timer = getattr(self, timer_attr)
            if timer is not None:
                timer.cancel()
                setattr(self, timer_attr, None)

        self._busy = False
        self.get_logger().warn(
            'emergency_stop 요청 수신 — 현재 궤적 실행 취소함(소프트 정지)'
        )
        response.success = True
        response.message = (
            '궤적 실행 취소함(소프트 정지) — 하드웨어 토크 차단이 아님, '
            '실물 토크까지 끊으려면 RPi에서 release_all_servos() 직접 호출 필요'
        )
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

        approach_position = [target.x - APPROACH_OFFSET_X, target.y, target.z]

        self._busy = True
        self._publish_target_collision_object(target)
        self._pending_approach_position = approach_position
        self._pending_target_position = [target.x, target.y, target.z]
        self.get_logger().info(
            f'목표 지점({target.x:.3f}, {target.y:.3f}, {target.z:.3f})에 '
            f'Octomap 클리어용 구 등록, {OCTOMAP_CLEAR_DELAY_SEC}초 대기 후 플래닝 시작'
        )
        # Octomap이 다음 point cloud로 자기 필터 갱신을 반영할 때까지 기다린
        # 뒤에 플래닝을 시작함 (바로 플래닝하면 아직 예전 voxel이 남아있어서
        # 여전히 충돌로 판정될 수 있음).
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

        entry_names = list(current_acm.entry_names)
        entry_rows = [list(entry.enabled) for entry in current_acm.entry_values]

        if TARGET_OBJECT_ID not in entry_names:
            for row in entry_rows:
                row.append(False)
            for i, name in enumerate(entry_names):
                entry_rows[i][-1] = name in GRIPPER_LINK_NAMES

            new_row = [name in GRIPPER_LINK_NAMES for name in entry_names]
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
            f'target_object <-> 그리퍼 링크({len(GRIPPER_LINK_NAMES)}개) 충돌 '
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

        self._roll_search_forward = forward
        self._roll_search_index = 0
        self._roll_search_retry = 0
        self._try_next_roll_candidate()

    def _try_next_roll_candidate(self) -> None:
        """ROLL_CANDIDATES_RAD를 순서대로 compute_ik로 사전 체크해, 실제 IK가
        풀리는 첫 후보를 채택함(위 ROLL_CANDIDATES_RAD 설명 참고). 후보마다
        ROLL_CANDIDATE_IK_RETRIES번 재시도함(IK가 확률적이라 한 번만 시도하면
        실제로 유효한 후보를 놓칠 수 있음). 전체 후보가 다 실패하면 0번 후보
        (기존 WORLD_UP 단일값과 동일)로 진행 — compute_ik는 사전 필터일 뿐이라
        실제 move_to_pose의 OMPL 플래닝(더 넉넉한 시간/재시도)이 최종 판단임.
        """
        if self._roll_search_index >= len(ROLL_CANDIDATES_RAD):
            fallback_quat = _compute_look_at_quat_xyzw(
                self._roll_search_forward, roll_rad=ROLL_CANDIDATES_RAD[0]
            )
            self._finalize_planning(fallback_quat)
            return

        roll_rad = ROLL_CANDIDATES_RAD[self._roll_search_index]
        quat = _compute_look_at_quat_xyzw(self._roll_search_forward, roll_rad=roll_rad)

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
        position = self._pending_approach_position
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
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - IK 서비스 호출 실패는 재시도로 넘어감
            self.get_logger().warn(f'compute_ik 호출 실패, 재시도: {exc}')
            response = None

        if response is not None and response.error_code.val == MoveItErrorCodes.SUCCESS:
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

    def _finalize_planning(self, approach_quat) -> None:
        approach_position = self._pending_approach_position

        self.get_logger().info(
            f'목표 위치로 플래닝: {approach_position}, orientation(xyzw): '
            f'{[round(c, 3) for c in approach_quat]}'
        )
        # tolerance 기본값(0.001/0.001)은 IK가 이 정확한 위치+방향을 동시에
        # 만족하는 해를 못 찾는 경우가 많아(OMPL이 짧은 시간 안에 못 찾음),
        # orientation은 넉넉히 풀어줌 — 위치는 정확히, 방향은 근사치면 충분한
        # 접근(pregrasp) 자세이므로 문제 없음(이번 세션 테스트로 확인).
        self._moveit2.move_to_pose(
            position=approach_position,
            quat_xyzw=approach_quat,
            cartesian=False,
            tolerance_position=0.005,
            tolerance_orientation=0.5,
        )
        # wait_until_executed()는 내부적으로 rclpy.spin_once()를 호출해서 이미
        # 돌고 있는 MultiThreadedExecutor와 충돌하므로 쓰지 않음. 대신 같은
        # executor가 처리하는 타이머로 완료 여부만 폴링함.
        self._completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_motion_complete
        )

    def _check_motion_complete(self) -> None:
        if self._moveit2.query_state() != MoveIt2State.IDLE:
            return

        self._completion_timer.cancel()
        self._completion_timer = None

        succeeded = self._moveit2.motion_suceeded
        if succeeded:
            self.get_logger().info('플래닝/실행 완료.')
        else:
            self.get_logger().warn('플래닝/실행 실패.')

        # plan_result는 look pose 복귀까지 끝난 뒤에 발행함(아래 참고) —
        # 결과 값 자체는 이 접근 동작의 성공/실패를 그대로 반영.
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

        if self._moveit2.motion_suceeded:
            self.get_logger().info('look pose 복귀 완료.')
        else:
            self.get_logger().warn('look pose 복귀 실패 — 팔 상태 수동 확인 필요.')

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
