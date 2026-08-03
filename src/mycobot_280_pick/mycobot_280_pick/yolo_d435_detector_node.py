#!/usr/bin/env python3
"""
D435 RGB-D(ROS2 realsense2_camera 토픽 구독) -> YOLO 토마토 검출 -> 3D 좌표 계산
-> /target_point 발행 노드.

이전 버전은 pyrealsense2로 카메라를 직접 열었는데(rs.pipeline()), Octomap용
realsense2_camera_node가 이미 D435를 물고 있으면 장치 충돌이 남(D435는 한
프로세스만 열 수 있음). 그래서 demo_octomap.launch.py가 띄우는 ROS2 토픽
(image_raw/aligned_depth_to_color/camera_info)을 그대로 구독하도록 변경 —
Octomap과 YOLO 검출이 카메라 하나를 공유해서 동시에 동작함.

[게이팅+누적판단 재설계, so101-ros-physical-ai 자매 프로젝트 교훈 이식]
기존엔 매 프레임(최대 PUBLISH_RATE_HZ) "판단"(target_point/tomato_candidates)을
그대로 재발행했는데, 이 노드가 `coord_to_goal_node`와 함께 떠 있으면
"목표 도달 -> 자동 look pose 복귀 -> 재검출 -> 다시 자동 이동"이 무한 반복되는
사고를 실제로 겪었음(2026-07-24, docs/obstacle_avoidance_manual_test.md
"⚠️ 사고: look pose 자동 복귀 + 실물 자동 루프" 참고, 그 문서 자체가 해법으로
"look pose 세션 중에만 target_point를 받도록 게이팅"을 제안해뒀었음). so101이
동일한 문제를 해결하며 검증한 설계를 그대로 이식함:
  - (2026-07-24 최초 설계) 시각화(tomato_boxes/tomato_detections_image)는
    look pose 여부와 무관하게 항상 매 프레임 발행 — "검출 자체가 안 됐는지"
    vs "판단 로직이 막았는지"를 육안으로 구분할 수 있어야 하므로.
  - [2026-07-27 실물 조정, 되돌림] 위 "항상 추론"이 실물 세션에서 CPU를
    상시 크게 잡아먹어(다른 노드/카메라 드라이버와 경합) 시스템 부하가
    쌓이고 카메라 스트림 자체가 불안정해지는 악순환의 주요 원인 중 하나로
    확인됨 — 추론 자체를 look pose 누적 구간에서만 돌리도록 되돌림(트레이드
    오프: 평소엔 tomato_boxes/detections_image가 안 갱신됨, "검출 안 됨" vs
    "판단 로직이 막음" 육안 구분 능력을 CPU 부하와 맞바꾼 것 — 사용자 확인
    후 적용).
  - [2026-07-28, 다시 원복] 위 진단이 틀렸음이 밝혀짐 — 진짜 CPU 주범은
    YOLO 추론이 아니라 Octomap 파이프라인(realsense `pointcloud.enable`이
    켜져 있던 것 + `pointcloud_tomato_filter_node`, 둘 다 단독으로 CPU
    100%+ 관측)이었음. so101-ros-physical-ai 자매 프로젝트는 같은 D435로
    매 프레임 실시간 추론을 해도 부하가 훨씬 낮은데, 그 프로젝트엔 애초에
    Octomap/pointcloud 파이프라인이 없다는 게 유일한 구조적 차이였음
    (demo_octomap.launch.py의 `pointcloud.enable` 삭제와 짝을 이루는
    변경). 그래서 시각화는 다시 매 프레임 실시간으로 되돌림 — Octomap을
    같이 켜는 세션(위 [2026-07-27] 문단이 우려했던 상황)에서 다시 부하
    문제가 재현되면 이 게이팅을 재적용할 것.
  - "판단"(target_point/tomato_candidates)은 팔이 look pose
    (LOOK_POSE_JOINT_POSITIONS, /joint_states로 확인)에 있을 때만
    ACCUMULATION_WINDOW_SEC초 동안 프레임을 누적 -> 3D 위치로 클러스터링 ->
    클러스터당 1회만 발행 -> look pose를 벗어났다 다시 올 때까지 이후
    프레임은 전부 무시(judgment_locked). 이 변경만으로 target_point가
    "look pose 방문당 최대 1회"로 제한되어, `coord_to_goal_node` 쪽의 `_busy`
    플래그와 무관하게 연쇄 자동 이동 루프 자체가 구조적으로 불가능해짐.

[Tier3, depth 반지름 보정, so101 교훈 이식] bbox 중심 픽셀의 depth는 물체의
"카메라 쪽 표면"이지 "중심"이 아니라서, 그대로 좌표로 쓰면 실제 물체 중심보다
반지름만큼 짧게(카메라 쪽으로) 계산됨. bbox 픽셀 크기를 핀홀 역산해 반지름을
추정하고 표면 depth에 더해 중심 depth로 보정함. 주의: so101에서도 이 보정만
으로는 실물 파지 시 3~4cm 부족 문제가 완전히 해소되지 않았음이 확인됨 —
"체계적 과소평가를 완화"하는 정도로 기대할 것, 완전한 해결책은 아님.

여러 토마토가 동시에 검출되면 'ripe'(익음) 클래스 중 (관측 횟수 최다 ->
평균 confidence) 1개만 수확 대상으로 선택해 발행함 (익은 토마토만 수확
대상이므로, 관측 횟수 우선 선택은 confidence 최댓값 단발보다 안정적인
클러스터를 고르기 위함 — so101에서 검증된 기준).

발행: /target_point (geometry_msgs/msg/PointStamped), look pose 누적 구간
  종료 시 최대 1회 발행.
  - header.frame_id: camera_info의 frame_id를 그대로 씀 (보통
    "camera_color_optical_frame" — align_depth로 depth를 color 프레임에
    맞췄으므로 두 이미지 다 이 프레임 기준).
  - point: 위 프레임 기준 3D 좌표 (m 단위, depth 반지름 보정 반영됨)

발행: target_radius_m (std_msgs/msg/Float32) — [2026-07-27, 그리퍼 폭
  동적화] target_point가 가리키는 대상의 클러스터 평균 추정 반지름
  (MIN/MAX_ESTIMATED_RADIUS_M로 clamp된 값, 위 _estimate_radius_m 참고).
  target_point보다 먼저 발행함 — coord_to_goal_node가 target_point 콜백
  시점에 이미 이 값을 받아둔 상태이도록 순서를 맞춘 것(별개 토픽이라 완벽한
  원자성 보장은 아니지만, 그리퍼 폭 참고용 정보라 약간의 지연은 무해함).
  coord_to_goal_node가 이 값으로 그리퍼 열림 폭을 대상 크기에 맞게 조정함.

발행: tomato_boxes (std_msgs/msg/Float32MultiArray) — 이번 추론에서 검출된
"모든" bbox(클래스/ripe 여부 무관, [x1,y1,x2,y2, x1,y1,x2,y2, ...] 평탄화된
컬러 이미지 픽셀 좌표)를 look pose 여부와 무관하게 매 프레임 발행함
(추론을 두 번 돌리지 않음). pointcloud_tomato_filter_node가 이 bbox로
raw pointcloud에서 토마토 영역 포인트를 제거해 Octomap이 토마토 자체를
장애물로 잡지 않게 함(로드맵 2단계, docs/obstacle_avoidance_manual_test.md
참고). 이 모델은 토마토 상태 4클래스만 검출하도록 학습됐으므로 클래스
구분 없이 검출된 박스 전부가 "토마토"로 간주해도 됨.

발행: tomato_candidates (std_msgs/msg/Float32MultiArray) — HARVEST_CLASS_NAMES
  (ripe/disease) 클래스의 look pose 누적 구간 클러스터링 결과를
  [class_id, x, y, z, confidence, ...]로 평탄화해(카메라 프레임 기준 3D
  좌표, 위 target_point와 같은 핀홀 변환식 + depth 보정) 발행함.
  harvest_sequence_node가 이 후보 목록을 z(깊이) 오름차순으로 정렬해 순차
  접근하는 데 씀(로드맵, docs/obstacle_avoidance_manual_test.md "수확 순차
  처리" 절 참고). look pose를 벗어나면 stale 데이터 방지를 위해 빈 배열을
  발행함.

발행: tomato_detections_image (sensor_msgs/msg/Image) — 이번 추론 결과에
bbox/클래스명/confidence를 그려 넣은 컬러 이미지(ultralytics
`Results.plot()`). look pose 여부와 무관하게 매 프레임 발행 — RViz에 Image
디스플레이로 추가해서 YOLO가 실제로 무엇을 어떤 클래스로 검출했는지(또는
아예 못 했는지) 육안 디버깅하는 용도.

캘리브레이션(로드맵 3단계, docs 13장)이 이미 끝나 있어서 이 frame_id ->
g_base로 가는 TF가 실시간으로 존재함 — coord_to_goal_node가 코드 수정 없이
그대로 TF2 변환해서 실제 로봇을 움직임.

픽셀->3D 변환은 camera_info의 핀홀 카메라 내부 파라미터(K)로 직접 계산함
(rs2_deproject_pixel_to_point 대신 — 왜곡 계수는 D435 컬러 스트림에서
무시할 만큼 작아서 근사해도 충분함).

실행 시 주의: pip으로 설치된 opencv-python(~/.local, ultralytics 의존성으로
같이 깔림)이 시스템 opencv(cv_bridge가 링크하는 버전)보다 import 우선순위가
높아서 cv_bridge가 깨짐 (docs 13장, easy_handeye2 캘리브레이션 때 겪은 것과
같은 원인). 그런데 PYTHONNOUSERSITE=1로 전체 사용자 site-packages를 빼버리면
이번엔 ultralytics 자체를 못 찾음 (ultralytics는 사용자 site에만 있음) —
그래서 여기서는 통째로 빼는 대신, 시스템 dist-packages만 사용자 site보다
먼저 검색하도록 sys.path 순서를 조정함 (아래). cv2는 시스템 버전을 먼저
찾고, ultralytics는 시스템에 없으니 그대로 사용자 site에서 찾음.

설치 (pip 패키지라 package.xml/rosdep 대상이 아님, 전역 환경에 직접 설치):
  pip install ultralytics --break-system-packages
"""

import math
import os
import sys

import cv2
import numpy as np

# cv_bridge(시스템 opencv 필요)와 ultralytics(사용자 site-packages에만 있음)를
# 이 프로세스 하나에서 동시에 써야 해서, PYTHONNOUSERSITE로 통째로 빼는 대신
# 시스템 dist-packages를 사용자 site-packages보다 먼저 검색하도록만 재정렬함.
sys.path = (
    [p for p in sys.path if 'dist-packages' in p]
    + [p for p in sys.path if 'dist-packages' not in p]
)

from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float32, Float32MultiArray
from std_srvs.srv import SetBool
from ultralytics import YOLO

# [2026-07-27] 이전 세션엔 실물 검출이 전부 'rotten'으로 잡혀서 임시로
# 'rotten'을 타겟으로 바꿔뒀었는데, 오늘 재확인해보니 'ripe'로 정상 검출됨
# (모델/조명/각도 등 조건이 달라진 것으로 추정) — 원래 의도한 'ripe'로 복귀.
# [2026-07-27 후속] 이 오분류의 진짜 원인이 밝혀짐 — 아래 COLOR_OVERRIDE
# 설명 참고. 3D 프린트 소품(실제 토마토 아님) 특유의 단색이라 모델(v6)이
# 색 계열을 자주 틀리는 것으로, pick8.py(다른 파이프라인, 같은 v6 모델
# 사용)에서 이미 원인 규명 + 색상 기반 보정으로 해결한 전례를 그대로 포팅함.
TARGET_CLASS_NAME = 'ripe'
# TARGET_CLASS_NAME = 'rotten'
CONFIDENCE_THRESHOLD = 0.4

# [2026-07-27, pick8.py에서 포팅] 3D 프린트 방울토마토 색 보정.
# 문제: v6 모델은 박스 위치는 잘 잡는데 색 계열 분류를 틀린다 —
# 초록 3D 소품 → ripe로, 노랑 3D 소품 → unripe로 자주 오분류함(pick8.py에서
# 실측 확인된 것과 동일 증상, 이 노드에서 'rotten'으로 몰린 것도 같은 부류의
# 오분류로 추정). 우리가 쓰는 3D 프린트 토마토는 색이 빨강/초록/노랑
# 3가지뿐이라, 박스 안의 지배적 HSV 색상으로 클래스를 덮어씀:
#     빨강 = ripe / 초록 = unripe / 노랑 = disease
# ⚠️ 이건 '3D 프린트 소품' 전용 규칙이다. 실제 토마토는 익어가는 중간색이
# 있어 이 규칙이 틀림 — 실물 토마토로 갈 땐 COLOR_OVERRIDE_ENABLED=False로
# 끄고, 3D 소품이 아닌 실제 토마토로 재학습한 모델을 쓸 것(pick8.py의
# 같은 경고 그대로 적용됨).
COLOR_OVERRIDE_ENABLED = True
COLOR_MIN_RATIO = 0.25

# [2026-07-24, 수확 순차 처리] 모델 4클래스({ripe, unripe, rotten, disease}) 중
# 실제 수확/제거 대상. harvest_sequence_node가 이 후보 목록을 받아 look pose
# 스냅샷 기준으로 깊이(z) 오름차순 정렬해 순차 접근함(docs/
# obstacle_avoidance_manual_test.md "수확 순차 처리" 절 참고).
# 주의: TARGET_CLASS_NAME이 여기 없는 클래스면 _accumulate_detections()에서
# 애초에 걸러져 누적 버퍼에 들어가지도 않음 — TARGET_CLASS_NAME을 바꿀 땐
# 이 목록에도 포함돼 있는지 항상 같이 확인할 것.
HARVEST_CLASS_NAMES = ('ripe', 'rotten', 'disease')

# [Tier2] coord_to_goal_node.py의 JOINT_NAMES/LOOK_POSE_JOINT_POSITIONS와
# 반드시 동일해야 함(원본 확정값: docs/look_pose.md). 이 노드가
# coord_to_goal_node 없이도 독립적으로 뜰 수 있어야 해서(그 반대의 무거운
# import 의존을 피하려고) 상수를 복제함 — 두 값이 갈리면 look pose 게이팅이
# 어긋나므로 look pose 값을 바꿀 땐 두 파일 모두 갱신할 것.
JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]
LOOK_POSE_JOINT_POSITIONS = [
    -0.130376,
    1.816190,
    -0.989253,
    -0.872665,
    0.279078,
    0.024435,
]
# look pose 복귀 시 실물 오차(정지 오차/컨트롤러 tolerance)를 감안한 허용치.
LOOK_POSE_TOLERANCE_RAD = 0.08

# [Tier2] look pose 도착 후 판단을 위해 프레임을 누적하는 시간(so101 검증값).
# [2026-07-27 실물 조정] 카메라 케이블 지터로 color/depth 동기화 콜백 자체가
# 드물게만 들어와서(아래 ApproximateTimeSynchronizer 참고) 2초 안에 한 번도
# 못 잡는 경우가 실물에서 반복 확인됨 — 잡을 기회를 늘리기 위해 늘림.
ACCUMULATION_WINDOW_SEC = 5.0
# [Tier2] 3D 위치 기준 클러스터링 거리 임계값 — 이 이내면 같은 대상으로 취급
# (같은 클래스인 경우만, so101 검증값).
CLUSTER_DISTANCE_M = 0.03

# bbox 크기로 추정한 물체 반지름의 상하한 clamp(m) — 실측 토마토 두 종류
# (지름 25mm/36mm)의 반지름과 동일. depth 표면→중심 보정(아래
# _estimate_radius_m)과 coord_to_goal_node의 그리퍼 개방폭 결정(target_
# radius_m) 양쪽에 다 쓰이므로, coord_to_goal_node.py의 GRIPPER_TARGET_
# RADIUS_MIN/MAX_M과 반드시 같은 값으로 유지할 것(두 노드가 독립 실행
# 파일이라 값을 import로 공유하지 않음).
MIN_ESTIMATED_RADIUS_M = 0.0125  # 25mm 지름 토마토의 반지름
MAX_ESTIMATED_RADIUS_M = 0.018   # 36mm 지름 토마토의 반지름

# 반지름 보정(표면→중심) 이후 최종 depth에서 추가로 빼는 고정 여유(m) —
# g_base 기준 X(팔이 뻗는 깊이)가 그만큼 줄어듦. [2026-07-28] 0으로 변경 —
# 그리퍼를 flange 목표에 그대로 보내 실측한 결과, flange가 실제 토마토
# 표면보다 항상 정확히 이 값(당시 0.05)만큼 못 미쳤음이 서로 다른 두 목표
# 좌표에서 재현 확인됨(RViz+TF 실측, 그리퍼 길이 보정과는 무관하게 flange
# 자체 도달 거리에서 나타난 오차). 이 상수가 도입됐던 3~5차 조정 당시엔
# bbox 중심 픽셀 depth hole 버그(아래 _robust_bbox_depth_m)가 아직
# 안 고쳐진 상태라 그 노이즈까지 뭉뚱그려 보정하려던 것이었는데, 그 버그가
# 고쳐진 지금은 이 보정 자체가 정확히 실측 오차만큼 과보정하고 있었던
# 것으로 확인됨 — 값을 0으로 되돌림. 다시 부족/과함이 보이면 실측 기반으로
# 재조정할 것.
DEPTH_SAFETY_MARGIN_M = 0.0

DEFAULT_MODEL_PATH = os.path.expanduser(
    # '~/Projects/Eval_Yolo/tomato_4cls_model.pt'
    '~/Projects/Eval_Yolo/tomato_4cls_v6.pt'
)
DEFAULT_COLOR_TOPIC = '/camera/camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/camera/aligned_depth_to_color/camera_info'


def _classify_by_color(bgr, x1, y1, x2, y2):
    """[2026-07-27, pick8.py 포팅] 박스 안 지배적 HSV 색상 -> 클래스 이름.
    판단 불가면 None(=YOLO 원래 클래스 유지). 위 COLOR_OVERRIDE_ENABLED
    설명 참고 — 3D 프린트 소품 전용 색상표(빨강/초록/노랑 3색)에 맞춘
    Hue 범위이며, pick8.py에서 실측으로 확정한 값을 그대로 사용함.
    """
    h0, w0 = bgr.shape[:2]
    # 테두리·배경 섞임을 줄이려 안쪽 60%만 본다.
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    bw, bh = (x2 - x1) * 0.3, (y2 - y1) * 0.3
    a = max(0, int(cx - bw))
    b = min(w0, int(cx + bw))
    c = max(0, int(cy - bh))
    d = min(h0, int(cy + bh))
    if b - a < 3 or d - c < 3:
        return None
    hsv = cv2.cvtColor(bgr[c:d, a:b], cv2.COLOR_BGR2HSV)
    h_ch, s_ch, v_ch = hsv[:, :, 0].astype(int), hsv[:, :, 1].astype(int), hsv[:, :, 2].astype(int)
    ok = (s_ch >= 90) & (v_ch >= 60)  # 채도·명도 낮은 화소(그림자·배경)는 버림
    n = int(ok.sum())
    if n < 20:
        return None
    hue = h_ch[ok]
    counts = {
        'ripe': int(((hue <= 10) | (hue >= 170)).sum()),  # 빨강
        'disease': int(((hue >= 20) & (hue <= 35)).sum()),  # 노랑
        'unripe': int(((hue >= 40) & (hue <= 90)).sum()),  # 초록
    }
    best = max(counts, key=counts.get)
    return best if counts[best] / n >= COLOR_MIN_RATIO else None


# [2026-08-03] 사본이 셋이 되려던 참에 공용 모듈(look_pose.py)을 만들었다.
# 이 파일의 값은 그대로 두되, **기동 시 어긋났는지 검사한다** — 한쪽만 고치면
# 에러 없이 게이트만 조용히 안 열리는 종류의 버그라 눈으로는 못 찾는다.
try:
    from mycobot_280_pick import look_pose as _look_pose_shared
    _look_pose_shared.assert_matches(JOINT_NAMES, LOOK_POSE_JOINT_POSITIONS,
                                     source='yolo_d435_detector_node')
except ImportError:
    pass


def _is_near_look_pose(positions_by_name: dict) -> bool:
    for name, look_value in zip(JOINT_NAMES, LOOK_POSE_JOINT_POSITIONS):
        current_value = positions_by_name.get(name)
        if current_value is None or abs(current_value - look_value) > LOOK_POSE_TOLERANCE_RAD:
            return False
    return True


def _robust_bbox_depth_m(depth_image, x1, y1, x2, y2):
    """[2026-07-27, 실물에서 발견] bbox 중심 픽셀 딱 한 점만 보면 D435 depth
    hole(반사/각도로 특정 픽셀만 무효(0)인 경우 — 컬러 검출은 성공하는데
    정확히 그 지점만 depth=0이라 매번 조용히 스킵되는 것을 실물 캡처로 직접
    재현/확인함)에 취약함. bbox 영역 전체에서 유효한(0이 아닌) depth 값들의
    중앙값을 사용 — bbox가 물체에 타이트하게 잡히는 게 보통이라 배경이 섞여
    들어와도 중앙값이면 견고함. 유효 픽셀이 하나도 없으면 None.
    """
    height, width = depth_image.shape[:2]
    xi1 = min(max(int(x1), 0), width - 1)
    yi1 = min(max(int(y1), 0), height - 1)
    xi2 = min(max(int(x2), 0), width - 1)
    yi2 = min(max(int(y2), 0), height - 1)
    region = depth_image[yi1:yi2 + 1, xi1:xi2 + 1]
    valid = region[region > 0]
    if valid.size == 0:
        return None
    return float(np.median(valid)) / 1000.0


def _estimate_radius_m(x1, y1, x2, y2, depth_m, fx, fy):
    """bbox 픽셀 크기 + 핀홀 투영 역산으로 물체(토마토) 반지름을 추정.
    표면 depth를 중심 depth로 보정하는 데 씀(모듈 docstring "Tier3" 설명 참고).
    """
    width_px = max(x2 - x1, 1e-6)
    height_px = max(y2 - y1, 1e-6)
    diameter_m = ((width_px * depth_m / fx) + (height_px * depth_m / fy)) / 2.0
    radius_m = diameter_m / 2.0
    return min(max(radius_m, MIN_ESTIMATED_RADIUS_M), MAX_ESTIMATED_RADIUS_M)


def _cluster_detections(detections):
    """[Tier2] 누적된 검출(class_id, x, y, z, confidence, radius_m) 리스트를
    3D 위치 기준으로 클러스터링함(같은 class_id + CLUSTER_DISTANCE_M 이내
    거리면 같은 대상으로 취급, 평균 위치/신뢰도/반지름으로 병합). so101이
    실물로 검증한 방식과 동일(반지름 평균은 [2026-07-27, 그리퍼 폭 동적화]
    추가분).
    반환: [{'class_id','x','y','z','confidence','radius_m','count'}, ...]
    """
    clusters = []
    for class_id, x, y, z, confidence, radius_m in detections:
        matched = None
        for cluster in clusters:
            if cluster['class_id'] != class_id:
                continue
            count = cluster['count']
            mean_x = cluster['sum_x'] / count
            mean_y = cluster['sum_y'] / count
            mean_z = cluster['sum_z'] / count
            distance = math.sqrt(
                (x - mean_x) ** 2 + (y - mean_y) ** 2 + (z - mean_z) ** 2
            )
            if distance <= CLUSTER_DISTANCE_M:
                matched = cluster
                break

        if matched is None:
            clusters.append({
                'class_id': class_id,
                'sum_x': x,
                'sum_y': y,
                'sum_z': z,
                'sum_conf': confidence,
                'sum_radius': radius_m,
                'count': 1,
            })
        else:
            matched['sum_x'] += x
            matched['sum_y'] += y
            matched['sum_z'] += z
            matched['sum_conf'] += confidence
            matched['sum_radius'] += radius_m
            matched['count'] += 1

    return [
        {
            'class_id': cluster['class_id'],
            'x': cluster['sum_x'] / cluster['count'],
            'y': cluster['sum_y'] / cluster['count'],
            'z': cluster['sum_z'] / cluster['count'],
            'confidence': cluster['sum_conf'] / cluster['count'],
            'radius_m': cluster['sum_radius'] / cluster['count'],
            'count': cluster['count'],
        }
        for cluster in clusters
    ]


class YoloD435DetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_d435_detector_node')

        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        self.declare_parameter('color_topic', DEFAULT_COLOR_TOPIC)
        self.declare_parameter('depth_topic', DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('camera_info_topic', DEFAULT_CAMERA_INFO_TOPIC)
        # [2026-08-02] 이 노드가 **직접 팔을 움직이게 할 것인가.**
        #
        # false(기본)면 target_point/target_radius_m를 발행하지 않고
        # tomato_candidates만 낸다 — 즉 파지 명령은 오직 harvest_sequence_node를
        # 거친다. 이 노드가 목표를 직접 쏘면 "도달 -> look pose 복귀 -> 재검출 ->
        # 다시 이동"이 무한 반복되는 사고가 난다(2026-07-24 실물에서 실제로 겪음,
        # 위 모듈 docstring 참고). 누적판단 게이팅으로 "look pose 방문당 1회"까지는
        # 줄였지만, **방문 자체가 사이클마다 일어난다**:
        #   LPC  목표마다 look pose로 복귀 -> 매 수확마다 게이트가 열린다
        #   ASC/BSC  중간엔 안 열리지만 시퀀스 종료 후 /go_to_look_pose에서 열린다
        # 그래서 "1회로 줄이는 것"이 아니라 **발행 자체를 끄는 것**이 기본이다.
        #
        # true로 두면 예전 동작(단발 수동 테스트용) — 시퀀스 노드 없이 토마토
        # 하나를 바로 따 보게 된다. 실물에서는 위 사고 경로가 다시 열린다.
        self.declare_parameter('publish_target_point', False)

        model_path = (
            self.get_parameter('model_path').get_parameter_value().string_value
        )
        color_topic = (
            self.get_parameter('color_topic').get_parameter_value().string_value
        )
        depth_topic = (
            self.get_parameter('depth_topic').get_parameter_value().string_value
        )
        camera_info_topic = (
            self.get_parameter('camera_info_topic')
            .get_parameter_value()
            .string_value
        )

        self.get_logger().info(f'YOLO 모델 로드 중: {model_path}')
        self._model = YOLO(model_path)
        # [2026-07-27, COLOR_OVERRIDE용] 클래스 이름 -> id 역방향 조회.
        self._class_name_to_id = {name: idx for idx, name in self._model.names.items()}
        self._bridge = CvBridge()
        self._intrinsics = None  # (fx, fy, cx, cy), camera_info 수신 시 채워짐
        self._frame_id = None

        self._publish_target_point = (
            self.get_parameter('publish_target_point')
            .get_parameter_value()
            .bool_value
        )

        # [Tier2] look pose 게이팅 + 누적판단 상태.
        self._at_look_pose = False
        self._judgment_locked = False
        self._accumulated_detections = []  # [(class_id, x, y, z, confidence), ...]
        self._accumulation_timer = None
        # [2026-08-02] 판단 전체를 외부에서 얼릴 수 있게 한다(set_judgment_enabled).
        # 수확 시퀀스가 도는 동안 harvest_sequence_node가 이걸 내린다 — 시퀀스는
        # 시작 시점 스냅샷 하나로 끝까지 가므로(그 파일 docstring), 중간에 나오는
        # 새 판단은 큐를 못 바꾸면서 팔만 흔들 수 있다. 특히 LPC는 목표마다
        # look pose로 돌아가 판단이 매번 열린다.
        self._judgment_enabled = True

        self._publisher = self.create_publisher(PointStamped, 'target_point', 10)
        # [2026-07-27, 그리퍼 폭 동적화] target_point와 함께 발행되는 대상
        # 추정 반지름 — coord_to_goal_node가 그리퍼 열림 폭을 정하는 데 씀.
        self._target_radius_publisher = self.create_publisher(
            Float32, 'target_radius_m', 10
        )
        self._boxes_publisher = self.create_publisher(
            Float32MultiArray, 'tomato_boxes', 10
        )
        self._candidates_publisher = self.create_publisher(
            Float32MultiArray, 'tomato_candidates', 10
        )
        self._annotated_image_publisher = self.create_publisher(
            Image, 'tomato_detections_image', 10
        )

        # [2026-08-02] 수확 시퀀스가 도는 동안 판단을 얼리는 스위치.
        # harvest_sequence_node가 시작 시 false, 종료 시 true로 부른다.
        self._judgment_service = self.create_service(
            SetBool, '~/set_judgment_enabled', self._on_set_judgment_enabled
        )

        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, 10
        )
        self._joint_states_sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_states, 10
        )

        color_sub = message_filters.Subscriber(self, Image, color_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        # [2026-07-27 실물 조정] 카메라 케이블이 마진널해서 color/depth 스트림에
        # 순간적으로 최대 0.3~0.4초 정도 지터가 생기는 게 실측 확인됨 — 기존
        # slop=0.05초는 이보다 훨씬 타이트해서 대부분의 프레임 쌍이 동기화
        # 실패로 버려지고 있었을 가능성이 높음(콜백 자체가 거의 안 불림).
        # slop을 넉넉히 늘리고 queue_size도 키워 버퍼링 여유를 둠.
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=15, slop=0.3
        )
        self._synchronizer.registerCallback(self._on_synced_images)

        self.get_logger().info(
            f'yolo_d435_detector_node 준비 완료. 추론/시각화는 '
            f'매 프레임 실시간 발행(2026-07-28 원복), '
            f"'{TARGET_CLASS_NAME}' 등 판단은 look pose에서 "
            f'{ACCUMULATION_WINDOW_SEC}초 누적 후 1회만 발행합니다. '
            + ('target_point **발행함**(publish_target_point=true) — 이 노드가 '
               '직접 팔을 움직인다.'
               if self._publish_target_point else
               'target_point는 발행하지 않는다(publish_target_point=false) — '
               '파지는 harvest_sequence_node를 거친다.')
            + f' color={color_topic}, depth={depth_topic}'
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self._frame_id = msg.header.frame_id

    def _on_set_judgment_enabled(self, request, response):
        """[2026-08-02] 판단(누적 -> candidates/target_point)을 켜고 끈다.

        끄면 진행 중인 누적도 취소한다 — 안 그러면 얼린 직후 타이머가 한 번 더
        터져 판단이 나간다.

        **켤 때 팔이 이미 look pose에 있으면 그 자리에서 다시 누적을 시작한다.**
        처음엔 "들어오는 순간만 시작점"으로 두었는데, 실물에서 막혔다: 시퀀스를
        emergency_stop으로 멈추면 판단이 얼린 채로 남고(해제는 시퀀스 정상
        종료 때만 불린다), 팔은 look pose에 서 있으니 **다시 들어오는 순간이
        영영 안 온다.** 그 상태에서 export_detections.py가 60초를 기다려도
        후보를 못 받는다(실제로 겪었다). 해제 = "지금부터 다시 봐라"로 읽는
        것이 맞다.
        """
        want = bool(request.data)
        if want == self._judgment_enabled:
            response.success = True
            response.message = f'이미 {"켜짐" if want else "얼림"} 상태'
            return response

        self._judgment_enabled = want
        if not want:
            self._reset_judgment_state()
            self.get_logger().info(
                '판단 얼림 — 수확 시퀀스가 끝날 때까지 candidates/target_point를 '
                '내지 않는다.')
        elif self._at_look_pose:
            self._start_accumulation()      # 이미 look pose면 그 자리에서 다시
            self.get_logger().info(
                f'판단 해제 — 이미 look pose라 {ACCUMULATION_WINDOW_SEC}초 누적을 '
                '바로 시작한다.')
        else:
            self.get_logger().info('판단 해제 — look pose에 들어오면 누적한다.')
        response.success = True
        response.message = '판단 켜짐' if want else '판단 얼림'
        return response

    def _on_joint_states(self, msg: JointState) -> None:
        positions_by_name = dict(zip(msg.name, msg.position))
        at_look_pose = _is_near_look_pose(positions_by_name)

        # 얼려 있으면 look pose를 드나들어도 누적을 시작하지 않는다. 위치 추적
        # (_at_look_pose)은 계속 해둬야 해제 직후 상태가 맞는다.
        if at_look_pose and not self._at_look_pose:
            self._at_look_pose = True
            if self._judgment_enabled:
                self._start_accumulation()
        elif not at_look_pose and self._at_look_pose:
            self._at_look_pose = False
            self._reset_judgment_state()

    def _start_accumulation(self) -> None:
        self._accumulated_detections = []
        self._judgment_locked = False
        if self._accumulation_timer is not None:
            self._accumulation_timer.cancel()
        self._accumulation_timer = self.create_timer(
            ACCUMULATION_WINDOW_SEC, self._finalize_accumulation
        )
        self.get_logger().info(
            f'look pose 도착 감지, {ACCUMULATION_WINDOW_SEC}초 누적 시작'
        )

    def _reset_judgment_state(self) -> None:
        if self._accumulation_timer is not None:
            self._accumulation_timer.cancel()
            self._accumulation_timer = None
        self._accumulated_detections = []
        self._judgment_locked = False
        # look pose를 벗어났으니 이전 판단 결과가 stale해짐 — 빈 배열 발행.
        self._candidates_publisher.publish(Float32MultiArray())

    def _finalize_accumulation(self) -> None:
        self._accumulation_timer.cancel()
        self._accumulation_timer = None
        self._judgment_locked = True

        clusters = _cluster_detections(self._accumulated_detections)
        self.get_logger().info(
            f'누적 종료 — 검출 {len(self._accumulated_detections)}개 -> '
            f'클러스터 {len(clusters)}개로 판단'
        )
        self._publish_candidates(clusters)
        self._publish_best_target(clusters)

    def _publish_candidates(self, clusters) -> None:
        # [2026-07-31] stride 5 -> 7. 오프라인 스윕 시뮬레이션(로봇/베드 없이
        # 노트북에서 수확 경로를 평가)에 대상의 **크기**와 관측 신뢰도가 필요해서
        # radius_m와 count를 추가함. 둘 다 이미 클러스터에 들어 있던 값이라
        # 추가 계산은 없음. 소비자는 harvest_sequence_node와
        # scripts/export_detections.py 두 곳이며 같이 갱신했다.
        flat = []
        for cluster in clusters:
            flat.extend([
                float(cluster['class_id']),
                cluster['x'],
                cluster['y'],
                cluster['z'],
                cluster['confidence'],
                cluster['radius_m'],
                float(cluster['count']),
            ])
        msg = Float32MultiArray()
        msg.data = flat
        self._candidates_publisher.publish(msg)

    def _publish_best_target(self, clusters) -> None:
        # [2026-08-02] 기본은 발행하지 않는다 — 파지 명령은 시퀀스 노드를 거친다
        # (publish_target_point 파라미터 설명 참고). 조용히 넘어가면 "왜 안
        # 움직이지"가 되므로 한 번은 남긴다.
        if not self._publish_target_point:
            self.get_logger().info(
                'target_point 미발행(publish_target_point=false) — 파지는 '
                'harvest_sequence_node의 /start_harvest_sequence로 시작할 것. '
                f'후보 {len(clusters)}개는 tomato_candidates로 발행됨.'
            )
            return

        target_clusters = [
            c for c in clusters
            if self._model.names[c['class_id']] == TARGET_CLASS_NAME
        ]
        if not target_clusters:
            self.get_logger().info(
                f"누적 구간 동안 '{TARGET_CLASS_NAME}' 대상 없음 — target_point 미발행"
            )
            return

        # confidence 최댓값이 아니라 관측 횟수 우선(더 안정적인 클러스터
        # 선택) -> 그다음 confidence 순 (so101 검증 기준).
        target_clusters.sort(key=lambda c: (c['count'], c['confidence']), reverse=True)
        best = target_clusters[0]

        # target_point보다 먼저 발행(위 모듈 docstring "target_radius_m" 설명
        # 참고) — coord_to_goal_node가 target_point를 받을 때 이미 최신
        # 반지름을 캐시해둔 상태이길 기대함.
        radius_msg = Float32()
        radius_msg.data = best['radius_m']
        self._target_radius_publisher.publish(radius_msg)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.point = Point(x=best['x'], y=best['y'], z=best['z'])
        self._publisher.publish(msg)
        self.get_logger().info(
            f"{TARGET_CLASS_NAME} 판단 완료(관측 {best['count']}회, "
            f"평균 conf={best['confidence']:.2f}, 추정 반지름="
            f"{best['radius_m'] * 1000:.1f}mm): 카메라 기준 좌표="
            f"[{best['x']:.3f}, {best['y']:.3f}, {best['z']:.3f}]"
        )

    def _publish_tomato_boxes(self, result) -> None:
        """검출된 모든 bbox(클래스 무관)를 [x1,y1,x2,y2, ...] 평탄화해 발행.

        result.boxes가 비어있어도(이번 프레임에 검출 없음) 빈 배열을
        발행함 — pointcloud_tomato_filter_node가 이전 프레임의 마스크를
        계속 유지하지 않고 제때 해제하도록 함.
        """
        flat = []
        if result.boxes is not None:
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                flat.extend([x1, y1, x2, y2])
        msg = Float32MultiArray()
        msg.data = flat
        self._boxes_publisher.publish(msg)

    def _accumulate_detections(self, result, depth_image) -> None:
        """[Tier2+Tier3] HARVEST_CLASS_NAMES(ripe/disease) 검출을 depth
        반지름 보정된 3D 좌표로 변환해 이번 look pose 방문의 누적 버퍼에
        추가함. look pose 누적 구간 중에만 호출됨(`_on_synced_images` 참고).
        """
        if result.boxes is None:
            return

        height, width = depth_image.shape[:2]
        fx, fy, ppx, ppy = self._intrinsics
        for box in result.boxes:
            class_id = int(box.cls[0])
            if self._model.names[class_id] not in HARVEST_CLASS_NAMES:
                continue

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = min(max(int((x1 + x2) / 2), 0), width - 1)
            cy = min(max(int((y1 + y2) / 2), 0), height - 1)
            # [2026-07-27 실물 조정] 중심 픽셀 딱 한 점만 보면 depth hole(반사/
            # 각도로 그 지점만 무효 depth인 경우, 실물에서 실제 재현 확인 —
            # 컬러 검출은 성공하는데 중심 픽셀 depth=0이라 매번 조용히
            # 스킵되고 있었음)에 취약함. bbox 영역 전체에서 유효한(0이 아닌)
            # depth 값들의 중앙값을 사용 — 배경이 섞여 들어와도(bbox가 물체에
            # 타이트하게 잡히는 경우가 대부분이라) 중앙값이라 견고함.
            raw_depth = _robust_bbox_depth_m(depth_image, x1, y1, x2, y2)
            if raw_depth is None:
                continue

            radius_m = _estimate_radius_m(x1, y1, x2, y2, raw_depth, fx, fy)
            depth = raw_depth + radius_m - DEPTH_SAFETY_MARGIN_M
            x = (cx - ppx) * depth / fx
            y = (cy - ppy) * depth / fy
            self._accumulated_detections.append(
                (class_id, x, y, depth, float(box.conf[0]), radius_m)
            )

    def _publish_annotated_image(self, result, header) -> None:
        annotated = result.plot()  # bbox/클래스명/confidence가 그려진 BGR 이미지
        image_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        image_msg.header = header
        self._annotated_image_publisher.publish(image_msg)

    def _on_synced_images(self, color_msg: Image, depth_msg: Image) -> None:
        if self._intrinsics is None:
            return

        # [2026-07-28 원복] 2026-07-27에 "look pose 누적 구간에서만 추론"으로
        # 되돌렸던 이유는 CPU 경합으로 인한 카메라 스트림 불안정이었는데,
        # 그 부하의 실제 주범은 YOLO 추론 자체가 아니라 Octomap 파이프라인
        # (realsense pointcloud.enable=true + pointcloud_tomato_filter_node,
        # 둘 다 그 자체로 CPU 100%+ 관측됨)이었음이 이번에 밝혀짐 — so101-
        # ros-physical-ai 자매 프로젝트는 같은 D435로 매 프레임 실시간 추론을
        # 해도 부하가 훨씬 낮은데, 그 프로젝트엔 애초에 Octomap/pointcloud
        # 파이프라인이 없다는 게 유일한 구조적 차이였음(demo_octomap.launch.py
        # 쪽 pointcloud.enable 수정과 짝을 이루는 변경). 그래서 시각화
        # (tomato_boxes/detections_image)는 다시 look pose 여부와 무관하게
        # 매 프레임 발행 — "검출 자체가 안 됐는지" vs "판단 로직이 막았는지"를
        # 육안으로 바로 구분할 수 있음. "판단"(target_point/tomato_candidates,
        # 아래 _accumulate_detections 호출)은 CPU가 아니라 자동 재트리거 사고
        # 방지가 목적인 별개 안전장치라 그대로 look pose 게이팅 유지함(위 모듈
        # docstring 참고).
        image = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_image = self._bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough'
        )

        result = self._model.predict(image, conf=CONFIDENCE_THRESHOLD, verbose=False)[0]

        if COLOR_OVERRIDE_ENABLED and len(result.boxes) > 0:
            # result.boxes.data의 마지막 열이 class id — 슬라이스라 덮어쓰면
            # .cls/plot() 등 이후 전부 이 값을 그대로 읽음(ultralytics 내부
            # 구현 확인함, pick8.py의 COLOR_OVERRIDE와 동일한 목적). 단,
            # predict() 결과 텐서는 torch inference_mode 텐서라 clone() 없이
            # in-place로 건드리면 "Inplace update to inference tensor..."
            # RuntimeError가 남 — 직접 재현해서 확인함, 반드시 clone 먼저.
            result.boxes.data = result.boxes.data.clone()
            for i, box in enumerate(result.boxes):
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                color_class = _classify_by_color(image, x1, y1, x2, y2)
                if color_class is not None:
                    result.boxes.data[i, -1] = float(self._class_name_to_id[color_class])

        self._publish_tomato_boxes(result)
        self._publish_annotated_image(result, color_msg.header)

        if self._at_look_pose and not self._judgment_locked:
            self._accumulate_detections(result, depth_image)


def main():
    rclpy.init()

    node = YoloD435DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
