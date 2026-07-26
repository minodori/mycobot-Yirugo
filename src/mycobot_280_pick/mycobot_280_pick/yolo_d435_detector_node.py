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
  - 시각화(tomato_boxes/tomato_detections_image)는 look pose 여부와 무관하게
    항상 매 프레임 발행 — "검출 자체가 안 됐는지" vs "판단 로직이 막았는지"를
    육안으로 구분할 수 있어야 하므로.
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
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO

TARGET_CLASS_NAME = 'ripe'
CONFIDENCE_THRESHOLD = 0.4

# [2026-07-24, 수확 순차 처리] 모델 4클래스({ripe, unripe, rotten, disease}) 중
# 실제 수확/제거 대상. harvest_sequence_node가 이 후보 목록을 받아 look pose
# 스냅샷 기준으로 깊이(z) 오름차순 정렬해 순차 접근함(docs/
# obstacle_avoidance_manual_test.md "수확 순차 처리" 절 참고).
HARVEST_CLASS_NAMES = ('ripe', 'disease')

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
ACCUMULATION_WINDOW_SEC = 2.0
# [Tier2] 3D 위치 기준 클러스터링 거리 임계값 — 이 이내면 같은 대상으로 취급
# (같은 클래스인 경우만, so101 검증값).
CLUSTER_DISTANCE_M = 0.03

# [Tier3, depth 반지름 보정] 추정 반지름의 상하한 clamp — 실측 토마토
# 지름(~4cm)보다 넉넉한 범위(so101이 검증한 값과 동일).
MIN_ESTIMATED_RADIUS_M = 0.015
MAX_ESTIMATED_RADIUS_M = 0.035

DEFAULT_MODEL_PATH = os.path.expanduser(
    '~/Projects/Eval_Yolo/tomato_4cls_model.pt'
)
DEFAULT_COLOR_TOPIC = '/camera/camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/camera/aligned_depth_to_color/camera_info'


def _is_near_look_pose(positions_by_name: dict) -> bool:
    for name, look_value in zip(JOINT_NAMES, LOOK_POSE_JOINT_POSITIONS):
        current_value = positions_by_name.get(name)
        if current_value is None or abs(current_value - look_value) > LOOK_POSE_TOLERANCE_RAD:
            return False
    return True


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
    """[Tier2] 누적된 검출(class_id, x, y, z, confidence) 리스트를 3D 위치
    기준으로 클러스터링함(같은 class_id + CLUSTER_DISTANCE_M 이내 거리면 같은
    대상으로 취급, 평균 위치/신뢰도로 병합). so101이 실물로 검증한 방식과 동일.
    반환: [{'class_id','x','y','z','confidence','count'}, ...]
    """
    clusters = []
    for class_id, x, y, z, confidence in detections:
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
                'count': 1,
            })
        else:
            matched['sum_x'] += x
            matched['sum_y'] += y
            matched['sum_z'] += z
            matched['sum_conf'] += confidence
            matched['count'] += 1

    return [
        {
            'class_id': cluster['class_id'],
            'x': cluster['sum_x'] / cluster['count'],
            'y': cluster['sum_y'] / cluster['count'],
            'z': cluster['sum_z'] / cluster['count'],
            'confidence': cluster['sum_conf'] / cluster['count'],
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
        self._bridge = CvBridge()
        self._intrinsics = None  # (fx, fy, cx, cy), camera_info 수신 시 채워짐
        self._frame_id = None

        # [Tier2] look pose 게이팅 + 누적판단 상태.
        self._at_look_pose = False
        self._judgment_locked = False
        self._accumulated_detections = []  # [(class_id, x, y, z, confidence), ...]
        self._accumulation_timer = None

        self._publisher = self.create_publisher(PointStamped, 'target_point', 10)
        self._boxes_publisher = self.create_publisher(
            Float32MultiArray, 'tomato_boxes', 10
        )
        self._candidates_publisher = self.create_publisher(
            Float32MultiArray, 'tomato_candidates', 10
        )
        self._annotated_image_publisher = self.create_publisher(
            Image, 'tomato_detections_image', 10
        )

        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, 10
        )
        self._joint_states_sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_states, 10
        )

        color_sub = message_filters.Subscriber(self, Image, color_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.05
        )
        self._synchronizer.registerCallback(self._on_synced_images)

        self.get_logger().info(
            f'yolo_d435_detector_node 준비 완료. 시각화는 항상 발행하고, '
            f"'{TARGET_CLASS_NAME}' 등 판단은 look pose에서 "
            f'{ACCUMULATION_WINDOW_SEC}초 누적 후 1회만 /target_point에 '
            f'발행합니다. color={color_topic}, depth={depth_topic}'
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self._frame_id = msg.header.frame_id

    def _on_joint_states(self, msg: JointState) -> None:
        positions_by_name = dict(zip(msg.name, msg.position))
        at_look_pose = _is_near_look_pose(positions_by_name)

        if at_look_pose and not self._at_look_pose:
            self._at_look_pose = True
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
        flat = []
        for cluster in clusters:
            flat.extend([
                float(cluster['class_id']),
                cluster['x'],
                cluster['y'],
                cluster['z'],
                cluster['confidence'],
            ])
        msg = Float32MultiArray()
        msg.data = flat
        self._candidates_publisher.publish(msg)

    def _publish_best_target(self, clusters) -> None:
        ripe_clusters = [
            c for c in clusters
            if self._model.names[c['class_id']] == TARGET_CLASS_NAME
        ]
        if not ripe_clusters:
            self.get_logger().info(
                '누적 구간 동안 ripe 대상 없음 — target_point 미발행'
            )
            return

        # confidence 최댓값이 아니라 관측 횟수 우선(더 안정적인 클러스터
        # 선택) -> 그다음 confidence 순 (so101 검증 기준).
        ripe_clusters.sort(key=lambda c: (c['count'], c['confidence']), reverse=True)
        best = ripe_clusters[0]

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.point = Point(x=best['x'], y=best['y'], z=best['z'])
        self._publisher.publish(msg)
        self.get_logger().info(
            f"{TARGET_CLASS_NAME} 판단 완료(관측 {best['count']}회, "
            f"평균 conf={best['confidence']:.2f}): 카메라 기준 좌표="
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
            raw_depth = float(depth_image[cy, cx]) / 1000.0
            if raw_depth <= 0.0:
                continue

            radius_m = _estimate_radius_m(x1, y1, x2, y2, raw_depth, fx, fy)
            depth = raw_depth + radius_m
            x = (cx - ppx) * depth / fx
            y = (cy - ppy) * depth / fy
            self._accumulated_detections.append(
                (class_id, x, y, depth, float(box.conf[0]))
            )

    def _publish_annotated_image(self, result, header) -> None:
        annotated = result.plot()  # bbox/클래스명/confidence가 그려진 BGR 이미지
        image_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        image_msg.header = header
        self._annotated_image_publisher.publish(image_msg)

    def _on_synced_images(self, color_msg: Image, depth_msg: Image) -> None:
        if self._intrinsics is None:
            return

        image = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_image = self._bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough'
        )

        result = self._model.predict(image, conf=CONFIDENCE_THRESHOLD, verbose=False)[0]

        # [Tier2] 시각화는 look pose 여부와 무관하게 항상 발행.
        self._publish_tomato_boxes(result)
        self._publish_annotated_image(result, color_msg.header)

        # "판단"은 look pose 누적 구간에서만 진행(모듈 docstring 참고).
        if not self._at_look_pose or self._judgment_locked:
            return

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
