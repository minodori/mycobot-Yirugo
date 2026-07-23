#!/usr/bin/env python3
"""
D435 RGB-D(ROS2 realsense2_camera 토픽 구독) -> YOLO 토마토 검출 -> 3D 좌표 계산
-> /target_point 발행 노드.

이전 버전은 pyrealsense2로 카메라를 직접 열었는데(rs.pipeline()), Octomap용
realsense2_camera_node가 이미 D435를 물고 있으면 장치 충돌이 남(D435는 한
프로세스만 열 수 있음). 그래서 demo_octomap.launch.py가 띄우는 ROS2 토픽
(image_raw/aligned_depth_to_color/camera_info)을 그대로 구독하도록 변경 —
Octomap과 YOLO 검출이 카메라 하나를 공유해서 동시에 동작함.

여러 토마토가 동시에 검출되면 'ripe'(익음) 클래스 중 confidence가 가장 높은
1개만 수확 대상으로 선택해 발행함 (익은 토마토만 수확 대상이므로).

발행: /target_point (geometry_msgs/msg/PointStamped), 최대 PUBLISH_RATE_HZ로 발행.
  - header.frame_id: camera_info의 frame_id를 그대로 씀 (보통
    "camera_color_optical_frame" — align_depth로 depth를 color 프레임에
    맞췄으므로 두 이미지 다 이 프레임 기준).
  - point: 위 프레임 기준 3D 좌표 (m 단위)

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
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO

TARGET_CLASS_NAME = 'ripe'
CONFIDENCE_THRESHOLD = 0.4
PUBLISH_RATE_HZ = 1.0
MIN_PUBLISH_PERIOD = 1.0 / PUBLISH_RATE_HZ

DEFAULT_MODEL_PATH = os.path.expanduser(
    '~/Projects/Eval_Yolo/tomato_4cls_model.pt'
)
DEFAULT_COLOR_TOPIC = '/camera/camera/color/image_raw'
DEFAULT_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/camera/aligned_depth_to_color/camera_info'


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
        self._last_publish_time = 0.0

        self._publisher = self.create_publisher(PointStamped, 'target_point', 10)

        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, 10
        )

        color_sub = message_filters.Subscriber(self, Image, color_topic)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.05
        )
        self._synchronizer.registerCallback(self._on_synced_images)

        self.get_logger().info(
            f"yolo_d435_detector_node 준비 완료. '{TARGET_CLASS_NAME}' 클래스를 "
            f'최대 {PUBLISH_RATE_HZ}Hz로 /target_point에 발행합니다. '
            f'color={color_topic}, depth={depth_topic}'
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self._frame_id = msg.header.frame_id

    def _on_synced_images(self, color_msg: Image, depth_msg: Image) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_publish_time < MIN_PUBLISH_PERIOD:
            return
        if self._intrinsics is None:
            return

        image = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_image = self._bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough'
        )

        result = self._model.predict(image, conf=CONFIDENCE_THRESHOLD, verbose=False)[0]
        if result.boxes is None or len(result.boxes) == 0:
            return

        ripe_boxes = [
            box for box in result.boxes
            if self._model.names[int(box.cls[0])] == TARGET_CLASS_NAME
        ]
        if not ripe_boxes:
            return
        ripe_boxes.sort(key=lambda box: float(box.conf[0]), reverse=True)

        # bbox 중심 픽셀이 특정 표면(반사/모서리)에서 depth를 못 얻는 경우가
        # 종종 있어서, confidence 1등만 보지 않고 depth가 유효한 것을 찾을
        # 때까지 순서대로 시도함.
        cx = cy = None
        depth = 0.0
        best_confidence = 0.0
        height, width = depth_image.shape[:2]
        for box in ripe_boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            candidate_cx = min(max(int((x1 + x2) / 2), 0), width - 1)
            candidate_cy = min(max(int((y1 + y2) / 2), 0), height - 1)
            candidate_depth = float(depth_image[candidate_cy, candidate_cx]) / 1000.0
            if candidate_depth > 0.0:
                cx, cy, depth = candidate_cx, candidate_cy, candidate_depth
                best_confidence = float(box.conf[0])
                break

        if cx is None:
            self.get_logger().warn(
                f'{TARGET_CLASS_NAME} {len(ripe_boxes)}개 검출했지만 '
                'depth가 유효한 픽셀이 하나도 없음'
            )
            return

        fx, fy, ppx, ppy = self._intrinsics
        x = (cx - ppx) * depth / fx
        y = (cy - ppy) * depth / fy
        z = depth

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.point = Point(x=x, y=y, z=z)
        self._publisher.publish(msg)
        self._last_publish_time = now

        self.get_logger().info(
            f'{TARGET_CLASS_NAME} 검출 (conf={best_confidence:.2f}): '
            f'카메라 기준 좌표=[{x:.3f}, {y:.3f}, {z:.3f}]'
        )


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
