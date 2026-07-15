#!/usr/bin/env python3
"""
D435 RGB-D 캡처 -> YOLO 토마토 검출 -> 3D 좌표 계산 -> /target_point 발행 노드.

로드맵 2단계: 하드코딩 좌표를 실제 YOLO+D435 추론 결과로 교체.
Eval_Yolo/detect_camera.py에서 검증된 D435 align + depth 조회 방식과 동일한
방식을 사용하고, 여기에 픽셀->3D 변환(rs2_deproject_pixel_to_point)을 추가함.

여러 토마토가 동시에 검출되면 'ripe'(익음) 클래스 중 confidence가 가장 높은
1개만 수확 대상으로 선택해 발행함 (익은 토마토만 수확 대상이므로).

발행: /target_point (geometry_msgs/msg/PointStamped), PUBLISH_RATE_HZ 주기로 계속 발행.
  - header.frame_id: CAMERA_FRAME_ID ("camera_color_optical_frame" — 추후 ROS2
    realsense2_camera 래퍼로 전환해도 같은 이름 규칙이라 그대로 호환됨)
  - point: D435 컬러 광학 프레임 기준 3D 좌표 (m 단위)

주의: 핸드-아이 캘리브레이션(로드맵 3단계) 전이라 camera_color_optical_frame ->
g_base로 가는 TF가 아직 없음. coord_to_goal_node는 frame_id가 BASE_LINK_NAME과
다르면 TF2 변환을 시도하므로, 캘리브레이션 전에는 TF 조회 실패 로그가 정상이며
이는 좌표 계산 파이프라인 자체가 잘 동작하는지 검증하는 단계임 (좌표->로봇
이동까지 실제로 되려면 캘리브레이션이 필요함).

설치 (pip 패키지라 package.xml/rosdep 대상이 아님, 전역 환경에 직접 설치):
  pip install ultralytics pyrealsense2 --break-system-packages
"""

import os

from geometry_msgs.msg import Point, PointStamped
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from ultralytics import YOLO

CAMERA_FRAME_ID = 'camera_color_optical_frame'
TARGET_CLASS_NAME = 'ripe'
CONFIDENCE_THRESHOLD = 0.4
PUBLISH_RATE_HZ = 1.0
COLOR_WIDTH, COLOR_HEIGHT, FPS = 1280, 720, 30

DEFAULT_MODEL_PATH = os.path.expanduser(
    '~/Projects/Eval_Yolo/tomato_4cls_model.pt'
)


class YoloD435DetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_d435_detector_node')

        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        model_path = (
            self.get_parameter('model_path').get_parameter_value().string_value
        )

        self.get_logger().info(f'YOLO 모델 로드 중: {model_path}')
        self._model = YOLO(model_path)

        self._pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, FPS)
        config.enable_stream(rs.stream.depth, COLOR_WIDTH, COLOR_HEIGHT, rs.format.z16, FPS)
        self._pipeline.start(config)
        self._align = rs.align(rs.stream.color)

        self._publisher = self.create_publisher(PointStamped, 'target_point', 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._on_timer)

        self.get_logger().info(
            f"yolo_d435_detector_node 준비 완료. '{TARGET_CLASS_NAME}' 클래스를 "
            f'{PUBLISH_RATE_HZ}Hz로 /target_point에 발행합니다.'
        )

    def _on_timer(self) -> None:
        frames = self._pipeline.wait_for_frames()
        frames = self._align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        image = np.asanyarray(color_frame.get_data())
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
        for box in ripe_boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            candidate_cx, candidate_cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            candidate_depth = depth_frame.get_distance(candidate_cx, candidate_cy)
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

        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        x, y, z = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = CAMERA_FRAME_ID
        msg.point = Point(x=x, y=y, z=z)
        self._publisher.publish(msg)

        self.get_logger().info(
            f'{TARGET_CLASS_NAME} 검출 (conf={best_confidence:.2f}): '
            f'카메라 기준 좌표=[{x:.3f}, {y:.3f}, {z:.3f}]'
        )

    def destroy_node(self) -> bool:
        self._pipeline.stop()
        return super().destroy_node()


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
