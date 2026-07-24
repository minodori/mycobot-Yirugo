#!/usr/bin/env python3
"""
YOLO 토마토 검출 bbox로 D435 depth 이미지를 사전 마스킹해서 Octomap이
토마토 자체를 장애물로 잡지 않게 만드는 필터 노드.

배경: coord_to_goal_node의 기존 방식(목표 지점에 sphere CollisionObject를
등록해 Octomap 자기 필터를 유도 + 플래닝 직전 /clear_octomap 호출)은 그 순간
선택된 목표 토마토 1개만 커버하고, 타이밍에 의존하는 임시방편이라 그리퍼
근접 잔여 voxel 문제가 계속 재발함(docs/obstacle_avoidance_manual_test.md
"알려진 문제" 참고). 이 노드는 그 대신 depth 이미지 단계에서 애초에 토마토
픽셀 영역의 깊이 값 자체를 제거해서, 검출된 모든 토마토가(목표든 아니든)
Octomap에 장애물로 등록되지 않게 함 — 사후 클리어가 아니라 사전 차단.

[중요, 2026-07-24 세션에서 발견/수정] 최초 구현은 realsense가 직접 발행하는
'/camera/camera/depth/color/points'(pointcloud.ordered_pc:=true로 받은
organized cloud)를 그대로 구독해서 YOLO bbox(컬러 이미지 픽셀 좌표)를 그
그리드에 바로 적용했으나, 실물 테스트에서 RViz Octomap에 토마토 위치
구멍이 전혀 안 생기는 것으로 확인됨. 원인: 그 pointcloud의 frame_id가
`camera_depth_optical_frame`인 데서 알 수 있듯, depth 센서 고유의 픽셀
그리드를 따름(컬러 텍스처는 별도 재투영으로 입혀지지만 그리드 자체는 depth
원본 해상도/광학중심 기준) — `align_depth`를 켜도 이 pointcloud 생성 경로
자체는 안 바뀜. 반면 YOLO bbox는 컬러 이미지 픽셀 좌표라서, 해상도 숫자만
우연히 같을 뿐 서로 다른 광학중심/FOV를 가진 별개 그리드였음.
`yolo_d435_detector_node`가 애초에 raw depth 대신
`aligned_depth_to_color/image_raw`를 쓰는 이유가 정확히 이 정렬 문제
때문인데(그 모듈 상단 주석 참고), 이 필터 노드에는 그 교훈이 반영 안 됐던
것.

수정: realsense pointcloud를 아예 구독하지 않고,
`yolo_d435_detector_node`와 똑같이 `aligned_depth_to_color`(image_raw +
camera_info)를 직접 구독해서 픽셀->3D 변환(핀홀 근사, 왜곡 무시 — 같은
근거로 yolo 노드도 무시함)을 이 노드 스스로 수행함. bbox 마스킹과 pointcloud
생성이 완전히 같은 픽셀 그리드(컬러 프레임에 정렬된 depth) 위에서 이뤄지므로
그리드 불일치가 구조적으로 불가능해짐. 이에 따라 realsense
`pointcloud.ordered_pc:=true` 인자도 더 이상 필요 없어져 제거함.

입력:
  - <depth_topic> (sensor_msgs/msg/Image): 정렬된 depth 이미지(16UC1, mm
    단위). 기본값: /camera/camera/aligned_depth_to_color/image_raw
  - <camera_info_topic> (sensor_msgs/msg/CameraInfo): 위 depth와 같은 프레임의
    핀홀 내부 파라미터(K). 기본값:
    /camera/camera/aligned_depth_to_color/camera_info
  - <boxes_topic> (std_msgs/msg/Float32MultiArray): [x1,y1,x2,y2, ...]
    평탄화된 bbox 목록(컬러 이미지 픽셀 좌표, 클래스 무관 전체 검출).
    기본값: tomato_boxes. yolo_d435_detector_node가 발행함. 새 메시지가
    올 때마다 캐시를 통째로 교체함 — 이후 들어오는 모든 depth 프레임에
    캐시된 값을 그대로 적용(비동기, 느슨한 동기화 — YOLO 추론 자체가 이미
    ~1Hz로 throttle돼 있어 충분함).

파라미터:
  - bbox_padding_ratio (기본값 0.2): bbox 각 변을 이 비율만큼 바깥으로
    확장해서 마스킹함(2026-07-24, 실물 테스트에서 토마토가 작아 마스킹
    효과가 눈으로 명확히 안 보여 추가 — bbox가 물체 경계에 타이트하게
    잡히거나 depth-color 정렬에 픽셀 단위 슬랙이 있으면 테두리 voxel이
    안 지워질 수 있어 여유를 둠. 픽셀 고정값이 아니라 비율인 이유는 거리가
    멀어져 bbox가 작아져도 상대적 여유가 유지되게 하기 위함).

출력:
  - <output_topic> (sensor_msgs/msg/PointCloud2): x/y/z 필드만 가진 필터링된
    pointcloud, frame_id는 depth_topic/camera_info_topic과 동일(보통
    camera_color_optical_frame). 기본값:
    /camera/camera/depth/color/points_filtered. sensors_3d.yaml의
    point_cloud_topic을 이 토픽으로 바꿔서 occupancy_map_monitor가 raw 대신
    이걸 구독하게 해야 실제로 적용됨.
"""

import math

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray

DEFAULT_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/camera/aligned_depth_to_color/camera_info'
DEFAULT_OUTPUT_TOPIC = '/camera/camera/depth/color/points_filtered'
DEFAULT_BOXES_TOPIC = 'tomato_boxes'

# YOLO bbox는 물체 경계에 딱 맞게(타이트하게) 잡히는 경우가 많고, depth-color
# 정렬도 픽셀 단위로 약간의 슬랙이 있을 수 있어서, bbox 그대로만 마스킹하면
# 테두리 voxel이 안 지워지고 남을 수 있음. bbox 각 변을 이 비율만큼 바깥으로
# 확장해서 여유를 둠(비율 기반이라 거리가 멀어져 bbox가 작아져도 상대적
# 여유는 유지됨 — 픽셀 고정값보다 이게 더 일관적).
# DEFAULT_BBOX_PADDING_RATIO = 0.2
DEFAULT_BBOX_PADDING_RATIO = 0.5

# 출력 pointcloud는 occupancy_map_monitor가 필요로 하는 x/y/z만 담음.
_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP = 12


class PointcloudTomatoFilterNode(Node):

    def __init__(self):
        super().__init__('pointcloud_tomato_filter_node')

        self.declare_parameter('depth_topic', DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('camera_info_topic', DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter('output_topic', DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter('boxes_topic', DEFAULT_BOXES_TOPIC)
        self.declare_parameter('bbox_padding_ratio', DEFAULT_BBOX_PADDING_RATIO)

        depth_topic = (
            self.get_parameter('depth_topic').get_parameter_value().string_value
        )
        camera_info_topic = (
            self.get_parameter('camera_info_topic')
            .get_parameter_value()
            .string_value
        )
        output_topic = (
            self.get_parameter('output_topic').get_parameter_value().string_value
        )
        boxes_topic = (
            self.get_parameter('boxes_topic').get_parameter_value().string_value
        )
        self._bbox_padding_ratio = (
            self.get_parameter('bbox_padding_ratio')
            .get_parameter_value()
            .double_value
        )

        self._bridge = CvBridge()
        self._intrinsics = None  # (fx, fy, cx, cy), camera_info 수신 시 채워짐
        # 픽셀 좌표 그리드(u, v)는 이미지 크기가 바뀌지 않는 한 매 프레임 같으므로
        # 캐시해서 재사용함(매번 np.indices 재계산 방지).
        self._pixel_grid_shape = None
        self._u_grid = None
        self._v_grid = None
        # [(x1, y1, x2, y2), ...], 컬러 이미지 픽셀 좌표.
        self._boxes = []

        # occupancy_map_monitor(moveit PointCloudOctomapUpdater)가 RELIABLE로
        # 구독을 요청함(2026-07-24 실물 테스트로 확인 — "incompatible QoS ...
        # Last incompatible policy: RELIABILITY" 경고 발생, best-effort로는
        # 메시지가 전혀 전달 안 됨). 그래서 기본 QoS(reliable, keep-last)를 씀.
        self._publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self._boxes_sub = self.create_subscription(
            Float32MultiArray, boxes_topic, self._on_boxes, 10
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, 10
        )
        # yolo_d435_detector_node와 동일하게 기본 QoS로 구독함(이미 이 조합으로
        # 정상 동작 확인됨).
        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth, 10
        )

        self.get_logger().info(
            f'pointcloud_tomato_filter_node 준비 완료. {depth_topic} -> '
            f'{output_topic} (bbox 구독: {boxes_topic})'
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_boxes(self, msg: Float32MultiArray) -> None:
        data = msg.data
        usable_len = len(data) - len(data) % 4
        self._boxes = [
            tuple(data[i:i + 4]) for i in range(0, usable_len, 4)
        ]

    def _on_depth(self, depth_msg: Image) -> None:
        if self._intrinsics is None:
            return

        depth_image = self._bridge.imgmsg_to_cv2(
            depth_msg, desired_encoding='passthrough'
        )
        height, width = depth_image.shape[:2]

        if self._pixel_grid_shape != (height, width):
            self._v_grid, self._u_grid = np.indices((height, width), dtype=np.float32)
            self._pixel_grid_shape = (height, width)

        fx, fy, cx, cy = self._intrinsics
        depth_m = depth_image.astype(np.float32) / 1000.0

        xs = (self._u_grid - cx) * depth_m / fx
        ys = (self._v_grid - cy) * depth_m / fy
        zs = depth_m.copy()

        invalid = depth_m <= 0.0
        xs[invalid] = math.nan
        ys[invalid] = math.nan
        zs[invalid] = math.nan

        for x1, y1, x2, y2 in self._boxes:
            pad_x = (x2 - x1) * self._bbox_padding_ratio
            pad_y = (y2 - y1) * self._bbox_padding_ratio
            col_start = max(0, int(math.floor(x1 - pad_x)))
            col_end = min(width, int(math.ceil(x2 + pad_x)))
            row_start = max(0, int(math.floor(y1 - pad_y)))
            row_end = min(height, int(math.ceil(y2 + pad_y)))
            if col_start >= col_end or row_start >= row_end:
                continue
            xs[row_start:row_end, col_start:col_end] = math.nan
            ys[row_start:row_end, col_start:col_end] = math.nan
            zs[row_start:row_end, col_start:col_end] = math.nan

        stacked = np.empty((height, width, 3), dtype=np.float32)
        stacked[..., 0] = xs
        stacked[..., 1] = ys
        stacked[..., 2] = zs

        cloud = PointCloud2()
        cloud.header = depth_msg.header
        cloud.height = height
        cloud.width = width
        cloud.fields = _FIELDS
        cloud.is_bigendian = False
        cloud.point_step = _POINT_STEP
        cloud.row_step = _POINT_STEP * width
        cloud.is_dense = False
        cloud.data = stacked.tobytes()

        self._publisher.publish(cloud)


def main():
    rclpy.init()

    node = PointcloudTomatoFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
