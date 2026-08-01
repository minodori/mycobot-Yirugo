#!/usr/bin/env python3
"""
YOLO 검출 후보(`tomato_candidates`) 중 수확 대상(ripe/disease)을 look pose
스냅샷 기준 깊이(z, 카메라로부터의 거리) 오름차순으로 정렬해 순차적으로
`coord_to_goal_node`에 접근시키는 노드.

배경: `yolo_d435_detector_node`는 카메라가 살아있는 한 계속 검출 결과를
쏟아내는데(연속 스트림), 접근 도중 팔(=카메라, eye-in-hand)이 움직이면
같은 물체라도 좌표가 흔들리거나 팔이 시야를 가려 엉뚱한 검출로 바뀔 수 있어서,
"매 프레임 반응"이 아니라 "한 순간에 스냅샷 -> 그 목록만 갖고 끝까지 순차
처리"가 필요함(docs/obstacle_avoidance_manual_test.md "수확 순차 처리" 절
참고). 그래서:
1. `/start_harvest_sequence`(std_srvs/srv/Trigger) 호출 시점의 최신
   `tomato_candidates`(카메라 프레임 기준 3D 좌표) 스냅샷 하나만 사용.
2. 그 스냅샷을 즉시(팔이 아직 look pose에 있을 때) TF로 g_base 좌표로
   변환해 큐에 저장 — 접근 도중 팔이 움직여 camera->g_base 변환 자체가
   바뀌어버리면 나중 목표들의 좌표가 틀어지므로, 절대 나중에 다시 변환하지
   않음.
3. 원본 카메라 프레임 z(깊이, 카메라에서 가까운 순서)로 오름차순 정렬.
4. `coord_to_goal_node`가 `plan_result`(성공/실패 무관)를 발행할 때마다
   큐에서 다음 목표를 하나씩 꺼내 `/target_point`로 발행.

주의: 호출 시점 이후에 들어오는 새 `tomato_candidates`는 진행 중인 시퀀스에
영향을 주지 않음(스냅샷 고정). 시퀀스 진행 중 재호출은 거부됨.
"""

import functools

from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
import tf2_ros

BASE_LINK_NAME = 'g_base'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/camera/aligned_depth_to_color/camera_info'

# yolo_d435_detector_node의 모델 클래스 순서(model.names)와 동일 — 로그용.
NAME_BY_CLASS_ID = {0: 'ripe', 1: 'unripe', 2: 'rotten', 3: 'disease'}

# 한 목표 완료 후 다음 목표를 보내기 전 대기 시간. coord_to_goal_node의
# OCTOMAP_CLEAR_DELAY_SEC(1.2s)보다 여유 있게 잡아 그 다음 목표용 sphere
# 등록/옥토맵 갱신과 안 겹치게 함.
NEXT_TARGET_DELAY_SEC = 1.5


class HarvestSequenceNode(Node):

    def __init__(self):
        super().__init__('harvest_sequence_node')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._camera_frame_id = None
        self._latest_candidates = []  # [(class_id, x, y, z, confidence), ...]
        self._queue = []  # [(Point(g_base), class_id, confidence, original_depth), ...]
        self._waiting_for_result = False
        self._next_target_timer = None

        self.create_subscription(
            CameraInfo, DEFAULT_CAMERA_INFO_TOPIC, self._on_camera_info, 10
        )
        self.create_subscription(
            Float32MultiArray, 'tomato_candidates', self._on_candidates, 10
        )
        self.create_subscription(Bool, 'plan_result', self._on_plan_result, 10)
        self._target_publisher = self.create_publisher(
            PointStamped, 'target_point', 10
        )
        self.create_service(
            Trigger, 'start_harvest_sequence', self._on_start_harvest
        )
        # [2026-08-01] 시퀀스 종료 후 look pose 복귀용. coord_to_goal_node의
        # 사이클 복귀가 armed pose로 바뀌어서, 다음 관측 전에 한 번은 카메라를
        # 베드로 돌려놔야 한다(_return_to_look_pose 참고).
        self._go_to_look_pose_client = self.create_client(
            Trigger, 'go_to_look_pose'
        )

        self.get_logger().info(
            'harvest_sequence_node 준비 완료. /start_harvest_sequence 서비스 대기 중 '
            '(팔이 look pose에서 대상을 보고 있는 상태에서 호출할 것).'
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_frame_id = msg.header.frame_id

    def _on_candidates(self, msg: Float32MultiArray) -> None:
        # [2026-07-31] yolo_d435_detector_node가 stride 5 -> 7로 바뀜
        # (radius_m, count 추가). 이 노드는 앞의 5개만 쓰지만, 길이가 안 맞는
        # 잔여분을 조용히 잘못 읽지 않도록 stride를 맞춰 둔다.
        data = list(msg.data)
        stride = 7
        parsed = []
        for i in range(0, len(data) - len(data) % stride, stride):
            class_id, x, y, z, confidence = data[i:i + 5]
            parsed.append((int(class_id), x, y, z, confidence))
        self._latest_candidates = parsed

    def _on_start_harvest(self, request, response) -> Trigger.Response:
        if self._waiting_for_result or self._queue:
            response.success = False
            response.message = '이미 수확 시퀀스 진행 중 — 완료 후 다시 호출할 것'
            return response

        if not self._latest_candidates:
            response.success = False
            response.message = 'tomato_candidates가 비어있음(검출된 ripe/disease 없음)'
            return response

        if self._camera_frame_id is None:
            response.success = False
            response.message = 'camera_info를 아직 못 받음(frame_id 미확인)'
            return response

        try:
            transform = self._tf_buffer.lookup_transform(
                BASE_LINK_NAME,
                self._camera_frame_id,
                Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            response.success = False
            response.message = f'TF2 변환 실패({self._camera_frame_id} -> {BASE_LINK_NAME}): {exc}'
            return response

        # 스냅샷 즉시 전부 g_base로 변환 — 시퀀스 진행 중 팔이 움직여도(camera
        # 프레임 자체가 바뀌어도) 이후 목표 좌표가 흔들리지 않도록 함(위 모듈
        # docstring 참고).
        queue = []
        for class_id, x, y, z, confidence in self._latest_candidates:
            camera_point = PointStamped()
            camera_point.header.frame_id = self._camera_frame_id
            camera_point.point = Point(x=x, y=y, z=z)
            base_point = do_transform_point(camera_point, transform)
            queue.append((base_point.point, class_id, confidence, z))

        # 원본 카메라 프레임 z(카메라로부터의 깊이) 오름차순 — 가까운 것부터.
        queue.sort(key=lambda item: item[3])
        self._queue = queue

        response.success = True
        response.message = f'{len(queue)}개 대상 큐잉, 가까운 순서로 순차 접근 시작'
        self.get_logger().info(response.message)
        self._publish_next_target()
        return response

    def _publish_next_target(self) -> None:
        if not self._queue:
            self.get_logger().info('수확 시퀀스 완료 — 큐 비어있음.')
            self._return_to_look_pose()
            return

        point, class_id, confidence, depth = self._queue.pop(0)
        class_name = NAME_BY_CLASS_ID.get(class_id, f'class_{class_id}')

        msg = PointStamped()
        msg.header.frame_id = BASE_LINK_NAME
        msg.point = point
        self._waiting_for_result = True
        self._target_publisher.publish(msg)
        self.get_logger().info(
            f'다음 목표 발행: {class_name}(conf={confidence:.2f}, '
            f'원본 깊이={depth:.3f}m) -> g_base [{point.x:.3f}, {point.y:.3f}, '
            f'{point.z:.3f}] (남은 큐: {len(self._queue)}개)'
        )

    def _return_to_look_pose(self) -> None:
        """[2026-08-01] 시퀀스가 끝나면 look pose로 한 번 돌아간다.

        `coord_to_goal_node`의 사이클 복귀가 look pose -> armed pose로 바뀌면서
        (그 파일 ARMED_POSE_JOINT_POSITIONS 주석 참고) 시퀀스가 끝나도 팔이
        베드를 보지 않는 자세에 남는다. 그러면 다음 `/start_harvest_sequence`가
        쓸 `tomato_candidates`가 갱신되지 않는다 — YOLO 판단이 look pose에서만
        열리기 때문이다.

        관측은 시퀀스당 1회면 충분하므로 여기서 한 번만 돌아가면 된다.
        서비스가 없거나 실패해도 시퀀스 자체는 이미 끝났으므로 경고만 남긴다.
        """
        if not self._go_to_look_pose_client.service_is_ready():
            self.get_logger().warn(
                '/go_to_look_pose 서비스가 없어 look pose 복귀를 건너뜀 — '
                '다음 스냅샷 전에 수동으로 이동시킬 것.'
            )
            return
        self.get_logger().info('다음 관측을 위해 look pose로 복귀 요청.')
        self._go_to_look_pose_client.call_async(Trigger.Request())

    def _on_plan_result(self, msg: Bool) -> None:
        if not self._waiting_for_result:
            # 우리가 시작하지 않은 결과(수동 테스트 등)는 시퀀스에 영향 없음.
            return

        self._waiting_for_result = False
        self.get_logger().info(f'이전 목표 결과: {"성공" if msg.data else "실패"}')

        if self._next_target_timer is not None:
            self._next_target_timer.cancel()
        self._next_target_timer = self.create_timer(
            NEXT_TARGET_DELAY_SEC, self._on_next_target_timer
        )

    def _on_next_target_timer(self) -> None:
        self._next_target_timer.cancel()
        self._next_target_timer = None
        self._publish_next_target()


def main():
    rclpy.init()

    node = HarvestSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
