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

[2026-08-02] 둘이 더 붙었다.

1. **목표 출처를 고를 수 있다** — `target_source:=yolo`(기본) | `file`.
   `file`이면 `targets_file`(scripts/export_detections.py 출력 형식)을 읽는다.
   YOLO 좌표는 실제 파지에서 오차가 있어(카메라 쪽 표면 depth) 사람이 보정한
   파일을 쓰는 경로가 필요하다. 파일 좌표는 g_base라 TF를 안 타므로 팔이 look
   pose에 없어도 유효하다.

2. **시퀀스가 도는 동안 YOLO 판단을 얼린다** — `set_judgment_enabled(False)`.
   스냅샷 방식이라 큐 자체는 안 흔들리지만, 판단이 열리는 시점이 사이클마다
   다르다(LPC는 목표마다 look pose로 복귀해 매번 열린다). 얼려 두면 진행 중
   candidates가 덮이는 혼란과, `publish_target_point:=true`인 세션의 자동
   이동을 둘 다 막는다.

또 하나 고친 것: **목표마다 그 목표의 `target_radius_m`를 먼저 발행한다.**
예전엔 YOLO가 최적 목표 하나에 대해서만 이 값을 내서, 시퀀스의 2번째 이후
목표는 남의 반지름으로 그리퍼를 열고 있었다.
"""

import functools
import json
import os

from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, ColorRGBA, Float32, Float32MultiArray
from std_srvs.srv import SetBool, Trigger
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

BASE_LINK_NAME = 'g_base'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/camera/aligned_depth_to_color/camera_info'

# yolo_d435_detector_node의 모델 클래스 순서(model.names)와 동일 — 로그용.
NAME_BY_CLASS_ID = {0: 'ripe', 1: 'unripe', 2: 'rotten', 3: 'disease'}

# 한 목표 완료 후 다음 목표를 보내기 전 대기 시간. coord_to_goal_node의
# OCTOMAP_CLEAR_DELAY_SEC(1.2s)보다 여유 있게 잡아 그 다음 목표용 sphere
# 등록/옥토맵 갱신과 안 겹치게 함.
NEXT_TARGET_DELAY_SEC = 1.5

# 파일 경로에서 radius_m가 없을 때 쓰는 기본 반지름(m).
# coord_to_goal_node의 GRIPPER_TARGET_RADIUS_MAX_M과 같은 값 — 가장 넓게 여는
# 쪽이 안전하다(좁게 열면 열매를 밀어낸다).
DEFAULT_TARGET_RADIUS_M = 0.018


class HarvestSequenceNode(Node):

    def __init__(self):
        super().__init__('harvest_sequence_node')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # [2026-08-02] 목표를 어디서 받나.
        #
        #   yolo (기본)  look pose에서 판단한 tomato_candidates 스냅샷
        #   file         사람이 손으로 고친 좌표 파일(targets_file)
        #
        # 왜 파일 경로가 필요한가: YOLO 좌표는 실제로 파지해 보면 오차가 있다
        # (bbox depth는 물체의 카메라 쪽 **표면**이라 중심보다 짧게 잡힌다 —
        # yolo_d435_detector_node의 Tier3 주석). 그 오차를 사람이 보정한 파일을
        # 쓰는 경로다. 형식은 scripts/export_detections.py의 출력 그대로라
        # (base_x/base_y/base_z, radius_m, class_name) 새 형식을 만들지 않았다.
        #
        # 파일 좌표는 **g_base**라 TF를 안 탄다 — coord_to_goal_node가
        # frame_id == 'g_base'면 변환을 건너뛴다. 즉 팔이 look pose에 없어도
        # 유효하다(YOLO 경로와 다른 점).
        self.declare_parameter('target_source', 'yolo')
        self.declare_parameter('targets_file', '')
        self._target_source = (
            self.get_parameter('target_source').get_parameter_value().string_value
            or 'yolo'
        ).strip().lower()
        if self._target_source not in ('yolo', 'file'):
            self.get_logger().warn(
                f"target_source:={self._target_source}는 모르는 값 — "
                "'yolo'로 진행함 (yolo | file)")
            self._target_source = 'yolo'
        self._targets_file = (
            self.get_parameter('targets_file').get_parameter_value().string_value
        )
        # 파일 경로에서 어떤 순서로 큐를 만들었는지(로그·응답 메시지용).
        self._file_order_label = '파일에 적힌 순서'

        self._camera_frame_id = None
        self._latest_candidates = []  # [(class_id, x, y, z, confidence, radius), ...]
        self._queue = []  # [(Point(g_base), class_id, confidence, original_depth), ...]
        self._waiting_for_result = False
        self._next_target_timer = None
        self._all_targets = []

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
        # [2026-08-03] 큐에 든 목표를 **구 마커**로 그린다.
        #
        # 왜 필요한가: 실물 세션의 RViz에는 지금 무엇이 "토마토"인지 보여주는
        # 것이 없다. octomap은 열매를 일부러 지우고(따려는 열매 자신이 장애물이
        # 되면 안 되므로 — 핸드오프 함정 11) coord_to_goal_node가 씬에 넣는
        # 구는 **지금 목표 하나**(target_object)뿐이다. 그래서 화면에서 열매
        # 자리가 통째로 비어 보인다.
        #
        # 마스킹 안 한 포인트클라우드를 깔아 보는 방법도 있는데(실물 영상),
        # 팔·구·voxel을 오히려 가려서 안 쓰기로 했다(2026-08-03). 대신 검출
        # 결과를 구로 그린다 — 스윕 화면과 같은 그림이다.
        #
        # **표시 전용이다.** collision object가 아니므로 플래닝에 영향이 없다.
        self._marker_publisher = self.create_publisher(
            MarkerArray, 'harvest_targets', 10
        )
        # [2026-08-02] 목표마다 그 목표의 추정 반지름을 같이 낸다
        # (_publish_next_target 참고). 예전엔 YOLO가 최적 목표 하나에 대해서만
        # 냈고, 그래서 2번째 이후 목표의 그리퍼 폭이 남의 값이었다.
        self._radius_publisher = self.create_publisher(
            Float32, 'target_radius_m', 10
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
        # [2026-08-02] 시퀀스가 도는 동안 YOLO 판단을 얼린다.
        #
        # 이 노드는 시작 시점 스냅샷 하나로 끝까지 가므로 큐 자체는 오염되지
        # 않는다. 문제는 **큐와 무관하게** yolo_d435_detector_node가 판단을
        # 내는 것이다 — 판단이 열리는 시점이 사이클마다 다르다:
        #   LPC  목표마다 look pose로 복귀 -> 매 수확마다 열린다
        #   ASC/BSC  중간엔 안 열리고, 시퀀스 끝의 /go_to_look_pose에서 열린다
        # publish_target_point가 기본 false라 팔이 바로 끌려가지는 않지만,
        # 얼려 두면 (1) 진행 중 tomato_candidates가 스냅샷과 다른 값으로 덮이는
        # 혼란과 (2) 그 파라미터를 true로 켠 세션의 자동 이동을 둘 다 막는다.
        self._judgment_client = self.create_client(
            SetBool, '/yolo_d435_detector_node/set_judgment_enabled'
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
            class_id, x, y, z, confidence, radius_m, _count = data[i:i + stride]
            # [2026-08-02] radius_m를 여기서 같이 들고 온다. 예전엔 앞의 5개만
            # 읽었는데, 그러면 목표마다 그리퍼 폭을 정할 근거가 이 노드에 없다.
            parsed.append((int(class_id), x, y, z, confidence, radius_m))
        self._latest_candidates = parsed

    def _on_start_harvest(self, request, response) -> Trigger.Response:
        if self._waiting_for_result or self._queue:
            response.success = False
            response.message = '이미 수확 시퀀스 진행 중 — 완료 후 다시 호출할 것'
            return response

        if self._target_source == 'file':
            queue, error = self._queue_from_file()
        else:
            queue, error = self._queue_from_yolo()
        if error:
            response.success = False
            response.message = error
            return response

        self._queue = queue
        self._all_targets = list(queue)      # 마커용 원본(처리해도 안 지운다)
        self._publish_markers()
        # 시퀀스가 도는 동안 YOLO 판단을 얼린다(그 클라이언트 주석 참고).
        self._set_judgment(False)

        order = ('카메라 깊이 오름차순' if self._target_source == 'yolo'
                 else self._file_order_label)
        response.success = True
        response.message = (f'{len(queue)}개 대상 큐잉({self._target_source}), '
                            f'{order}으로 순차 접근 시작')
        self.get_logger().info(response.message)
        self._publish_next_target()
        return response

    def _queue_from_yolo(self):
        """look pose 스냅샷 -> 큐. (queue, error) 를 돌려준다."""
        if not self._latest_candidates:
            return None, 'tomato_candidates가 비어있음(검출된 ripe/disease 없음)'
        if self._camera_frame_id is None:
            return None, 'camera_info를 아직 못 받음(frame_id 미확인)'

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
            return None, (f'TF2 변환 실패({self._camera_frame_id} -> '
                          f'{BASE_LINK_NAME}): {exc}')

        # 스냅샷 즉시 전부 g_base로 변환 — 시퀀스 진행 중 팔이 움직여도(camera
        # 프레임 자체가 바뀌어도) 이후 목표 좌표가 흔들리지 않도록 함(위 모듈
        # docstring 참고).
        queue = []
        for class_id, x, y, z, confidence, radius_m in self._latest_candidates:
            camera_point = PointStamped()
            camera_point.header.frame_id = self._camera_frame_id
            camera_point.point = Point(x=x, y=y, z=z)
            base_point = do_transform_point(camera_point, transform)
            queue.append({
                'point': base_point.point,
                'class_name': NAME_BY_CLASS_ID.get(class_id, f'class_{class_id}'),
                'confidence': confidence,
                'depth': z,
                'radius_m': radius_m or DEFAULT_TARGET_RADIUS_M,
            })

        # 원본 카메라 프레임 z(카메라로부터의 깊이) 오름차순 — 가까운 것부터.
        queue.sort(key=lambda item: item['depth'])
        return queue, None

    def _queue_from_file(self):
        """사람이 보정한 좌표 파일 -> 큐. (queue, error) 를 돌려준다.

        형식은 scripts/export_detections.py의 출력과 같다:
            {"detections": [{"base_x":..,"base_y":..,"base_z":..,
                             "radius_m":.., "class_name":"ripe", "camera_z":..}, ...]}

        **조용히 빈 큐로 시작하지 않는다** — 파일이 없거나 키가 어긋나면 그
        이유를 그대로 돌려준다. 빈 큐로 시작하면 "시퀀스가 즉시 끝났다"로 보여
        원인을 찾기 어렵다.
        """
        path = self._targets_file
        if not path:
            return None, ('target_source:=file인데 targets_file 파라미터가 '
                          '비어 있음')
        if not os.path.exists(path):
            return None, f'좌표 파일 없음: {path}'
        try:
            with open(path, encoding='utf-8') as fh:
                dets = json.load(fh)['detections']
        except (ValueError, KeyError, OSError) as exc:
            return None, f'좌표 파일을 읽을 수 없음({path}): {exc}'
        if not dets:
            return None, f'좌표 파일에 detections가 비어 있음: {path}'

        queue = []
        for i, d in enumerate(dets, 1):
            try:
                point = Point(x=float(d['base_x']), y=float(d['base_y']),
                              z=float(d['base_z']))
            except (KeyError, TypeError, ValueError) as exc:
                return None, (f'{path}의 {i}번째 항목에 base_x/base_y/base_z가 '
                              f'없거나 숫자가 아님: {exc}')
            queue.append({
                'point': point,
                'class_name': d.get('class_name', 'manual'),
                'confidence': float(d.get('confidence', 1.0)),
                'depth': d.get('camera_z'),
                'radius_m': float(d.get('radius_m') or DEFAULT_TARGET_RADIUS_M),
            })

        # camera_z가 있으면 YOLO 경로와 **같은 규칙**(깊이 오름차순)으로 정렬해
        # 두 경로를 비교할 수 있게 한다. 없으면 파일에 적힌 순서가 곧 의도다.
        if all(item['depth'] is not None for item in queue):
            queue.sort(key=lambda item: item['depth'])
            self._file_order_label = '카메라 깊이 오름차순'
        else:
            for item in queue:
                item['depth'] = None
            self._file_order_label = '파일에 적힌 순서'
        self.get_logger().info(
            f'좌표 파일 {path}에서 {len(queue)}개 로드 '
            f'({self._file_order_label}).')
        return queue, None

    # 마커 색: 대기=빨강, 지금 목표=노랑(크게), 끝난 것=회색.
    MARKER_COLORS = {
        'queued': (0.85, 0.15, 0.15, 0.75),
        'current': (1.00, 0.85, 0.10, 1.00),
        'done': (0.45, 0.45, 0.45, 0.45),
    }

    def _publish_markers(self, current=None) -> None:
        """큐에 들어온 목표 전체를 구로 그린다(표시 전용).

        `_all_targets`는 시퀀스 시작 시점의 전체 목록이고 `_queue`는 남은 것이다.
        둘을 비교해 이미 처리한 것과 남은 것을 색으로 나눈다 — 영상에서
        "몇 개 중 몇 번째"가 보여야 의미가 있다.
        """
        array = MarkerArray()
        remaining = {id(item) for item in self._queue}
        for i, item in enumerate(self._all_targets):
            m = Marker()
            m.header.frame_id = BASE_LINK_NAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'harvest_targets'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = item['point']
            m.pose.orientation.w = 1.0
            if item is current:
                state = 'current'
            elif id(item) in remaining:
                state = 'queued'
            else:
                state = 'done'
            r = float(item['radius_m'])
            scale = 2.0 * r * (1.6 if state == 'current' else 1.0)
            m.scale.x = m.scale.y = m.scale.z = scale
            cr, cg, cb, ca = self.MARKER_COLORS[state]
            m.color = ColorRGBA(r=cr, g=cg, b=cb, a=ca)
            array.markers.append(m)
        self._marker_publisher.publish(array)

    def _publish_next_target(self) -> None:
        if not self._queue:
            self.get_logger().info('수확 시퀀스 완료 — 큐 비어있음.')
            self._return_to_look_pose()
            return

        item = self._queue.pop(0)
        point = item['point']
        self._publish_markers(current=item)

        # [2026-08-02] **반지름을 목표보다 먼저 발행한다.**
        # coord_to_goal_node는 target_point를 받는 순간의 캐시값을 그 사이클용으로
        # 스냅샷하므로 순서가 곧 의미다(그 파일 _pending_target_radius_m).
        #
        # 예전엔 yolo_d435_detector_node가 **최적 목표 하나**에 대해서만 이 값을
        # 냈다. 그래서 시퀀스의 2번째 이후 목표는 **다른 열매의 반지름**으로
        # 그리퍼를 열고 있었다. 이제 목표마다 그 목표의 값을 낸다.
        radius_msg = Float32()
        radius_msg.data = float(item['radius_m'])
        self._radius_publisher.publish(radius_msg)

        msg = PointStamped()
        msg.header.frame_id = BASE_LINK_NAME
        msg.point = point
        self._waiting_for_result = True
        self._target_publisher.publish(msg)
        depth_txt = ('' if item['depth'] is None
                     else f", 원본 깊이={item['depth']:.3f}m")
        self.get_logger().info(
            f"다음 목표 발행: {item['class_name']}(conf={item['confidence']:.2f}"
            f"{depth_txt}, 반지름={item['radius_m'] * 1000:.1f}mm) -> g_base "
            f'[{point.x:.3f}, {point.y:.3f}, {point.z:.3f}] '
            f'(남은 큐: {len(self._queue)}개)'
        )

    def _set_judgment(self, enabled: bool) -> None:
        """YOLO 판단을 켜고 끈다. 서비스가 없으면 경고만 하고 진행한다 —
        YOLO 없이 도는 세션(재생 스택·FakeSystem 수동 테스트)이 있다."""
        if not self._judgment_client.service_is_ready():
            self.get_logger().warn(
                'YOLO 판단 스위치(set_judgment_enabled)가 없어 건너뜀 — '
                'yolo_d435_detector_node가 떠 있다면 수확 중에도 판단이 열린다.'
            )
            return
        request = SetBool.Request()
        request.data = enabled
        self._judgment_client.call_async(request)
        self.get_logger().info(
            f'YOLO 판단 {"해제" if enabled else "얼림"} 요청.')

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
        # 복귀를 **요청한 뒤** 판단을 푼다 — 팔이 look pose에 들어오는 순간
        # 누적이 시작되도록. 먼저 풀어도 결과는 같지만(누적 시작점은 도착
        # 시점이다) 순서를 이렇게 두면 로그가 사이클 순서대로 읽힌다.
        self._set_judgment(True)

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
