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

  - <color_topic> (sensor_msgs/msg/Image, publish_rgb=true일 때만): 컬러
    이미지(bgr8). 기본값: /camera/camera/color/image_raw. depth가
    aligned_depth_to_color라 이 컬러 이미지와 픽셀이 1:1로 대응하므로
    재투영 없이 인덱스 그대로 색을 입힘. 별도 시간 동기화 없이 "가장 최근
    프레임"을 캐시해서 씀 — 색은 시각화 전용이고 xyz에는 영향이 없어서,
    카메라 지터(실측 최대 0.3~0.4초, yolo_d435_detector_node 주석 참고)로
    한두 프레임 어긋나도 무해하기 때문(동기화를 붙이면 그만큼 CPU와 지연이
    늘어남).

파라미터(이어서):
  - publish_rgb (기본값 True): 출력 pointcloud에 rgb 필드를 포함할지.
    occupancy_map_monitor는 x/y/z만 쓰므로 순전히 RViz 육안 확인용이며,
    포인트당 12바이트 -> 16바이트로 대역폭이 33% 늘어남. RViz에서 이 클라우드를
    RGB8로 볼 필요가 없으면 false로 꺼서 부하를 줄일 것.
  - publish_rate_hz (기본값 5.0): 출력 발행 주파수 상한. 0 이하면 제한
    없음(depth 프레임마다, 보통 30Hz). depth 콜백마다 640x480 전체를 역투영해
    reliable로 내보내는 비용이 이 노드의 거의 전부라, 이 상한이 곧 이 노드의
    CPU 사용량을 결정함 — 30Hz로 두면 같은 카메라 스트림을 구독하는
    yolo_d435_detector_node의 콜백이 밀려서 tomato_detections_image가 끊김
    (2026-07-31 실물 세션에서 관측). Octomap voxel 갱신은 팔 이동 속도에 비해
    5Hz면 충분함.

출력:
  - <output_topic> (sensor_msgs/msg/PointCloud2): x/y/z(+ publish_rgb면 rgb)
    필드를 가진 필터링된 pointcloud, frame_id는 depth_topic/camera_info_topic과
    동일(보통 camera_color_optical_frame). 기본값:
    /camera/camera/depth/color/points_filtered. sensors_3d.yaml의
    point_cloud_topic을 이 토픽으로 바꿔서 occupancy_map_monitor가 raw 대신
    이걸 구독하게 해야 실제로 적용됨.
"""

import math

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool

from mycobot_280_pick import look_pose

DEFAULT_DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
DEFAULT_CAMERA_INFO_TOPIC = '/camera/camera/aligned_depth_to_color/camera_info'
DEFAULT_COLOR_TOPIC = '/camera/camera/color/image_raw'
DEFAULT_OUTPUT_TOPIC = '/camera/camera/depth/color/points_filtered'
DEFAULT_BOXES_TOPIC = 'tomato_boxes'

# [2026-07-31] 출력에 rgb 필드를 넣을지. occupancy_map_monitor는 x/y/z만 읽으므로
# 순수하게 RViz 육안 확인용임(rgb 필드가 없으면 RViz PointCloud2 디스플레이의
# Color Transformer에 RGB8 항목 자체가 안 뜸 — 실제로 그 문의를 받아 추가함).
# 켜면 포인트당 12->16바이트라 대역폭이 33% 늘어남.
DEFAULT_PUBLISH_RGB = True

# [2026-07-31] 출력 발행 주파수 상한(Hz). 0 이하면 제한 없음(depth 프레임마다 발행).
# 배경: 이 노드는 depth 콜백마다 640x480 전체를 numpy로 역투영하고 그 결과를
# reliable QoS로 직렬화해 내보냄 — 30Hz면 rgb 포함 초당 약 147MB(4.9MB x 30)라
# 이 노드 하나가 CPU를 크게 점유하고, 같은 카메라 스트림을 구독하는
# yolo_d435_detector_node의 콜백이 밀려서 RViz의 tomato_detections_image가
# 뚝뚝 끊기는 게 실물 세션에서 관측됨(2026-07-31 사용자 보고). Octomap voxel
# 갱신은 30Hz가 필요한 작업이 아니라서(팔이 그보다 훨씬 느리게 움직임) 상한을
# 두는 쪽이 이득이 훨씬 큼. 초과분은 numpy 연산 전에 조기 반환해서 버림.
DEFAULT_PUBLISH_RATE_HZ = 5.0

# [2026-07-31] rosbag 재생 전용 옵션. `ros2 bag play`는 메시지의 header.stamp를
# **녹화 당시 값 그대로** 내보내는데, TF는 살아있는 스택이 현재 시각으로 발행한다.
# 그래서 재생본으로 만든 클라우드는 stamp가 수십 분 과거가 되고, tf2 버퍼(기본
# 10초)를 벗어나 소비자들이 변환을 못 한다 — RViz PointCloud2가 Error가 되면서
# Position/Color Transformer가 빈 채로 남고(색 문제로 오해하기 쉽다),
# occupancy_map_monitor도 octomap을 못 쌓는다. 반면 Image 디스플레이는 TF가
# 필요 없어서 멀쩡히 보이므로 증상이 더 헷갈린다. 실측: 재생 중 stamp가 현재보다
# 1789초 과거였다.
#
# 정석은 `ros2 bag play --clock` + 모든 노드 use_sim_time이지만, 그러려면
# move_group과 FakeSystem 컨트롤러까지 전부 sim time으로 재시작해야 한다.
# 장면이 정지해 있고 팔이 녹화 당시 자세(look pose)에 그대로 있는 재생
# 시나리오라면 출력 stamp를 현재로 바꾸는 것으로 충분하다.
#
# **주의**: 팔이 움직이는 중에 이걸 켜면 안 된다. 옛 장면이 새 TF로 변환되어
# 엉뚱한 위치에 놓인다(eye-in-hand 제약, docs/ROSBAG_HANDOFF.md 5절).
DEFAULT_RESTAMP_NOW = False

# YOLO bbox는 물체 경계에 딱 맞게(타이트하게) 잡히는 경우가 많고, depth-color
# 정렬도 픽셀 단위로 약간의 슬랙이 있을 수 있어서, bbox 그대로만 마스킹하면
# 테두리 voxel이 안 지워지고 남을 수 있음. bbox 각 변을 이 비율만큼 바깥으로
# 확장해서 여유를 둠(비율 기반이라 거리가 멀어져 bbox가 작아져도 상대적
# 여유는 유지됨 — 픽셀 고정값보다 이게 더 일관적).
# DEFAULT_BBOX_PADDING_RATIO = 0.2
DEFAULT_BBOX_PADDING_RATIO = 0.5

# occupancy_map_monitor가 필요로 하는 최소 구성(x/y/z).
_XYZ_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
_XYZ_POINT_STEP = 12

# publish_rgb=true일 때의 구성. rgb는 PCL/RViz 관례대로 "FLOAT32 슬롯에 packed
# uint32(0x00RRGGBB)의 비트를 그대로 담는" 형태 — 값 자체는 float으로 해석하면
# 의미가 없고, 소비자(RViz Color Transformer=RGB8)가 다시 비트로 읽음.
_XYZRGB_FIELDS = _XYZ_FIELDS + [
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]
_XYZRGB_POINT_STEP = 16


class PointcloudTomatoFilterNode(Node):

    def __init__(self):
        super().__init__('pointcloud_tomato_filter_node')

        self.declare_parameter('depth_topic', DEFAULT_DEPTH_TOPIC)
        self.declare_parameter('camera_info_topic', DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter('color_topic', DEFAULT_COLOR_TOPIC)
        self.declare_parameter('output_topic', DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter('boxes_topic', DEFAULT_BOXES_TOPIC)
        self.declare_parameter('bbox_padding_ratio', DEFAULT_BBOX_PADDING_RATIO)
        self.declare_parameter('publish_rgb', DEFAULT_PUBLISH_RGB)
        self.declare_parameter('publish_rate_hz', DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter('restamp_now', DEFAULT_RESTAMP_NOW)
        # [2026-08-03] look pose 게이트.
        #
        # 카메라가 eye-in-hand(joint6)라, 팔이 움직이는 동안에도 클라우드를 계속
        # 내면 occupancy_map_monitor가 **팔이 있는 자리에서 본 것**을 계속 합치고
        # 시야를 따라 지운다. 화면에서는 수확 중 장면이 쉬지 않고 요동친다는
        # 뜻이고, 실제로 "무엇을 보고 있는지 헷갈린다"는 지적이 나왔다.
        #
        # true면 팔이 look pose 근처에 있을 때만 발행한다 — 관측은 look pose에서
        # 한 번이면 충분하다는 이 팔의 look-then-move 구조와 같은 전제다
        # (yolo_d435_detector_node의 판단 게이팅과 같은 판정).
        #
        # 기본은 false다. scene_replay 세션은 팔이 look pose에 없을 수도 있는데
        # 거기서 켜지면 클라우드가 아예 안 나온다.
        # 화면 하단에서 버릴 행 수(0이면 안 자름). 카메라가 자기 그리퍼를
        # 보는 문제 대응 — 자세한 근거는 _on_depth의 crop 주석.
        #
        # [2026-08-03, 실물] 480행 기준 72 = 아래 15%에서 **그리퍼 voxel이
        # 깔끔히 사라지고 베드는 그대로 남는다**(RViz 육안 확인). 10%(48)로는
        # 부족했다. 해상도가 바뀌면 비율로 환산해 다시 잡을 것.
        self.declare_parameter('crop_bottom_rows', 72)
        self.declare_parameter('look_pose_gate', False)
        self.declare_parameter('look_pose_tolerance_rad', 0.08)

        depth_topic = (
            self.get_parameter('depth_topic').get_parameter_value().string_value
        )
        color_topic = (
            self.get_parameter('color_topic').get_parameter_value().string_value
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
        self._publish_rgb = (
            self.get_parameter('publish_rgb').get_parameter_value().bool_value
        )
        publish_rate_hz = (
            self.get_parameter('publish_rate_hz')
            .get_parameter_value()
            .double_value
        )
        # 발행 간 최소 간격(초). 0이면 제한 없음.
        self._min_publish_period_s = (
            1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.0
        )
        self._last_publish_time = None
        self._restamp_now = (
            self.get_parameter('restamp_now').get_parameter_value().bool_value
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
        # publish_rgb=true일 때 가장 최근 컬러 프레임(bgr8, HxWx3). depth와
        # 시간 동기화하지 않고 최신 것을 그대로 씀(모듈 docstring 참고).
        self._latest_color = None

        # occupancy_map_monitor(moveit PointCloudOctomapUpdater)가 RELIABLE로
        # 구독을 요청함(2026-07-24 실물 테스트로 확인 — "incompatible QoS ...
        # Last incompatible policy: RELIABILITY" 경고 발생, best-effort로는
        # 메시지가 전혀 전달 안 됨). 그래서 기본 QoS(reliable, keep-last)를 씀.
        self._look_pose_tolerance = (
            self.get_parameter('look_pose_tolerance_rad')
            .get_parameter_value().double_value
        ) or 0.08
        self._at_look_pose = False
        # **구독은 항상 걸어 둔다.** 게이트를 실행 중에 켤 수 있어야 하는데
        # (이 노드는 demo_octomap.launch.py가 띄우므로 런치 인자를 고치려면
        # vendor 파일을 건드려야 한다), 그때 가서 구독을 만들면 첫 판정까지
        # 공백이 생긴다. /joint_states는 가벼우니 항상 받는다.
        self.create_subscription(JointState, 'joint_states',
                                 self._on_joint_states, 10)

        # [2026-08-03] 수확 시퀀스 동안 클라우드 발행을 통째로 멈추는 스위치.
        # harvest_sequence_node가 YOLO 판단 얼림(set_judgment_enabled)과 **같이**
        # 부른다 — 얼리는 대상이 다를 뿐 목적은 같다("수확 중에는 씬을 건드리지
        # 않는다"). 이유는 _on_depth의 주석 참고.
        #
        # 이름이 set_cloud_enabled(True=발행)인데 내부 플래그는 _sequence_frozen
        # (True=정지)으로 반대인 것은, 서비스 쪽은 판단 스위치와 같은 관례를
        # 따르고(enabled=정상) 코드 쪽은 "얼었는가"로 읽는 게 자연스러워서다.
        #
        # 이름 앞의 `~/`는 **필수**다. ROS 2에서 상대 이름은 노드 이름이 아니라
        # 네임스페이스 기준으로 풀리므로, 'set_cloud_enabled'로 만들면
        # /set_cloud_enabled가 되어 클라이언트가 찾는
        # /pointcloud_tomato_filter_node/set_cloud_enabled와 안 맞는다
        # (2026-08-03에 실제로 이 실수로 스위치가 조용히 건너뛰어졌다).
        # yolo_d435_detector_node의 '~/set_judgment_enabled'와 같은 관례다.
        self._sequence_frozen = False
        self.create_service(SetBool, '~/set_cloud_enabled',
                            self._on_set_cloud_enabled)

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
        # rgb를 안 쓸 거면 컬러 이미지 구독 자체를 만들지 않음 — 640x480 bgr8
        # 프레임을 30Hz로 역직렬화하는 비용이 그대로 들기 때문.
        self._color_sub = (
            self.create_subscription(Image, color_topic, self._on_color, 10)
            if self._publish_rgb
            else None
        )

        self.get_logger().info(
            f'pointcloud_tomato_filter_node 준비 완료. {depth_topic} -> '
            f'{output_topic} (bbox 구독: {boxes_topic}, rgb: '
            f'{"on, " + color_topic if self._publish_rgb else "off"}, '
            f'발행 상한: '
            f'{f"{publish_rate_hz:g}Hz" if self._min_publish_period_s > 0.0 else "없음"}'
            f'{", restamp_now(재생용)" if self._restamp_now else ""})'
        )

    def _crop_bottom_rows(self) -> int:
        """화면 하단에서 버릴 행 수. 파라미터를 **매번 다시 읽는다** —
        RViz를 보며 `ros2 param set`으로 맞추는 값이라 재시작 없이 반영돼야
        한다(look_pose_gate와 같은 이유)."""
        try:
            return max(0, int(self.get_parameter('crop_bottom_rows').value))
        except Exception:
            return 0

    def _on_set_cloud_enabled(self, request, response):
        """수확 시퀀스 동안 클라우드 발행을 멈추고/재개한다.

        멈추면 octomap이 갱신되지 않으므로, 시퀀스가 시작 직후 부른
        /clear_octomap 상태(빈 씬)가 시퀀스 내내 유지된다. 계획과 실행이
        같은 씬을 보게 되는 것이 목적이다.
        """
        self._sequence_frozen = not request.data
        response.success = True
        response.message = ('클라우드 발행 재개' if request.data
                            else '클라우드 발행 정지 — 수확 중 씬 고정')
        self.get_logger().info(response.message)
        return response

    def _gate_enabled(self) -> bool:
        """look_pose_gate 파라미터를 **매번 다시 읽는다** — 실행 중에
        `ros2 param set /pointcloud_tomato_filter_node look_pose_gate true`로
        켜고 끌 수 있어야 한다."""
        try:
            return bool(self.get_parameter('look_pose_gate').value)
        except Exception:
            return False

    def _on_joint_states(self, msg: JointState) -> None:
        at = look_pose.is_near_look_pose(dict(zip(msg.name, msg.position)),
                                         self._look_pose_tolerance)
        if at != self._at_look_pose:
            self._at_look_pose = at
            if self._gate_enabled():
                self.get_logger().info(
                    'look pose 도착 — 클라우드 발행 재개' if at
                    else 'look pose 이탈 — 클라우드 발행 중지(장면 고정)')

    def _on_color(self, msg: Image) -> None:
        self._latest_color = self._bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8'
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

        # [2026-08-03, 실물] 수확 시퀀스가 도는 동안은 아예 발행하지 않는다.
        #
        # look_pose_gate만으로는 부족하다는 것이 실물에서 확인됐다. 게이트는
        # "look pose에 있을 때만 낸다"인데 [0/5] 경유가 **바로 그 look pose에서
        # 출발**하므로, 출발 시점의 씬이 계속 갱신된다. /clear_octomap을 사이클
        # 시작으로 옮겨도 소용없었다 — 클리어 1.2초 뒤 플래닝인데 그 사이
        # (센서 1Hz) 한 프레임이 들어와 joint3/joint4를 덮었다. 계획이 통과해도
        # 실행 중에 "path became invalid (environment changed)"로 깨졌다.
        #
        # 성공했던 실행들은 타이밍이 맞았을 뿐이고 재현성이 없었다. 씬을 아예
        # 고정해야 계획도 실행도 같은 세계를 본다.
        #
        # 대가: 수확 중에는 장애물(줄기·지지대) 회피가 없다. 이건 핸드오프
        # 5절 5번("octomap 회피를 켤 것인가")의 실측 근거 위에서 내린 선택이다 —
        # 회피를 켠 상태의 성공률이 100 -> 77%였고, 오늘 실패는 전부 여기서 나왔다.
        if self._sequence_frozen:
            return

        # look pose 게이트(위 파라미터 설명). 역투영 **앞에서** 잘라야 CPU도
        # 같이 아낀다.
        if self._gate_enabled() and not self._at_look_pose:
            return

        # 주파수 상한 초과분은 여기서 버림 — 아래 numpy 역투영/직렬화가 이 노드
        # 비용의 대부분이라, 반드시 그 앞에서 잘라야 의미가 있음. 벽시계(ROS
        # clock) 기준이고 depth_msg.header.stamp를 안 쓰는 이유는, 여기서 줄이려는
        # 게 "센서 시각"이 아니라 "이 프로세스가 실제로 CPU를 쓰는 빈도"이기
        # 때문(스트림이 밀려 stamp가 뭉쳐 들어와도 실제 처리량은 제한돼야 함).
        if self._min_publish_period_s > 0.0:
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if (
                self._last_publish_time is not None
                and now_s - self._last_publish_time < self._min_publish_period_s
            ):
                return
            self._last_publish_time = now_s

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

        # [2026-08-03] 화면 아래쪽 잘라내기 — 카메라가 **자기 그리퍼**를 본다.
        #
        # eye-in-hand라 그리퍼 끝이 화면 하단에 걸린다. 그게 octomap voxel이
        # 되어 팔 자신과 충돌 판정되고, 긴 전이가 죽는다. self-filter의
        # padding으로 덮으려 했으나 실패했다 — padding_scale은 비례라 얇은
        # 손가락은 안 덮이고 큰 링크만 부풀어 실제 물체가 지워졌고,
        # padding_offset(절대값)은 베드 전체를 지워버렸다(sensors_3d.yaml 주석).
        # 안 보이게 만드는 쪽이 지우려 애쓰는 쪽보다 확실하다.
        #
        # **핸드아이 캘리브레이션은 다시 안 해도 된다.** 픽셀을 버리는 것은
        # joint6 -> camera_link 변환을 안 건드린다. 다만 이미지 **크기를 줄이면
        # 안 된다** — 주점(cx, cy)이 이동해 역투영이 틀어진다. 그래서 원본
        # 크기를 유지한 채 해당 행만 NaN으로 만든다(픽셀 좌표가 그대로 유지됨).
        #
        # YOLO 검출에는 영향이 없다 — 이 노드는 octomap용 클라우드만 만들고,
        # yolo_d435_detector_node는 컬러/depth 원본을 따로 본다.
        #
        # 값은 RViz로 보며 맞출 것(실행 중 ros2 param set으로 조정 가능):
        #   그리퍼가 voxel에서 사라지는 최소값을 쓴다. 크게 잡을수록 베드
        #   아래쪽의 진짜 장애물(줄기 밑동)도 같이 사라진다.
        crop = self._crop_bottom_rows()
        if crop > 0:
            xs[height - crop:, :] = math.nan
            ys[height - crop:, :] = math.nan
            zs[height - crop:, :] = math.nan

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

        color = self._latest_color if self._publish_rgb else None
        # 컬러가 아직 안 왔거나(구동 직후) 해상도가 다르면(런치 인자를 서로
        # 다르게 준 경우) 색만 포기하고 검은색으로 채움 — 필드 구성을 프레임마다
        # 바꾸면 RViz/구독자 쪽이 혼란스러워지므로 레이아웃은 항상 고정함.
        channels = 4 if self._publish_rgb else 3
        stacked = np.empty((height, width, channels), dtype=np.float32)
        stacked[..., 0] = xs
        stacked[..., 1] = ys
        stacked[..., 2] = zs

        if self._publish_rgb:
            if color is not None and color.shape[:2] == (height, width):
                # bgr8 -> 0x00RRGGBB. depth가 aligned_depth_to_color라 컬러
                # 이미지와 픽셀이 1:1이므로 재투영 없이 인덱스 그대로 대응됨.
                packed = (
                    color[..., 2].astype(np.uint32) << 16
                    | color[..., 1].astype(np.uint32) << 8
                    | color[..., 0].astype(np.uint32)
                )
            else:
                packed = np.zeros((height, width), dtype=np.uint32)
            # 비트를 그대로 float32 슬롯에 옮김(값 변환이 아니라 재해석).
            stacked[..., 3] = packed.view(np.float32)

        cloud = PointCloud2()
        # header를 통째로 대입하면 입력 메시지의 header 객체를 그대로 참조하게
        # 되어, 아래에서 stamp를 바꿀 때 남의 메시지를 건드리게 된다. 필드별로 씀.
        cloud.header.frame_id = depth_msg.header.frame_id
        cloud.header.stamp = (
            self.get_clock().now().to_msg()
            if self._restamp_now
            else depth_msg.header.stamp
        )
        cloud.height = height
        cloud.width = width
        cloud.fields = _XYZRGB_FIELDS if self._publish_rgb else _XYZ_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = (
            _XYZRGB_POINT_STEP if self._publish_rgb else _XYZ_POINT_STEP
        )
        cloud.row_step = cloud.point_step * width
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
