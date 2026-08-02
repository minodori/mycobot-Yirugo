#!/usr/bin/env python3
"""토마토 베드 장면 rosbag **재생**용 런치 (트랙 A). 로봇도 카메라도 필요 없다.

무엇을 띄우는가 (터미널 1개로 6개짜리 수동 절차를 대체한다)
------------------------------------------------------------
  demo_octomap.launch.py    enable_camera:=false, enable_pointcloud_filter:=false,
                            use_rviz:=false, enable_octomap:=true
                            -> move_group·컨트롤러·TF·occupancy_map_monitor만 얻는다
  republish x2              bag의 압축본을 raw로 되돌린다 (필터/YOLO는 raw를 구독)
  필터 노드 x2              restamp_now:=true. 마스킹본(octomap 입력) + 비마스킹본(육안 확인)
  yolo_d435_detector_node   target_point 리맵. 재생본에서 bbox를 재생성
  ros2 bag play             카메라 토픽 + /tf_static 만
  /clear_octomap            clear_delay초 뒤 1회
  rviz2                     scene_replay.rviz

이 런치가 대신 막아 주는 함정들 (전부 2026-07-31에 실제로 밟았다)
-----------------------------------------------------------------
  1. republish의 in_transport는 **파라미터**다. 위치 인자로 주면 조용히 raw로 잡혀
     아무것도 안 나온다(에러도 없다).
  2. 재생 토픽에 `/tf_static`이 빠지면 camera_*_optical_frame이 없어서 클라우드가
     안 그려진다 — 그 static TF는 realsense 드라이버가 발행하던 것이다.
  3. `/tomato_boxes`를 재생하면 살아있는 YOLO와 발행자가 겹친다. 빼야 한다.
  4. `/joint_states`·`/tf`도 재생하면 안 된다(발행자 이중화 → sync_plan이 붙어 있으면
     실물이 움직인다). 녹화 중 팔은 look pose에 정지해 있었으므로 재생할 이유도 없다.
  5. bag은 header.stamp를 녹화 당시 값 그대로 내보낸다. 실측 1788.8초 과거였고,
     tf2 버퍼(10초)를 벗어나 RViz PointCloud2가 Error(Color Transformer가 빈칸으로
     남아 색 문제로 오해하기 쉽다)가 되고 octomap도 안 쌓인다 -> restamp_now:=true.
  6. demo_octomap이 필터 노드를 자동으로 띄워 발행자가 둘이 된다
     -> enable_pointcloud_filter:=false.
  7. 마스킹은 사전 차단이지 사후 삭제가 아니다. 이미 들어간 토마토 voxel은 그 자리로
     가는 광선이 없어 영원히 안 지워진다 -> 다 뜬 뒤 /clear_octomap 1회.

사용법
------
    ros2 launch mycobot_280_pick scene_replay.launch.py
    ros2 launch mycobot_280_pick scene_replay.launch.py bag:=bags/bed_0801 rate:=0.5

주의: 실물 스택이나 sync_plan이 떠 있으면 안 된다(docs/ROSBAG_HANDOFF.md 2절 함정 1).
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

COLOR_COMPRESSED = '/camera/camera/color/image_raw/compressed'
COLOR_RAW = '/camera/camera/color/image_raw'
DEPTH_COMPRESSED = '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth'
DEPTH_RAW = '/camera/camera/aligned_depth_to_color/image_raw'

# 재생할 토픽 — 위 함정 2~4의 결론이 그대로 목록이 된다.
PLAY_TOPICS = [
    COLOR_COMPRESSED,
    DEPTH_COMPRESSED,
    '/camera/camera/color/camera_info',
    '/camera/camera/aligned_depth_to_color/camera_info',
    '/tf_static',
]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('bag', default_value='bags/bed_look_slim'))
    ld.add_action(DeclareLaunchArgument('rate', default_value='1.0'))
    ld.add_action(DeclareLaunchArgument('loop', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_yolo', default_value='true',
                                        description='false면 검출 재생성을 건너뜀(CPU 절약)'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true'))
    # [2026-08-01] 필터 노드의 기본값은 0.5인데, docs/ROSBAG_HANDOFF.md 6.5절이
    # 그 값이 과하다고 실측해 두었다(마스킹 면적 0.0→11.8% / 0.2→21.7% /
    # 0.5→31.5%). 넓게 지우면 토마토뿐 아니라 **줄기·지지대까지 octomap에서
    # 사라진다** — 그것들은 실제로 피해야 할 장애물이다.
    #
    # 이 인자가 필요한 이유는 octomap 재검증(docs/ARMED_POSE_HANDOFF.md 5절)
    # 때문이다. 노드는 이 파라미터를 기동 시 한 번만 읽으므로(ros2 param set으로는
    # 안 바뀐다) 런치에서 주는 수밖에 없다.
    #
    # **낮춘다고 더 보수적인 씬이 되는 게 아니다 — 같은 문서 함정 11.**
    # 처음엔 "0.2면 줄기가 더 남으니 실물에 가깝겠다"고 봤는데 반대였다.
    # bbox가 열매를 다 못 덮어 **열매 표면이 octomap voxel로 살아남고**, octomap
    # voxel에는 ACM 완화가 안 걸리므로 따려는 열매 자신이 회피 대상이 된다.
    # 실측: 열매 구 안 voxel이 0.5에서 0개, 0.2에서 33개(13/15 토마토).
    # 성공률 77.8% -> 19.3%. 그 19.3%는 실물이 아니라 측정 인공물이다.
    ld.add_action(DeclareLaunchArgument(
        'bbox_padding', default_value='0.5',
        description='마스킹본 필터의 bbox_padding_ratio. 낮추면 줄기가 더 남지만 '
                    '열매까지 남는다 — 낮춰 쓰기 전에 함정 11을 읽을 것'))
    ld.add_action(DeclareLaunchArgument(
        'clear_delay', default_value='25.0',
        description=(
            '/clear_octomap 자동 호출까지 대기(초). "tomato_boxes가 흐르기 시작하면"이라는 '
            '조건을 런치가 직접 감시할 수 없어 시간으로 근사한다. YOLO 모델 로드 + '
            'ultralytics 첫 추론 워밍업 + 노드 기동에 여유를 얹은 값이다.'
        )))

    # ---- 스택 (카메라·필터 노드·RViz 없이) ----
    #
    # [2026-08-01] GroupAction(scoped=True)로 감싸는 것이 **필수**다.
    # IncludeLaunchDescription의 launch_arguments는 스코프를 만들지 않아서 부모
    # 컨텍스트에 그대로 새어 들어온다. 여기서 demo_octomap에 넘기는
    # use_rviz:=false가 최상위 use_rviz까지 덮어써서, 아래 rviz2 노드의
    # IfCondition이 거짓이 되어 **RViz가 전혀 뜨지 않았다**(에러도 없이 조용히).
    # 이름이 같은 인자를 자식에게 다른 값으로 넘길 때 항상 생기는 문제다.
    ld.add_action(
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('mycobot_280_moveit2'),
                        '/launch/demo_octomap.launch.py',
                    ]),
                    launch_arguments={
                        'enable_octomap': 'true',
                        'enable_camera': 'false',
                        'enable_pointcloud_filter': 'false',
                        'use_rviz': 'false',
                    }.items(),
                )
            ],
        )
    )

    # ---- 압축본 -> raw 복원 ----
    for label, in_transport, in_topic, out_topic in (
        ('color', 'compressed', COLOR_COMPRESSED, COLOR_RAW),
        ('depth', 'compressedDepth', DEPTH_COMPRESSED, DEPTH_RAW),
    ):
        ld.add_action(
            Node(
                package='image_transport',
                executable='republish',
                name=f'republish_{label}',
                # in_transport는 반드시 파라미터로. 위치 인자로 주면 raw로 잡힌다(함정 1).
                parameters=[{'in_transport': in_transport, 'out_transport': 'raw'}],
                remappings=[(f'in/{in_transport}', in_topic), ('out', out_topic)],
            )
        )

    # ---- 클라우드 재생성 2개 ----
    # 마스킹본: occupancy_map_monitor(sensors_3d.yaml)가 구독하는 기본 출력 토픽.
    ld.add_action(
        Node(
            package='mycobot_280_pick',
            executable='pointcloud_tomato_filter_node',
            name='pointcloud_tomato_filter_node',
            parameters=[{
                'restamp_now': True,
                # LaunchConfiguration은 문자열로 풀리므로 value_type을 줘야
                # 선언된 double 파라미터와 타입이 맞는다.
                'bbox_padding_ratio': ParameterValue(
                    LaunchConfiguration('bbox_padding'), value_type=float),
            }],
        )
    )
    # 비마스킹본: "어느 voxel이 토마토인가"를 눈으로 찾는 용도. points_filtered는
    # 정의상 토마토가 지워져 있어서 이 목적에 쓸 수 없다. octomap과 토글해서 본다.
    ld.add_action(
        Node(
            package='mycobot_280_pick',
            executable='pointcloud_tomato_filter_node',
            name='pointcloud_unmasked_node',
            parameters=[{
                'restamp_now': True,
                'output_topic': '/camera/camera/depth/color/points_unmasked',
                'boxes_topic': '__no_boxes__',   # 존재하지 않는 토픽 = 마스킹 없음
                'publish_rate_hz': 2.0,          # 육안 확인용이라 낮게
            }],
        )
    )

    # ---- YOLO ----
    ld.add_action(
        Node(
            package='mycobot_280_pick',
            executable='yolo_d435_detector_node',
            name='yolo_d435_detector_node',
            output='screen',
            remappings=[('target_point', 'target_point_dryrun')],
            condition=IfCondition(LaunchConfiguration('use_yolo')),
        )
    )

    # ---- 재생 ----
    ld.add_action(
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
                 '--rate', LaunchConfiguration('rate'),
                 '--loop', '--topics'] + PLAY_TOPICS,
            output='screen',
            condition=IfCondition(LaunchConfiguration('loop')),
        )
    )
    ld.add_action(
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
                 '--rate', LaunchConfiguration('rate'),
                 '--topics'] + PLAY_TOPICS,
            output='screen',
            # IfCondition(['not ', ...])는 launch가 "not true"를 불린으로 해석하지
            # 못해 InvalidConditionExpressionError를 낸다. UnlessCondition을 쓸 것.
            condition=UnlessCondition(LaunchConfiguration('loop')),
        )
    )

    # ---- 다 뜬 뒤 octomap 1회 clear (함정 7) ----
    ld.add_action(
        TimerAction(
            period=LaunchConfiguration('clear_delay'),
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call',
                         '/clear_octomap', 'std_srvs/srv/Empty'],
                    output='screen',
                )
            ],
        )
    )

    # ---- RViz ----
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('mycobot_280_pick'), 'config', 'scene_replay.rviz',
            ])],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        )
    )

    return ld
