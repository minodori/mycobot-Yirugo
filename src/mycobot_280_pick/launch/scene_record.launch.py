#!/usr/bin/env python3
"""토마토 베드 장면 rosbag **기록**용 런치 (트랙 A). 실물 카메라가 필요하다.

무엇을 띄우는가
---------------
  demo_octomap.launch.py (enable_octomap:=true)   move_group·컨트롤러·TF·카메라·octomap
  yolo_d435_detector_node                          검출 -> tomato_boxes / tomato_candidates
  pointcloud_tomato_filter_node                    (런치가 자동으로 띄움) 마스킹된 클라우드
  [record:=true면] ros2 bag record                 scripts/record_scene.sh와 동일한 토픽 구성

무엇을 **일부러 안** 띄우는가
-----------------------------
`coord_to_goal_node`. 이게 떠 있으면 YOLO가 look pose에서 발행한 target_point를 받아
팔이 자동으로 움직인다. 카메라가 eye-in-hand라 팔이 움직이면 녹화 장면이 깨진다.
같은 이유로 YOLO의 `target_point`는 죽은 토픽으로 리맵한다(이 런치가 처리) —
`tomato_boxes`/`tomato_candidates`는 그대로 나오므로 마스킹과 좌표 추출에는 지장이 없다.

기록 순서가 중요하다
--------------------
octomap voxel은 **광선이 통과할 때만** 지워진다. 그런데 마스킹은 토마토 픽셀의 포인트를
아예 없애므로 그 방향으로 나가는 광선도 사라진다. 즉 토마토 voxel이 한 번이라도 들어가면
이후 마스킹이 아무리 잘 돼도 영원히 안 지워진다. 필터 노드는 YOLO보다 먼저 뜨고 그동안
`_boxes`가 비어 있어 **마스킹 없는 클라우드**를 먼저 내보내므로, 이 일은 거의 항상 일어난다.
그래서 이 런치는 `clear_delay`초 뒤에 `/clear_octomap`을 한 번 자동 호출한다.

사용법
------
    # 장면만 띄우고 눈으로 확인 (기록 안 함)
    ros2 launch mycobot_280_pick scene_record.launch.py

    # 30초 기록
    ros2 launch mycobot_280_pick scene_record.launch.py record:=true bag_name:=bed_0801 duration:=30

    # 무기한 기록 (Ctrl+C로 종료 — bag은 SIGINT여야 metadata.yaml이 써진다)
    ros2 launch mycobot_280_pick scene_record.launch.py record:=true duration:=0

검출 좌표(클래스·3D·크기)를 파일로 뽑으려면 이 런치와 별개로:
    python3 scripts/export_detections.py --out bags/lab_bed_detections
단, YOLO 추론이 워밍업되기 전에 look pose 누적창이 시작되면 프레임 1장만 잡힌다
(docs/ROSBAG_HANDOFF.md 6.6절). 관측 횟수가 한 자리면 재수집할 것.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# scripts/record_scene.sh의 slim 프로파일과 **같은 목록**이어야 한다.
# 클라우드(4.9MB/장)와 검출영상은 담지 않는다 — 재생 때 재생성되고, 그 둘이
# full 프로파일 용량의 46.7%였다. 압축본은 raw 대비 색 1/9.4, depth 1/8.5.
RECORD_TOPICS = [
    '/camera/camera/color/image_raw/compressed',
    '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth',
    '/camera/camera/color/camera_info',
    '/camera/camera/aligned_depth_to_color/camera_info',
    '/camera/camera/extrinsics/depth_to_color',
    '/tf',
    '/tf_static',
    '/joint_states',
    '/robot_description',
    '/robot_description_semantic',
    '/tomato_boxes',
    '/tomato_candidates',
    '/target_radius_m',
    '/monitored_planning_scene',
]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('record', default_value='false',
                                        description='true면 rosbag 기록도 함께 시작'))
    ld.add_action(DeclareLaunchArgument('bag_name', default_value='scene',
                                        description='bags/<이름> 으로 저장'))
    ld.add_action(DeclareLaunchArgument('duration', default_value='30',
                                        description='기록 길이(초). 0이면 Ctrl+C까지'))
    ld.add_action(DeclareLaunchArgument('clear_delay', default_value='25.0',
                                        description='/clear_octomap 자동 호출까지 대기(초)'))

    # ---- 스택 + 카메라 + octomap + 필터 노드 ----
    # 기록 세션에서는 필터 노드를 런치 기본값 그대로 쓴다. 카메라가 실시간이라
    # header.stamp가 현재 시각이므로 restamp_now가 필요 없다.
    # GroupAction(scoped=True)로 감싼다 — IncludeLaunchDescription의
    # launch_arguments는 스코프를 만들지 않아 부모 컨텍스트를 오염시킨다
    # (scene_replay.launch.py에서 use_rviz가 덮어써져 RViz가 안 뜨는 사고를 겪었다).
    # 여기서는 지금 당장 충돌하는 이름이 없지만, 나중에 인자를 추가하다 같은 함정을
    # 밟지 않도록 같은 형태로 둔다.
    ld.add_action(
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('mycobot_280_moveit2'),
                        '/launch/demo_octomap.launch.py',
                    ]),
                    launch_arguments={'enable_octomap': 'true'}.items(),
                )
            ],
        )
    )

    # ---- YOLO (target_point 리맵 = 팔 자동 이동 차단) ----
    ld.add_action(
        Node(
            package='mycobot_280_pick',
            executable='yolo_d435_detector_node',
            name='yolo_d435_detector_node',
            output='screen',
            remappings=[('target_point', 'target_point_dryrun')],
        )
    )

    # ---- 모든 노드가 뜬 뒤 octomap 한 번 비우기 (위 "기록 순서가 중요하다") ----
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

    # ---- rosbag 기록 ----
    # 길이 제한은 SIGINT로 건다. `ros2 bag record`의 -d는 "이 주기로 파일 분할"이지
    # "이만큼 녹화하고 멈춤"이 아니고(--max-duration은 Jazzy에 없음), SIGKILL로 끊으면
    # metadata.yaml이 안 써져서 bag이 열리지 않는다.
    #
    # octomap을 비운 **뒤** 기록을 시작해야 토마토 구멍이 있는 상태가 담긴다.
    record_cmd = ['ros2', 'bag', 'record', '--storage', 'mcap',
                  '-o', ['bags/', LaunchConfiguration('bag_name')]] + RECORD_TOPICS
    ld.add_action(
        TimerAction(
            period=PythonExpression([LaunchConfiguration('clear_delay'), ' + 3.0']),
            actions=[
                ExecuteProcess(
                    cmd=(['timeout', '-s', 'INT', LaunchConfiguration('duration')]
                         + record_cmd),
                    output='screen',
                    condition=IfCondition(
                        PythonExpression(
                            ["'", LaunchConfiguration('record'), "'.lower() == 'true' and ",
                             LaunchConfiguration('duration'), " > 0"]
                        )
                    ),
                ),
                ExecuteProcess(
                    cmd=record_cmd,
                    output='screen',
                    condition=IfCondition(
                        PythonExpression(
                            ["'", LaunchConfiguration('record'), "'.lower() == 'true' and ",
                             LaunchConfiguration('duration'), " <= 0"]
                        )
                    ),
                ),
            ],
        )
    )

    return ld
