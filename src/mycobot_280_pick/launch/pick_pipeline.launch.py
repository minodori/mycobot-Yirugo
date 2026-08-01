"""
YOLO+D435 검출 -> coord_to_goal_node -> MoveIt2(Octomap 충돌회피) 전체
파이프라인을 한 번에 띄우는 launch 파일.

mycobot_280_moveit2의 demo_octomap.launch.py(MoveIt2 + Octomap + D435 +
핸드-아이 캘리브레이션 결과 발행, docs 13장)에 이 패키지의
yolo_d435_detector_node와 coord_to_goal_node를 추가로 얹음. 카메라 노드는
demo_octomap.launch.py가 하나만 띄우고, YOLO 검출은 그 노드가 발행하는
ROS2 토픽(image_raw/aligned_depth_to_color)을 그대로 구독하므로 장치
충돌 없이 Octomap 충돌회피와 검출이 동시에 동작함.

사용법:
  ros2 launch mycobot_280_pick pick_pipeline.launch.py
  ros2 launch mycobot_280_pick pick_pipeline.launch.py \
    model_path:=/path/to/model.pt

[2026-07-27] `enable_octomap:=false`로 Octomap 장애물 회피 파이프라인을
건너뛰고 검출->좌표계산->파지 경로만 먼저 검증할 수 있음(demo_octomap.
launch.py로 그대로 전달됨) — 그리퍼 근접거리 self-filter 잔여 voxel로 인한
간헐적 START_STATE_IN_COLLISION(`docs/obstacle_avoidance_manual_test.md`
"알려진 문제" 참고)이 harvest 검증을 자꾸 막을 때 사용. 이 모드에서는
MoveIt이 실제 장애물을 전혀 못 보니 팔 주변 안전을 사람이 직접 확인할 것.
  ros2 launch mycobot_280_pick pick_pipeline.launch.py enable_octomap:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# yolo_d435_detector_node.py의 DEFAULT_MODEL_PATH와 동일 (빈 문자열로
# 넘기면 노드의 기본값을 덮어써버려서 여기서도 같은 기본값을 씀).
DEFAULT_MODEL_PATH = os.path.expanduser('~/Projects/Eval_Yolo/tomato_4cls_model.pt')


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument('model_path', default_value=DEFAULT_MODEL_PATH)
    )
    ld.add_action(
        DeclareLaunchArgument(
            'enable_octomap',
            default_value='true',
            description=(
                "false면 Octomap 장애물 회피 파이프라인을 건너뜀 — harvest "
                "파이프라인만 먼저 검증하고 싶을 때 사용(모듈 docstring 참고)."
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('mycobot_280_moveit2'), 'launch', 'demo_octomap.launch.py']
                )
            ),
            launch_arguments={
                'enable_octomap': LaunchConfiguration('enable_octomap'),
            }.items(),
        )
    )

    yolo_parameters = []
    model_path = LaunchConfiguration('model_path')
    yolo_parameters.append({'model_path': model_path})

    ld.add_action(
        Node(
            package='mycobot_280_pick',
            executable='yolo_d435_detector_node',
            name='yolo_d435_detector_node',
            parameters=yolo_parameters,
            # cv_bridge/ultralytics 임포트 우선순위 문제는 노드 코드 안에서
            # sys.path를 재정렬해서 해결함 (PYTHONNOUSERSITE는 ultralytics까지
            # 못 찾게 만들어서 여기선 안 씀 — 노드 파일 상단 주석 참고).
        )
    )

    ld.add_action(
        Node(
            package='mycobot_280_pick',
            executable='coord_to_goal_node',
            name='coord_to_goal_node',
        )
    )

    return ld
