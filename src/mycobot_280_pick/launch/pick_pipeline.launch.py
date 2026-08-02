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

[2026-08-02] `cycle:=`로 **수확 사이클의 종류**를 고를 수 있음. 대기 자세를
무엇으로 두느냐가 곧 사이클의 종류다(coord_to_goal_node의 WAITING_POSES 참고):

  cycle:=bsc   Bin-Staged   수확통 위에서 대기하며 거기서 놓는다 (기본)
  cycle:=asc   Armed-Staged armed pose에서 대기 (2026-08-01)
  cycle:=lpc   Look-Parked  목표마다 look pose로 복귀 (원판)

  ros2 launch mycobot_280_pick pick_pipeline.launch.py cycle:=asc

`look|armed|bin`도 그대로 받음. 값 검증은 노드 한 곳에서만 하므로(모르는 값이면
경고 후 BSC), 여기서는 그대로 넘기기만 한다.

[2026-08-02] `harvest_sequence_node`가 이 런치에 들어왔다. 그 전까지는 YOLO가
`/target_point`를 직접 쏴서 `coord_to_goal_node`가 바로 움직였는데, 그 경로가
자동 루프 사고의 통로여서 기본으로 껐다(`publish_target_point:=false`).
**그래서 시퀀스 노드가 없으면 이 런치는 팔을 전혀 안 움직인다.** 수확은
서비스로 시작한다:

  ros2 service call /start_harvest_sequence std_srvs/srv/Trigger

목표 출처도 여기서 고른다 — YOLO 라이브 검출(기본) 또는 사람이 보정한 파일:

  ros2 launch mycobot_280_pick pick_pipeline.launch.py \
      target_source:=file targets_file:=bags/lab_bed_detections_fixed.json
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
        DeclareLaunchArgument(
            'target_source',
            default_value='yolo',
            description=(
                '수확 목표를 어디서 받나. yolo(기본, look pose 검출 스냅샷) | '
                'file(targets_file의 보정 좌표). 파일 좌표는 g_base라 TF를 '
                '안 탄다.'
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'targets_file',
            default_value='',
            description='target_source:=file일 때 읽을 좌표 파일'
                        '(scripts/export_detections.py 출력 형식)',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'cycle',
            default_value='bsc',
            description=(
                '수확 사이클의 종류 = 대기 자세. bsc(수확통, 기본) | asc(armed) | '
                'lpc(look). look|armed|bin도 받음. 노드의 waiting_pose 파라미터로 '
                '그대로 넘어가고 검증도 거기서 한다.'
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
            parameters=[{'waiting_pose': LaunchConfiguration('cycle')}],
        )
    )

    # [2026-08-02] 수확 시퀀스 노드. **이게 있어야 팔이 움직인다** — YOLO의
    # target_point 직접 발행을 껐기 때문에(위 docstring) 파지 명령을 내는 주체가
    # 이 노드뿐이다. 시작은 /start_harvest_sequence 서비스.
    ld.add_action(
        Node(
            package='mycobot_280_pick',
            executable='harvest_sequence_node',
            name='harvest_sequence_node',
            parameters=[{
                'target_source': LaunchConfiguration('target_source'),
                'targets_file': LaunchConfiguration('targets_file'),
            }],
        )
    )

    return ld
