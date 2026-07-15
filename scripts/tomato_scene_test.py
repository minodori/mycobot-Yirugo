#!/usr/bin/env python3
"""
토마토 수확 씬 테스트 스크립트 (pymoveit2 기반)

씬 구성:
  - 목표 토마토 (빨간 구, 접근 대상)
  - 방해용 토마토 2개 (충돌 회피 확인용)
  - 나뭇잎/가지 장애물 1개 (목표 토마토 바로 앞을 가로막아 우회 경로를 강제)

사전 준비:
  pip install pymoveit2 --break-system-packages
  (또는 git clone https://github.com/AndrejOrsula/pymoveit2 하여 워크스페이스에 포함)

실행 전:
  RPi 또는 로컬에서 demo.launch.py가 이미 실행 중이어야 함
  (move_group, ros2_control 백엔드가 떠 있어야 이 스크립트가 통신 가능)

*** 확인 필요한 값 (SRDF/URDF 기준으로 맞춰야 함) ***
  - base_link_name: "g_base" (static_transform_publisher 로그에서 확인됨)
  - end_effector_name: myCobot 280 SRDF의 실제 엔드이펙터 링크명으로 교체
    -> 확인법: ros2 run tf2_ros tf2_echo g_base <후보링크명>
       또는 firefighter.srdf 파일 안 <end_effector> 태그 확인
  - joint_names: SRDF arm_group 순서 기준 (아래 6개, 확인 후 필요시 수정)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

from pymoveit2 import MoveIt2


# ---- 로봇 설정 (확인 필요) ----
JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
BASE_LINK_NAME = "g_base"
END_EFFECTOR_NAME = "joint6_flange"  # TODO: 실제 링크명으로 교체
GROUP_NAME = "arm_group"


def main():
    rclpy.init()

    node = Node("tomato_scene_test")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=JOINT_NAMES,
        base_link_name=BASE_LINK_NAME,
        end_effector_name=END_EFFECTOR_NAME,
        group_name=GROUP_NAME,
        callback_group=callback_group,
    )

    # 별도 스레드에서 spin (액션/서비스 응답 받으려면 필요)
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # DDS discovery 대기: /collision_object 퍼블리셔가 막 생성된 직후라
    # move_group(구독자)이 아직 이 퍼블리셔를 발견 못 했을 수 있음.
    # 여기서 기다리지 않으면 publish한 메시지가 조용히 유실됨.
    node.get_logger().info("퍼블리셔 discovery 대기 중...")
    import time
    time.sleep(2.0)

    node.get_logger().info("씬에 토마토 + 장애물 추가 중...")

    # ---- 1. 목표 토마토 (수확 대상, 빨간 구) ----
    target_position = [0.28, 0.00, 0.20]  # x, y, z (base_link 기준, 단위: m)
    moveit2.add_collision_sphere(
        id="tomato_target",
        radius=0.02,
        position=target_position,
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id=BASE_LINK_NAME,
    )

    # ---- 2. 방해용 토마토 2개 (경로 상 다른 위치) ----
    moveit2.add_collision_sphere(
        id="tomato_distractor_1",
        radius=0.02,
        position=[0.22, 0.08, 0.28],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id=BASE_LINK_NAME,
    )
    moveit2.add_collision_sphere(
        id="tomato_distractor_2",
        radius=0.02,
        position=[0.25, -0.10, 0.15],
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id=BASE_LINK_NAME,
    )

    # ---- 3. 나뭇잎/가지 장애물 (목표 토마토 바로 앞을 가로막음) ----
    # 얇은 박스로 표현. 팔이 직선으로 못 가고 옆으로 돌아가야 하는 상황을 만듦.
    moveit2.add_collision_box(
        id="leaf_obstacle",
        size=[0.08, 0.10, 0.005],  # 가로, 세로, 두께 (얇은 판)
        position=[0.15, 0.00, 0.20],  # 로봇 쪽으로 당겨서 접근점과 안 겹치게 배치
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],
        frame_id=BASE_LINK_NAME,
    )

    node.get_logger().info("씬 구성 완료. RViz에서 Scene Objects 확인해보세요.")

    # ---- 4. 목표 토마토 "앞"으로 접근하는 pose 설정 (현재 비활성화) ----
    # 이제 이동은 coord_to_goal_node(mycobot_280_pick)가 /target_point 토픽으로
    # 받은 좌표로 대신 수행함. 이 스크립트는 씬(장애물) 등록 전용으로만 사용.
    #
    # 그리퍼가 없으므로 팔 끝(flange)이 토마토 근처까지 접근하는 것으로 대체
    # approach_position = [
    #     target_position[0] - 0.05,  # 토마토보다 약간 앞(로봇 쪽)에서 정지
    #     target_position[1],
    #     target_position[2],
    # ]
    # approach_quat = [0.0, 0.0, 0.0, 1.0]  # TODO: 실제로는 접근 방향에 맞는 orientation 필요
    #
    # node.get_logger().info(f"목표 위치로 플래닝: {approach_position}")
    # moveit2.move_to_pose(
    #     position=approach_position,
    #     quat_xyzw=approach_quat,
    #     cartesian=False,  # False = OMPL 자유 플래닝 (장애물 회피 확인용)
    # )
    # moveit2.wait_until_executed()
    #
    # node.get_logger().info("플래닝/실행 완료. RViz에서 경로 확인해보세요.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()