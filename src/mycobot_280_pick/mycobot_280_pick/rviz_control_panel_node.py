#!/usr/bin/env python3
"""
[Tier4, nice-to-have, so101-ros-physical-ai의 rviz_control_panel_node 이식]
RViz에 인터랙티브 마커(우클릭 메뉴)를 하나 띄워, 매번 `ros2 service call ...`을
터미널에 치는 대신 마커 메뉴 클릭만으로 다음 여섯 가지를 호출할 수 있게 함:

  - "look pose로 이동"       -> /go_to_look_pose (coord_to_goal_node)
  - "수확 시퀀스 시작"        -> /start_harvest_sequence (harvest_sequence_node)
  - "정지(소프트, 궤적 취소)" -> /emergency_stop (coord_to_goal_node)
  - "서보 릴리즈"            -> /release_servos (RPi sync_plan)
  - "서보 재포커스"          -> /refocus_servos (RPi sync_plan)
  - "grasp 확인/실행"        -> /confirm_grasp (coord_to_goal_node,
    require_grasp_confirmation:=true로 띄웠을 때만 의미 있음 — 대기 중인
    목표가 없으면 거부됨)

여섯 서비스 모두 std_srvs/srv/Trigger. 실제 동작/완료 여부는 이 노드가 아니라
호출받는 노드(coord_to_goal_node/harvest_sequence_node/RPi sync_plan)의
로그로 확인할 것 — 이 노드는 순수하게 "클릭 -> 서비스 호출"만 담당하는 얇은
UI 계층임. sync_plan은 RPi에서 도는 별도 노드지만 같은 ROS_DOMAIN_ID면
서비스가 그대로 discover되므로 이 노드가 로컬에서 떠도 호출 가능함.

주의(so101 교훈과 동일): "정지" 메뉴는 MoveIt 궤적 실행을 취소하는
소프트 정지일 뿐, mycobot 280의 실물 서보 토크를 직접 끊는 하드웨어
e-stop이 아님(coord_to_goal_node._on_emergency_stop_request 독스트링 참고).
"서보 릴리즈"가 실제 토크 차단임(sync_plan._on_release_servos 참고, 릴리즈
중엔 sync_plan이 /joint_states relay를 일시정지함). 진짜 위급 상황에서는
반드시 실물 전원/RPi 쪽 조치를 우선할 것.
"""

import functools

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

# 패널 마커를 띄울 위치(g_base 기준) — 팔 작업공간 옆, 시야를 가리지 않는 자리.
BASE_LINK_NAME = 'g_base'
PANEL_POSITION = (0.0, -0.25, 0.35)
PANEL_BOX_SIZE = 0.06

GO_TO_LOOK_POSE_SERVICE = 'go_to_look_pose'
START_HARVEST_SERVICE = 'start_harvest_sequence'
EMERGENCY_STOP_SERVICE = 'emergency_stop'
RELEASE_SERVOS_SERVICE = 'release_servos'
REFOCUS_SERVOS_SERVICE = 'refocus_servos'
CONFIRM_GRASP_SERVICE = 'confirm_grasp'
# [2026-08-03] 재생 속도 조절. coord_to_goal_node의 speed_scale 파라미터를
# 직접 바꾼다(그 노드가 쓰는 자리마다 다시 읽으므로 재시작이 필요 없다).
#
# **RViz의 MotionPlanning 패널에 있는 Velocity Scaling 슬라이더는 이 노드와
# 무관하다** — 그건 그 패널이 직접 보내는 계획에만 붙는다. 우리 사이클은
# coord_to_goal_node가 계획하므로 그쪽 파라미터를 바꿔야 한다.
SPEED_PARAM_SERVICE = '/coord_to_goal_node/set_parameters'
SPEED_PRESETS = (1.0, 2.0, 3.0, 5.0)


class RvizControlPanelNode(Node):

    def __init__(self):
        super().__init__('rviz_control_panel_node')

        self._look_pose_client = self.create_client(Trigger, GO_TO_LOOK_POSE_SERVICE)
        self._harvest_client = self.create_client(Trigger, START_HARVEST_SERVICE)
        self._stop_client = self.create_client(Trigger, EMERGENCY_STOP_SERVICE)
        self._release_client = self.create_client(Trigger, RELEASE_SERVOS_SERVICE)
        self._refocus_client = self.create_client(Trigger, REFOCUS_SERVOS_SERVICE)
        self._confirm_grasp_client = self.create_client(Trigger, CONFIRM_GRASP_SERVICE)
        self._speed_client = self.create_client(SetParameters, SPEED_PARAM_SERVICE)

        self._server = InteractiveMarkerServer(self, 'rviz_control_panel')
        self._menu_handler = MenuHandler()
        self._menu_handler.insert(
            'look pose로 이동', callback=self._on_look_pose_menu
        )
        self._menu_handler.insert(
            '수확 시퀀스 시작', callback=self._on_harvest_menu
        )
        self._menu_handler.insert(
            '정지(소프트, 궤적 취소)', callback=self._on_stop_menu
        )
        self._menu_handler.insert(
            '서보 릴리즈', callback=self._on_release_menu
        )
        self._menu_handler.insert(
            '서보 재포커스', callback=self._on_refocus_menu
        )
        self._menu_handler.insert(
            'grasp 확인/실행', callback=self._on_confirm_grasp_menu
        )
        # 속도는 하위 메뉴로 묶는다 — 최상위에 네 줄을 더하면 자주 쓰는
        # 항목(수확 시작·정지)이 밀려 내려간다.
        speed_menu = self._menu_handler.insert('재생 속도')
        for preset in SPEED_PRESETS:
            label = f'x{preset:.0f}' + (' (실물 기본)' if preset == 1.0 else '')
            self._menu_handler.insert(
                label, parent=speed_menu,
                callback=functools.partial(self._on_speed_menu, preset),
            )

        self._create_panel_marker()

        self.get_logger().info(
            'rviz_control_panel_node 준비 완료 — RViz에 InteractiveMarkers '
            '디스플레이 추가 후 Interactive Marker Namespace를 '
            "'rviz_control_panel'로 설정하면 마커가 보임. 우클릭으로 메뉴 호출."
        )

    def _on_speed_menu(self, scale, feedback) -> None:
        """coord_to_goal_node의 speed_scale을 바꾼다.

        **실물에서는 1.0으로 되돌릴 것.** 이 값은 VELOCITY_SCALING(0.2) 등
        실물에서 튜닝한 스케일링에 그대로 곱해지고, 계획한 궤적의 시간축이
        곧 실물 속도다(sync_plan은 /joint_states를 중계할 뿐이다).
        """
        if not self._speed_client.service_is_ready():
            self.get_logger().warn(
                f'{SPEED_PARAM_SERVICE} 없음 — coord_to_goal_node가 떠 있는지 확인할 것')
            return
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name='speed_scale',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                 double_value=float(scale)))]
        self._speed_client.call_async(request)
        self.get_logger().info(
            f'재생 속도 x{scale:.0f} 요청 (다음 동작부터 반영)'
            + ('' if scale == 1.0 else ' — 실물에서는 x1로 되돌릴 것'))

    def _create_panel_marker(self) -> None:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = BASE_LINK_NAME
        int_marker.name = 'harvest_control_panel'
        int_marker.description = '수확 제어판 (우클릭)'
        int_marker.scale = 0.15
        int_marker.pose.position.x = PANEL_POSITION[0]
        int_marker.pose.position.y = PANEL_POSITION[1]
        int_marker.pose.position.z = PANEL_POSITION[2]
        int_marker.pose.orientation.w = 1.0

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = PANEL_BOX_SIZE
        box_marker.scale.y = PANEL_BOX_SIZE
        box_marker.scale.z = PANEL_BOX_SIZE
        box_marker.color.r = 0.2
        box_marker.color.g = 0.5
        box_marker.color.b = 1.0
        box_marker.color.a = 0.8

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.always_visible = True
        menu_control.markers.append(box_marker)
        int_marker.controls.append(menu_control)

        self._server.insert(int_marker, feedback_callback=self._on_marker_feedback)
        self._menu_handler.apply(self._server, int_marker.name)
        self._server.applyChanges()

    def _on_marker_feedback(self, feedback) -> None:
        # 메뉴 항목 클릭은 MenuHandler에 등록한 개별 콜백이 처리함 — 여기선
        # 마커 자체 클릭(메뉴 팝업 트리거) 이벤트라 별도 동작 불필요.
        pass

    def _call_trigger_service(self, client, service_name: str, label: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warn(
                f"'{label}' 요청 무시 — {service_name} 서비스가 아직 준비 안 됨"
            )
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            functools.partial(self._on_trigger_response, label=label)
        )

    def _on_trigger_response(self, future, label: str) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - 서비스 호출 실패는 로그만 남김
            self.get_logger().error(f"'{label}' 서비스 호출 실패: {exc}")
            return

        if response.success:
            self.get_logger().info(f"'{label}' 완료: {response.message}")
        else:
            self.get_logger().warn(f"'{label}' 거부됨: {response.message}")

    def _on_look_pose_menu(self, feedback) -> None:
        self._call_trigger_service(
            self._look_pose_client, GO_TO_LOOK_POSE_SERVICE, 'look pose로 이동'
        )

    def _on_harvest_menu(self, feedback) -> None:
        self._call_trigger_service(
            self._harvest_client, START_HARVEST_SERVICE, '수확 시퀀스 시작'
        )

    def _on_stop_menu(self, feedback) -> None:
        self._call_trigger_service(
            self._stop_client, EMERGENCY_STOP_SERVICE, '정지(소프트, 궤적 취소)'
        )

    def _on_release_menu(self, feedback) -> None:
        self._call_trigger_service(
            self._release_client, RELEASE_SERVOS_SERVICE, '서보 릴리즈'
        )

    def _on_refocus_menu(self, feedback) -> None:
        self._call_trigger_service(
            self._refocus_client, REFOCUS_SERVOS_SERVICE, '서보 재포커스'
        )

    def _on_confirm_grasp_menu(self, feedback) -> None:
        self._call_trigger_service(
            self._confirm_grasp_client, CONFIRM_GRASP_SERVICE, 'grasp 확인/실행'
        )


def main():
    rclpy.init()

    node = RvizControlPanelNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
