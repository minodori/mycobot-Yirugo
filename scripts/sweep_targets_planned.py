#!/usr/bin/env python3
"""검출된 토마토 각각에 대해 **실제 MoveIt 플래닝**을 돌려 수확 가능성을 평가한다.

sweep_bed_offline.py와의 차이
-----------------------------
그쪽은 기하 게이트만 재현한다(ROS 불필요, 점당 1ms). 여기서 통과한 것은
"IK가 풀리면 성공" 후보일 뿐이다. 이 스크립트는 그 위에 **진짜 플래너**를 얹는다:
IK가 실제로 풀리는지, OMPL이 경로를 찾는지, 그 경로가 관절을 얼마나 돌리는지.

왜 실행(execute)은 안 하는가
----------------------------
FakeSystem은 궤적을 **실시간으로** 재생하므로 사이클당 약 30초가 걸리는데, 그중
판단 로직은 0.3초(1%)뿐이다(sweep_bed_offline.py 상단 참고). 플래닝만 하면
목표당 0.3~1초라 15개 x 반복 5회가 몇 분에 끝난다. 대신 실행 중에만 드러나는
문제(경로 무효화, 추종 오차)는 못 본다 — 그건 표본을 줄여 따로 검증해야 한다.

왜 compute_ik가 아니라 plan_kinematic_path인가
----------------------------------------------
`coord_to_goal_node`가 이미 실측해 둔 것이 있다: `avoid_collisions=True` +
정확한 orientation(tolerance 없음) 조합은 이 그리퍼 자세에서 **IK 성공률이
사실상 0**이다(그 파일 _on_roll_candidate_result 주석). 즉 compute_ik 단독
성공/실패는 신뢰할 수 있는 지표가 아니다. 반면 플래너는 자체적으로 목표를
샘플링하며 IK를 여러 번 시도하므로, "실제로 이 토마토에 갈 수 있는가"에
대한 답에 훨씬 가깝다.

무엇을 재는가 (목표당 --repeat 회 반복)
---------------------------------------
  성공률          OMPL은 확률적이라 같은 목표도 시도마다 갈린다
  플래닝 시간
  J1 도달각       정렬이 끝난 시점의 베이스 각도. **A/B 분기 판별용**
                  (docs/ROSBAG_HANDOFF.md 4절 — 예측 34°인데 실측의 54%가
                   127°짜리 "뒤로 감는 분기"였다. 그게 이 15개에서도 나오는가)
  6축 이동량 합   look pose에서 정렬 자세까지
  관절 한계 여유  URDF limit 대비 궤적 최댓값

측정 구간은 **look pose -> 정렬 위치**다. 사이클 시간의 62%가 정렬+복귀이므로
여기가 가장 중요하고, 위 4절 수치와 직접 비교할 수 있다.

사용법
------
    # 터미널 A — 스택만(카메라·RViz 없이). 로봇 없어도 된다
    ros2 launch mycobot_280_moveit2 demo_octomap.launch.py \
        enable_octomap:=false enable_camera:=false use_rviz:=false

    # 터미널 B
    python3 scripts/sweep_targets_planned.py --repeat 5 --csv planned.csv

`enable_octomap:=false`인 이유는 2절 함정 2와 같다 — 카메라가 없으면 octomap은
비어 있지만, 켜 두면 occupancy_map_monitor가 붙어 불필요한 부하만 준다.

장애물까지 넣고 재려면 `--tomatoes`(열매를 구로) 와 `--octomap FILE`(녹화 장면의
줄기·지지대)을 준다. octomap 파일은 `scripts/octomap_io.py capture`로 만든다.

    python3 -u scripts/sweep_targets_planned.py --repeat 10 \
        --orientation-tolerance 0.2 --tomatoes --octomap bags/bed_look_octomap.bin

**씬을 바꾸면 수치가 바뀐다**(2절 함정 3). 어느 씬에서 잰 값인지 반드시 같이
기록할 것. 씬 구성을 교차로 비교하려면 `scripts/eval_bed_scene.py`를 쓰는 편이
낫다 — 한 프로세스 안에서 조건만 갈아 끼우므로 환경 차이가 섞이지 않는다.
"""

import argparse
import json
import math
import os
import statistics
import sys
import time

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
for _p in ('src/mycobot_280_pick', 'src/pymoveit2'):
    sys.path.insert(0, os.path.join(_ROOT, _p))

try:
    import rclpy
    from geometry_msgs.msg import Point, PoseStamped
    from moveit_msgs.msg import (
        BoundingVolume,
        Constraints,
        DisplayRobotState,
        DisplayTrajectory,
        JointConstraint,
        MotionPlanRequest,
        OrientationConstraint,
        PositionConstraint,
        RobotState,
        WorkspaceParameters,
    )
    from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray
    from mycobot_280_pick import coord_to_goal_node as N
except ImportError as exc:  # pragma: no cover
    sys.exit(f'import 실패: {exc}\n  source /opt/ros/jazzy/setup.bash 를 먼저 할 것')

WRIST_LATERAL_OFFSET_M = 0.0732
SHOULDER = [0.0, 0.0, N.SHOULDER_HEIGHT_M]
PLAN_SERVICE = '/plan_kinematic_path'
# 스윕 전용 궤적 표시 토픽. 표준 /display_planned_path를 쓰면 move_group이 쏘는
# 원본 궤적과 섞인다(PlanProbe.__init__ 주석). sweep_view.rviz의 Trajectory
# 디스플레이가 이 토픽을 본다.
DISPLAY_TOPIC = '/sweep_display_path'
# 애니메이션을 **스크립트가 직접 그릴 때** 쓰는 토픽. sweep_view.rviz의
# RobotState 디스플레이가 이걸 본다. 아래 animate_trajectory 주석 참고.
STATE_TOPIC = '/sweep_robot_state'

# 목표 허용오차. coord_to_goal_node는 tolerance 없는 정확한 목표를 쓰지만, 그건
# 실행까지 하는 실제 파지라 그래야 한다. 여기서는 "도달 가능한가"를 보는 것이라
# 플래너에 현실적인 여유를 준다(위치 5mm, 방향 0.05rad ~= 2.9deg).
POSITION_TOLERANCE_M = 0.005
ORIENTATION_TOLERANCE_RAD = 0.05


def select_approach(x, y, z):
    """coord_to_goal_node와 **같은 함수**로 접근축을 고른다.

    sweep_bed_offline.evaluate()와 같은 로직이지만, 여기서는 플래닝에 필요한
    orientation(쿼터니언)까지 만들어 돌려준다.
    """
    r = math.hypot(x, y)
    az_deg = math.degrees(math.atan2(y, x))
    if not (N.MIN_TARGET_RADIUS_M <= r <= N.MAX_TARGET_RADIUS_M):
        return None, 'radius'
    if abs(az_deg) > N.MAX_TARGET_AZIMUTH_DEG:
        return None, 'azimuth'

    target = Point()
    target.x, target.y, target.z = x, y, z
    azimuth_rad = math.atan2(y, x)
    d = [t - p for t, p in zip((x, y, z), N.APPROACH_REFERENCE_POINT)]
    ideal = math.atan2(d[2], math.hypot(d[0], d[1]))

    cands = []
    for elev in N.APPROACH_ELEVATION_CANDIDATES_RAD:
        fwd = N._forward_unit_vector(azimuth_rad, elev)
        flange, align = N._waypoints_along_forward(target, fwd)
        align_r = math.hypot(align[0], align[1])
        if align_r < N.MIN_ALIGN_RADIUS_M:
            continue
        if math.dist(flange, SHOULDER) > N.MAX_SHOULDER_DISTANCE_M:
            continue
        if math.dist(align, SHOULDER) > N.MAX_ALIGN_SHOULDER_DISTANCE_M:
            continue
        cands.append({'elev': elev, 'err': abs(elev - ideal), 'fwd': fwd,
                      'flange': flange, 'align': align, 'align_radius': align_r})
    if not cands:
        return None, 'nocand'

    cands.sort(key=lambda c: c['err'])
    best = cands[0]
    best['quat'] = N._compute_look_at_quat_xyzw(best['fwd'])
    best['n_candidates'] = len(cands)
    best['elev_deg'] = math.degrees(best['elev'])
    ratio = min(1.0, WRIST_LATERAL_OFFSET_M / best['align_radius'])
    best['min_j1_deg'] = math.degrees(math.asin(ratio))
    return best, 'ok'


class PlanProbe(Node):

    def __init__(self, planning_time, attempts, start_pose=None):
        super().__init__('sweep_targets_planned')
        self._client = self.create_client(GetMotionPlan, PLAN_SERVICE)
        self._planning_time = planning_time
        self._attempts = attempts
        self.start_pose = list(start_pose or N.LOOK_POSE_JOINT_POSITIONS)
        self._limits = None
        self.create_subscription(JointState, 'joint_states', self._on_js, 10)
        self._seen_joints = None
        # [2026-08-01] 시각화/영상 녹화용. 계획 결과를 우리가 직접 발행한다.
        #
        # **전용 토픽을 쓴다 — 이게 중요하다.** 처음엔 표준 토픽
        # `/display_planned_path`에 쐈는데, 실측해 보니 그 토픽의 발행자가
        # **6개**였고 그중 5개가 move_group이었다. 서비스(`/plan_kinematic_path`)로
        # 계획해도 move_group이 결과를 그 토픽에 그대로 발행한다("액션 경로에서만
        # 발행한다"고 적어 뒀던 예전 주석은 틀렸다).
        #
        # 그래서 RViz가 재생하던 것은 우리가 시간을 손본 궤적이 아니라 **move_group이
        # 쏜 원본들**이었다. --repeat 3이면 목표마다 원본이 3개 더 날아가므로,
        # 한 목표에서 궤적이 열댓 번 재생되고 재생 속도 조절도 전혀 안 먹혔다.
        # 토픽을 분리하면 목표당 정확히 1개만 흐른다.
        self._display_pub = self.create_publisher(
            DisplayTrajectory, DISPLAY_TOPIC, 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/tomato_markers', 10)
        self._state_pub = self.create_publisher(
            DisplayRobotState, STATE_TOPIC, 10)
        # [3/5] 직진 접근을 재현하려면 노드와 같은 서비스를 써야 한다.
        self._cart_client = self.create_client(GetCartesianPath,
                                               '/compute_cartesian_path')

    def plan_straight_in(self, from_joints, best):
        """정렬 자세 -> flange 목표의 **직진 접근**([3/5])을 계획한다.

        스윕이 지금까지 보여준 것은 [1/5] 정렬(armed -> 정렬 위치)뿐이었다.
        그런데 "어느 방향에서 목표로 진입하는가"를 정하는 것은 정렬이 아니라
        **이 직진 구간**이다. 정렬은 OMPL 자유공간 경로라 마지막에 어느
        방향에서 들어올지 보장이 없고, 화면에서 본 "측면/아래위 진입"의
        상당 부분이 그 꼬리였다. 둘을 이어 붙여야 판단이 가능하다.

        노드의 `_move_arm_cartesian`과 같은 서비스·같은 해상도를 쓰므로,
        여기서 나오는 fraction이 곧 노드의 사전 dry-run
        (`_verify_grasp_approach_reachable`)이 보는 값이다.
        """
        if not self._cart_client.service_is_ready():
            if not self._cart_client.wait_for_service(timeout_sec=5.0):
                return None
        req = GetCartesianPath.Request()
        req.header.frame_id = N.BASE_LINK_NAME
        req.group_name = N.GROUP_NAME
        req.link_name = N.END_EFFECTOR_NAME
        req.max_step = N.CARTESIAN_MAX_STEP_M
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        state = RobotState()
        state.joint_state.name = list(N.JOINT_NAMES)
        state.joint_state.position = list(from_joints)
        state.is_diff = False
        req.start_state = state

        goal = PoseStamped()
        goal.header.frame_id = N.BASE_LINK_NAME
        (goal.pose.position.x, goal.pose.position.y,
         goal.pose.position.z) = best['flange']
        (goal.pose.orientation.x, goal.pose.orientation.y,
         goal.pose.orientation.z, goal.pose.orientation.w) = best['quat']
        req.waypoints = [goal.pose]

        fut = self._cart_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        res = fut.result()
        if res is None:
            return None
        return {'fraction': res.fraction, 'trajectory': res.solution,
                'points': len(res.solution.joint_trajectory.points)}

    def animate_trajectory(self, traj, seconds, repeats, fps=25.0):
        """궤적을 **스크립트가 직접** 한 프레임씩 그린다.

        왜 RViz Trajectory 디스플레이에 맡기지 않는가 — 재생 속도를 그쪽 설정
        (`State Display Time`)이 정하는데, 그 값이 이 MoveIt(2.12.4)에서는
        `0.5x` / `0.05s` / `0.1s` / `0.5s` 형식이고 **파싱에 실패하면 조용히
        기본값 `3x`로 되돌아간다.** 실제로 `REALTIME`(다른 버전의 표기)을 써서
        계속 3배속으로 재생되고 있었고, 설정을 바꿔도 안 바뀌는 것처럼 보였다.

        관절값을 직접 보간해서 DisplayRobotState로 쏘면 재생 속도도 반복 횟수도
        **파이썬 쪽 숫자 두 개**가 되어 RViz 설정과 무관해진다. 궤적이 28~44점
        뿐이라 그대로 재생하면 뚝뚝 끊기므로, 시간축으로 선형 보간해 fps로 채운다.

        중간에 rclpy를 계속 돌려 준다 — 이 동안 발행이 멈추면 RViz가 갱신되지 않는다.
        """
        pts = traj.joint_trajectory.points
        names = list(traj.joint_trajectory.joint_names)
        if not pts or seconds <= 0:
            return
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                 for p in pts]
        span = times[-1] or 1.0
        msg = DisplayRobotState()
        msg.state.joint_state.name = names
        msg.state.is_diff = False

        n_frames = max(2, int(seconds * fps))
        for _ in range(max(1, repeats)):
            t0 = time.time()
            for f in range(n_frames + 1):
                want = f / n_frames * span
                # want가 든 구간을 찾아 선형 보간
                i = 0
                while i < len(times) - 2 and times[i + 1] < want:
                    i += 1
                lo, hi = times[i], times[i + 1]
                u = 0.0 if hi <= lo else (want - lo) / (hi - lo)
                msg.state.joint_state.position = [
                    a + (b - a) * u
                    for a, b in zip(pts[i].positions, pts[i + 1].positions)]
                msg.state.joint_state.header.stamp = \
                    self.get_clock().now().to_msg()
                self._state_pub.publish(msg)
                # 프레임 목표 시각까지 rclpy를 돌리며 기다린다
                target = t0 + (f + 1) / n_frames * seconds
                while time.time() < target:
                    rclpy.spin_once(self, timeout_sec=0.005)

    def publish_trajectory(self, response_trajectory, display_seconds=0.0):
        """계획 궤적을 RViz에 발행한다. 돌려주는 값은 **재생에 걸릴 초**다.

        display_seconds > 0이면 모든 웨이포인트의 time_from_start를 다시 스케일해
        **어느 궤적이든 정확히 그 초만큼** 재생되게 만든다.

        왜 "배속"이 아니라 "고정 길이"인가 — 실측해 보니 계획 궤적의
        time_from_start 총합이 목표마다 **2.8초에서 21.4초까지** 널뛴다(점 개수는
        28~37로 거의 같은데도). MoveIt의 시간 파라미터화가 관절 이동량에 따라
        길이를 정하기 때문이다. 그래서 배율로 조절하면 편차가 그대로 곱해져서,
        어떤 목표는 한 번 재생되고 어떤 목표는 열댓 번 반복된다 —
        **배속을 바꿔도 체감이 안 바뀌는 이유가 이것이었다.**

        길이를 통일하면 목표끼리 재생 속도가 같아져 비교가 되고, 반복 횟수도
        예측 가능해진다. 계획만 하고 실행은 안 하므로 시간을 바꿔도 안전과는
        무관하다.

        **전제: RViz의 Trajectory > State Display Time이 `REALTIME`이어야 한다.**
        고정값("0.05 s" 등)으로 두면 RViz가 time_from_start를 무시하고
        점 개수 x 그 값으로 재생하므로 이 조절이 통하지 않는다.
        """
        traj = response_trajectory
        pts = traj.joint_trajectory.points
        if display_seconds > 0 and pts:
            import copy
            traj = copy.deepcopy(traj)
            pts = traj.joint_trajectory.points
            last = pts[-1].time_from_start
            total = last.sec + last.nanosec * 1e-9
            # 총합이 0인 궤적(시간 파라미터화 실패)은 균등 간격으로 깔아 준다.
            for i, pt in enumerate(pts):
                if total > 0:
                    cur = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                    t = cur / total * display_seconds
                else:
                    t = (i / max(1, len(pts) - 1)) * display_seconds
                pt.time_from_start.sec = int(t)
                pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        msg = DisplayTrajectory()
        msg.model_id = 'firefighter'
        msg.trajectory_start = self._look_pose_state()
        msg.trajectory = [traj]
        self._display_pub.publish(msg)
        if display_seconds > 0:
            return display_seconds
        last = pts[-1].time_from_start if pts else None
        return (last.sec + last.nanosec * 1e-9) if last else 0.0

    def publish_markers(self, dets, current_index=None, status_by_index=None):
        """검출 토마토를 구로 표시한다. 영상에서 "무엇을 향해 가는지"가 보여야
        의미가 있으므로, 현재 목표는 크게/불투명하게 그린다.

        색: ripe=빨강, disease=주황, 그 외=회색. 평가가 끝난 것은 성공=초록,
        실패=검정 테두리 대신 어둡게 — 색만으로 결과가 읽히도록 한다.
        """
        array = MarkerArray()
        for i, d in enumerate(dets):
            m = Marker()
            m.header.frame_id = N.BASE_LINK_NAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'tomatoes'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = d['base_x']
            m.pose.position.y = d['base_y']
            m.pose.position.z = d['base_z']
            m.pose.orientation.w = 1.0
            # 추정 반지름 그대로 그린다(지름 = 2r). 실제 크기감이 보여야 한다.
            r = float(d.get('radius_m', 0.017))
            scale = 2.0 * r * (1.6 if i == current_index else 1.0)
            m.scale.x = m.scale.y = m.scale.z = scale

            name = d.get('class_name', '?')
            base = {'ripe': (0.85, 0.10, 0.10), 'disease': (0.95, 0.55, 0.10)}
            cr, cg, cb = base.get(name, (0.6, 0.6, 0.6))
            status = (status_by_index or {}).get(i)
            if status == 'ok':
                cr, cg, cb = (0.15, 0.75, 0.20)
            elif status == 'fail':
                cr, cg, cb = (0.25, 0.25, 0.25)
            alpha = 1.0 if i == current_index else 0.65
            m.color = ColorRGBA(r=cr, g=cg, b=cb, a=alpha)
            array.markers.append(m)
        self._marker_pub.publish(array)

    # 라벨을 놓는 자리(g_base)와 줄 높이(m). --label-pos / --label-scale로 덮어쓴다.
    #
    # 자리를 두 번 옮겼다. 처음엔 베드 바로 위(0.25, 0, 0.52) — 카메라를 돌리면
    # 글자가 열매 사이로 들어갔다. 다음엔 월드 바깥 위쪽(0.10, -0.34, 0.46) —
    # 이번엔 너무 멀어서 무엇에 대한 설명인지 연결이 안 됐다. 지금은
    # **베드 옆구리**다. 베드가 x≈0.25, y −0.07~+0.09, z 0.16~0.36이므로
    # 열매와는 안 겹치면서 시선 안에 같이 들어온다.
    LABEL_ANCHOR = (0.26, -0.21, 0.30)
    # 글자 높이(m)와 줄 간격 배수.
    #
    # **RViz MovableText는 글자를 `2 x scale.z` 높이로 그린다**(바이너리
    # 역어셈블로 확인: calculateTotalDimensionsForPositioning이 char_height_를
    # 두 배로 쓴다). 즉 scale.z=0.012는 실제 24mm다. 그런데 줄 간격을
    # 0.012*1.25=15mm로 줬으니 **줄이 9mm씩 겹쳐서** 5줄이 뭉개졌다 —
    # "텍스트가 흩어져 보인다"의 실제 원인이 이것이었다.
    # LABEL_LINE_GAP은 반드시 **2.0 이상**이어야 줄이 안 겹친다.
    #
    # 폰트 자체는 정상이다(Ogre.log에 'Liberation SansTexture' 512x256 생성
    # 확인). 다만 fontdef에 code_points가 없어 아틀라스가 33~166만 담으므로
    # **라벨은 ASCII로만 쓸 것** — 한글이나 `°`(176), `·`(183)를 넣으면
    # 글리프가 없어 폭이 1.0(정사각)으로 잡히고 글자는 안 보인다.
    LABEL_SCALE = 0.025            # 실제 글자 높이 = 이 값 x 2 = 50mm
    LABEL_LINE_GAP = 2.4           # 줄 간격 = 글자 높이(2x) x 1.2

    def publish_status_label(self, lines, rgb=(1.0, 1.0, 1.0), target_xyz=None):
        """씬 옆에 상태 텍스트를 띄운다 — 화면만 보고도 지금 무엇을 보는지 알게.

        터미널을 같이 안 보면 **지금 화면의 궤적이 어느 시작 자세·어느 목표의
        것인지** 알 수 없다. 영상으로 녹화하면 터미널이 아예 안 남으므로 더 그렇다.

        마커 ns를 'label'로 따로 둬서 publish_markers(ns='tomatoes')와 섞이지
        않게 한다 — RViz는 ns+id로 마커를 관리하므로 서로를 지우지 않는다.

        J1 도달각과 분기(A/B)를 같이 띄운다. armed pose의 이득이 상당 부분
        "B분기를 덜 고르게 되는 것"에서 나오므로(docs/ARMED_POSE_HANDOFF.md
        5.3절의 A/B 열), 궤적만 봐서는 그게 안 보인다.

        target_xyz를 주면 라벨에서 그 목표까지 **지시선**을 긋는다. 텍스트가
        공간에 떠 있기만 하면 어느 열매를 설명하는지 알 수 없기 때문이다
        (현재 목표를 1.6배로 그리는 것만으로는 부족하다는 것이 실사용에서 드러났다).
        """
        array = MarkerArray()
        ax, ay, az = self.LABEL_ANCHOR
        stamp = self.get_clock().now().to_msg()

        # **줄마다 마커를 따로 만든다.** 하나의 마커에 '\n'으로 넣으면 줄 간격을
        # RViz(MovableText)가 정하는데, 그 간격이 글자 높이에 비해 크게 잡혀
        # 다섯 줄이 세로로 한참 벌어진다 — "텍스트가 공간에 흩어져 있다"는 지적이
        # 두 번 나온 원인이 이것이었다. 줄마다 위치를 주면 간격이 우리 숫자가 된다.
        # 글자 실제 높이가 2*scale.z이므로 간격도 그 기준으로 잡는다.
        step = self.LABEL_SCALE * self.LABEL_LINE_GAP
        for i, line in enumerate(lines):
            m = Marker()
            m.header.frame_id = N.BASE_LINK_NAME
            m.header.stamp = stamp
            m.ns = 'label'
            m.id = 10 + i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = ax
            m.pose.position.y = ay
            m.pose.position.z = az - i * step
            m.pose.orientation.w = 1.0
            m.scale.z = self.LABEL_SCALE
            m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)
            m.text = line
            array.markers.append(m)
        # 줄 수가 줄었을 때 옛 줄이 남지 않게 지운다(RViz는 ns+id로 기억한다).
        for i in range(len(lines), 10):
            m = Marker()
            m.header.frame_id = N.BASE_LINK_NAME
            m.header.stamp = stamp
            m.ns = 'label'
            m.id = 10 + i
            m.action = Marker.DELETE
            array.markers.append(m)

        leader = Marker()
        leader.header.frame_id = N.BASE_LINK_NAME
        leader.header.stamp = stamp
        leader.ns = 'label'
        leader.id = 1
        leader.type = Marker.LINE_STRIP
        if target_xyz is None:
            leader.action = Marker.DELETE
        else:
            leader.action = Marker.ADD
            leader.scale.x = 0.003          # 선 두께
            leader.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.55)
            leader.pose.orientation.w = 1.0
            p0 = Point(); p0.x, p0.y, p0.z = ax, ay, az - len(lines) * step
            p1 = Point(); p1.x, p1.y, p1.z = target_xyz
            leader.points = [p0, p1]
        array.markers.append(leader)

        self._marker_pub.publish(array)

    def _on_js(self, msg):
        if self._seen_joints is None:
            self._seen_joints = list(msg.name)

    def wait_ready(self, timeout=30.0):
        t0 = time.time()
        while not self._client.wait_for_service(timeout_sec=0.5):
            if time.time() - t0 > timeout:
                return False
            self.get_logger().info(f'{PLAN_SERVICE} 대기 중...')
        return True

    def _look_pose_state(self):
        """모든 목표를 **같은 시작 자세**에서 플래닝한다.

        실제 스택의 현재 관절값을 쓰면 앞 목표의 결과에 따라 시작이 달라져
        목표 간 비교가 불가능해진다. 명시적으로 고정한다.

        [2026-08-01] 시작 자세를 `--start-pose`로 바꿀 수 있게 함(기본은 look
        pose). armed pos 후보를 평가하려면 "같은 목표들을 다른 시작 자세에서
        플래닝했을 때 무엇이 좋아지는가"를 재야 하는데, 여기가 유일하게 시작
        자세를 정하는 곳이다. 노드는 전혀 고치지 않아도 된다.
        """
        state = RobotState()
        state.joint_state.name = list(N.JOINT_NAMES)
        state.joint_state.position = list(self.start_pose)
        state.is_diff = False
        return state

    def plan_to_config(self, target_joints, from_joints=None):
        """[2026-08-01] 관절 목표(joint goal)로 플래닝한다 — 복귀 구간용.

        복귀는 `move_to_configuration`(pose goal이 아님)이라 목표가 결정적이다.
        그런데 **출발 자세**는 가는 길에서 OMPL이 고른 분기에 따라 달라진다.
        즉 복귀 비용은 가는 길의 분기와 결합돼 있고, 그걸 재려면 임의의 출발
        자세에서 관절 목표로 플래닝할 수 있어야 한다.
        """
        req = MotionPlanRequest()
        req.group_name = N.GROUP_NAME
        req.num_planning_attempts = self._attempts
        req.allowed_planning_time = self._planning_time
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0

        state = RobotState()
        state.joint_state.name = list(N.JOINT_NAMES)
        state.joint_state.position = list(from_joints or self.start_pose)
        state.is_diff = False
        req.start_state = state

        ws = WorkspaceParameters()
        ws.header.frame_id = N.BASE_LINK_NAME
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -1.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 1.0
        req.workspace_parameters = ws

        goal = Constraints()
        for name, value in zip(N.JOINT_NAMES, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        req.goal_constraints = [goal]

        return self._send(req)

    def plan_to(self, position, quat, box_xyz=None, yaw_free=False):
        req = MotionPlanRequest()
        req.group_name = N.GROUP_NAME
        req.num_planning_attempts = self._attempts
        req.allowed_planning_time = self._planning_time
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0
        req.start_state = self._look_pose_state()

        ws = WorkspaceParameters()
        ws.header.frame_id = N.BASE_LINK_NAME
        ws.min_corner.x = ws.min_corner.y = ws.min_corner.z = -1.0
        ws.max_corner.x = ws.max_corner.y = ws.max_corner.z = 1.0
        req.workspace_parameters = ws

        pose = PoseStamped()
        pose.header.frame_id = N.BASE_LINK_NAME
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quat

        # [2026-08-01] 목표가 "점"이 아닐 수 있다. 수확통에 떨어뜨릴 때는
        # 통 넓이(8x8cm) 안 어디든 되고, 그리퍼가 아래를 보기만 하면 수직축
        # 둘레 회전(yaw)은 완전히 자유다. 그 자유도를 제약에 반영하면 플래너가
        # 훨씬 싼 해를 고를 수 있다 — box_xyz는 위치 상자, yaw_free는 그리퍼
        # 정면축(로컬 +Z) 둘레를 풀어준다.
        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = N.END_EFFECTOR_NAME
        pc.weight = 1.0
        shape = SolidPrimitive()
        if box_xyz:
            shape.type = SolidPrimitive.BOX
            shape.dimensions = list(box_xyz)
        else:
            shape.type = SolidPrimitive.SPHERE
            shape.dimensions = [POSITION_TOLERANCE_M]
        volume = BoundingVolume()
        volume.primitives = [shape]
        volume.primitive_poses = [pose.pose]
        pc.constraint_region = volume

        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = N.END_EFFECTOR_NAME
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        oc.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        # 그리퍼 로컬 +Z가 정면(coord_to_goal_node _compute_look_at_quat_xyzw)이므로
        # 그 축 둘레가 곧 "정면 방향을 유지한 채 도는" 자유도다.
        oc.absolute_z_axis_tolerance = math.pi if yaw_free else ORIENTATION_TOLERANCE_RAD
        oc.weight = 1.0

        goal = Constraints()
        goal.position_constraints = [pc]
        goal.orientation_constraints = [oc]
        req.goal_constraints = [goal]

        return self._send(req)

    def _send(self, req):
        request = GetMotionPlan.Request()
        request.motion_plan_request = req
        t0 = time.time()
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._planning_time + 15.0)
        elapsed = time.time() - t0
        if not future.done() or future.result() is None:
            return {'ok': False, 'error': 'timeout', 'wall_s': elapsed}

        res = future.result().motion_plan_response
        if res.error_code.val != 1:  # SUCCESS
            return {'ok': False, 'error': f'code {res.error_code.val}', 'wall_s': elapsed}

        traj = res.trajectory.joint_trajectory
        if not traj.points:
            return {'ok': False, 'error': 'empty', 'wall_s': elapsed}

        names = list(traj.joint_names)
        idx = {n: i for i, n in enumerate(names)}
        final = traj.points[-1].positions
        # 관절별 이동량 합 = 인접 웨이포인트 차의 절댓값 누적(단순 max-min이
        # 아니다 — 되돌아가는 움직임도 비용이므로).
        travel = [0.0] * len(names)
        for a, b in zip(traj.points, traj.points[1:]):
            for i in range(len(names)):
                travel[i] += abs(b.positions[i] - a.positions[i])

        j1_name = N.JOINT_NAMES[0]
        j1_final = final[idx[j1_name]] if j1_name in idx else float('nan')
        return {
            'ok': True,
            'trajectory': res.trajectory,   # --display로 RViz에 재생할 때 씀
            'wall_s': elapsed,
            'plan_s': res.planning_time,
            'points': len(traj.points),
            'j1_deg': math.degrees(j1_final),
            'travel_deg': math.degrees(sum(travel)),
            # 관절별 이동량. J6(flange 자전)만 따로 보려고 넣었다 —
            # 합만 보면 "플랜지가 180도 도는" 비용이 다른 축에 묻힌다.
            'travel_by_joint': {n: math.degrees(travel[idx[n]]) for n in names},
            'final': {n: final[idx[n]] for n in names},
            'peak': {n: max(abs(p.positions[idx[n]]) for p in traj.points) for n in names},
        }


def _concat(traj_a, traj_b):
    """궤적 둘을 이어 붙인다(재생용).

    관절 순서가 서비스마다 다르게 올 수 있으므로 **이름으로 매핑해 재배열**한다.
    그냥 붙이면 이어지는 순간 팔이 튀는 그림이 된다.
    시간축은 animate_trajectory가 다시 정규화하므로 순서만 맞으면 된다.
    """
    import copy
    out = copy.deepcopy(traj_a)
    names_a = list(out.joint_trajectory.joint_names)
    jt_b = traj_b.joint_trajectory
    try:
        idx = [jt_b.joint_names.index(n) for n in names_a]
    except ValueError:
        return out
    last = out.joint_trajectory.points[-1].time_from_start
    base_t = last.sec + last.nanosec * 1e-9
    for pt in jt_b.points[1:]:
        new_pt = copy.deepcopy(out.joint_trajectory.points[-1])
        new_pt.positions = [pt.positions[i] for i in idx]
        new_pt.velocities = []
        new_pt.accelerations = []
        t = base_t + pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
        new_pt.time_from_start.sec = int(t)
        new_pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        out.joint_trajectory.points.append(new_pt)
    return out


def _reversed(traj):
    """궤적을 되감은 복사본(재생용). [5/5] 후퇴가 직진의 역이라 그걸 그린다.

    노드의 후퇴는 정렬 위치로 되돌아가는 **같은 직선**이므로(coord_to_goal_node
    _start_retreat), 직진 궤적의 점 순서만 뒤집으면 그 구간이 된다.

    시간은 **원래 궤적의 시각 배열을 그대로 재사용**한다(오름차순 유지, 위치만
    역순). 점마다 1초씩 새로 매기면 되감기 구간만 길어져서 재생이 후퇴에
    잡아먹힌다 — animate_trajectory가 전체 span으로 정규화하기 때문이다.
    """
    import copy
    out = copy.deepcopy(traj)
    pts = list(out.joint_trajectory.points)
    times = [copy.deepcopy(p.time_from_start) for p in pts]
    pts.reverse()
    for i, pt in enumerate(pts):
        pt.velocities = []
        pt.accelerations = []
        pt.time_from_start = times[i]
    out.joint_trajectory.points = pts
    return out


def _spin(results):
    """성공한 계획 중 J6(flange 자전) 이동량의 최솟값. 실패뿐이면 무한대."""
    good = [r for r in results if r['ok']]
    return min((r['travel_by_joint'][N.JOINT_NAMES[5]] for r in good),
               default=float('inf'))


def _label_lines(pose_label, index, total, d, results, return_travel=None):
    """화면 라벨 문구.

    **공백(스페이스)을 쓰지 말 것. 이게 이 라벨의 유일한 함정이다.**

    Ogre 폰트 아틀라스의 기본 코드포인트 범위는 33~166인데 스페이스는 **32**라
    빠져 있다. MovableText는 스페이스 폭을
        space_width = getGlyphAspectRatio(0x20) * char_height * 2
    로 잡는데, 없는 글리프는 aspect 1.0을 돌려주므로 **공백 하나가
    2 * scale.z (지금 값으로 50mm)** 가 된다. 그래서 한 줄 안의 단어들이
    월드를 가로질러 흩어졌다 — "armed"와 "pose"가 멀리 떨어져 보이던 것이
    전부 이것이었다. 줄 간격도 글자 크기도 한글도 원인이 아니었다.

    구분자는 `_` (95), `:` (58), `/` (47) 처럼 **33~166 안의 문자**로 쓴다.
    같은 이유로 `°`(176)·`·`(183)·en-dash도 금지다.
    """
    good = [r for r in results if r['ok']]
    cls = d.get('class_name', '?')[:4]
    head = [pose_label.replace(' ', '_'),
            f'#{index+1}/{total}:{cls}:z{d["base_z"]:.2f}']
    if not good:
        return head + ['UNREACHABLE'], (0.95, 0.35, 0.35)
    best = min(good, key=lambda r: (r['travel_by_joint'][N.JOINT_NAMES[5]],
                                    r['travel_deg']))
    is_a = abs(best['j1_deg']) < 80
    rgb = (0.45, 0.8, 1.0) if is_a else (1.0, 0.62, 0.2)
    j6 = best['travel_by_joint'][N.JOINT_NAMES[5]]
    last = f'J6:{j6:.0f}:{"A" if is_a else "B-wrap"}:{len(good)}/{len(results)}'
    if return_travel is not None:
        last += f':ret{return_travel:.0f}'
    return head + [
        f'trv{best["travel_deg"]:.0f}:J1{best["j1_deg"]:+.0f}',
        last,
    ], rgb


def evaluate_all(node, dets, repeat, verbose=True, display_pause=0.0,
                 pose_label='look pose', display_seconds=3.0, display_repeats=3,
                 roll_symmetry=False, straight_in=False, acm_only_target=False,
                 full_cycle=False):
    """모든 목표를 repeat회씩 플래닝하고 행 목록을 돌려준다.

    display_pause > 0이면 목표마다 계획 궤적을 /display_planned_path로 발행하고
    그만큼 쉰다 — RViz에서 눈으로 보거나 화면 녹화할 때 쓴다. 이때는 반복 중
    **마지막 성공 궤적**을 보여준다(여러 개를 연달아 쏘면 RViz가 마지막 것만
    재생해서 앞의 것이 안 보인다).

    [2026-08-02] full_cycle이면 **복귀 구간까지 계획해 고리를 닫는다**:

        대기 자세 -> 정렬 -> 직진 -> [후퇴] 정렬 -> [복귀] 대기 자세

    세 가지 사이클(LPC/ASC/BSC)의 차이가 가장 크게 드러나는 곳이 복귀 구간이라,
    정렬까지만 그리면 세 영상이 "출발 자세만 다른 그림"이 된다. 복귀는 노드와
    같은 성격(joint goal, _start_return_to_waiting_pose)이므로 plan_to_config로
    잰다. 목표당 플래닝이 1회 늘어난다.
    """
    rows = []
    status = {}
    for index, d in enumerate(dets):
        # [2026-08-02] 목표마다 **그 열매만** 완화한다 — 나머지 14개는 장애물로
        # 남는다. 기본(전부 완화)은 그리퍼가 옆 열매를 통과하는 경로도 성공으로
        # 집계한다(함정 17). scene_objects.allow_gripper_tomato_collisions의
        # only 인자 설명 참고.
        if acm_only_target:
            import scene_objects
            scene_objects.allow_gripper_tomato_collisions(node, dets,
                                                          only=index, quiet=True)
        best, reason = select_approach(d['base_x'], d['base_y'], d['base_z'])
        if best is None:
            rows.append({**d, 'reason': reason, 'success': 0, 'trials': 0})
            status[index] = 'fail'
            if display_pause > 0:
                node.publish_markers(dets, index, status)
            continue
        if display_pause > 0:
            node.publish_markers(dets, index, status)
        # [2026-08-01] 그리퍼는 2지 평행이라 **roll과 roll+180도가 물리적으로
        # 같은 자세**인데, 목표를 완전한 orientation으로 주면 플래너는 그걸
        # 모르고 고정된 roll을 맞추려 J6를 반 바퀴 돌린다(RViz에서 "플랜지가
        # 180도 회전"으로 보이던 것). 둘 다 풀어 보고 싼 쪽을 쓴다.
        # 실측: J6 이동량 중앙 166 -> 14도, 전체 502 -> 375도
        # (scripts/eval_roll_symmetry.py).
        rolls = [best['quat']]
        if roll_symmetry:
            rolls.append(N._compute_look_at_quat_xyzw(best['fwd'],
                                                      roll_rad=math.pi))
        results = [node.plan_to(best['align'], rolls[0]) for _ in range(repeat)]
        if roll_symmetry:
            alt = [node.plan_to(best['align'], rolls[1]) for _ in range(repeat)]
            # **J6 이동량으로 고른다.** 처음엔 6축 합으로 골랐는데, 원하는 것은
            # "플랜지가 안 도는 것"이라 기준이 달랐다. 둘은 대개 일치하지만
            # (싼 roll이 대체로 J6도 적다) 어긋나는 목표가 남아서, 화면에서는
            # 그 목표들만 계속 반 바퀴 도는 것으로 보였다.
            if _spin(alt) < _spin(results):
                results = alt
        # [3/5] 직진 접근은 **표시와 무관하게** 재야 한다 — standoff를 바꾸면
        # 이 fraction이 노드의 사전 dry-run 통과 여부를 결정하기 때문이다.
        cart_fraction = None
        good_now = [r for r in results if r['ok']]
        if straight_in and good_now:
            pick = min(good_now,
                       key=lambda r: (r['travel_by_joint'][N.JOINT_NAMES[5]],
                                      r['travel_deg']))
            cart = node.plan_straight_in(
                [pick['final'][n] for n in N.JOINT_NAMES], best)
            if cart:
                cart_fraction = cart['fraction']
                cart_traj = cart['trajectory']
        # [2026-08-02] 복귀 구간 — 화면에 쓰든 안 쓰든 **수치로 남긴다.** 세
        # 사이클의 차이가 여기 있는데 지금까지 스윕이 안 재던 구간이다.
        # 출발점은 화면에 그리는 것과 같은 해(J6 최소)여야 한다 — 다른 해에서
        # 재면 라벨의 정렬 자세와 복귀 비용이 서로 다른 자세의 값이 된다.
        return_travel = None
        return_traj = None
        if full_cycle and good_now:
            shown_pick = min(good_now,
                             key=lambda r: (r['travel_by_joint'][N.JOINT_NAMES[5]],
                                            r['travel_deg']))
            from_joints = [shown_pick['final'][n] for n in N.JOINT_NAMES]
            # 정렬 구간과 **같은 집계**로 잰다(반복 중 최소). 처음엔 1회만
            # 계획했는데, 그러면 이 값만 IK 분기 무작위성에 그대로 노출된다
            # (함정 6·16 — 두 분기의 비용이 거의 같아 실행마다 갈린다).
            for _ in range(max(1, repeat)):
                back = node.plan_to_config(list(node.start_pose),
                                           from_joints=from_joints)
                if back['ok'] and (return_travel is None
                                   or back['travel_deg'] < return_travel):
                    return_travel = back['travel_deg']
                    return_traj = back['trajectory']
        if display_pause > 0:
            ok_results = [r for r in results if r['ok']]
            status[index] = 'ok' if ok_results else 'fail'
            hold = display_pause
            if ok_results:
                # 반복 중 **이동량이 가장 적은** 궤적을 보여준다. 예전에는
                # 마지막 것을 띄웠는데, 그러면 같은 목표가 회차마다 A분기/B분기를
                # 오가며 다르게 보여 "무엇이 이 자세의 결과인지"가 안 잡힌다.
                # 최소를 고르는 것은 Phase 3이 적용된 상태와도 일치한다.
                # 보여줄 궤적도 J6 최소 기준(라벨과 같은 해를 그려야 한다).
                shown = min(ok_results,
                            key=lambda r: (r['travel_by_joint'][N.JOINT_NAMES[5]],
                                           r['travel_deg']))
                show_traj = shown['trajectory']
                # 위에서 계획해 둔 직진 구간을 이어 붙인다 — 이게 있어야
                # 화면으로 "어느 방향에서 진입하는가"를 판단할 수 있다.
                if cart_fraction and cart_fraction > 0.0:
                    show_traj = _concat(show_traj, cart_traj)
                    # [5/5] 후퇴 = 그 직선을 되돌아 나오는 구간.
                    if full_cycle:
                        show_traj = _concat(show_traj, _reversed(cart_traj))
                # 복귀까지 붙이면 고리가 닫혀 대기 자세로 돌아온다.
                if full_cycle and return_traj is not None:
                    show_traj = _concat(show_traj, return_traj)
                node.publish_trajectory(show_traj, display_seconds)
            node.publish_markers(dets, index, status)
            lines, rgb = _label_lines(pose_label, index, len(dets), d, results,
                                      return_travel)
            node.publish_status_label(
                lines, rgb, (d['base_x'], d['base_y'], d['base_z']))
            # 애니메이션을 우리가 직접 돌린다(animate_trajectory 주석). 재생
            # 시간 x 반복 횟수가 그대로 이 목표에 머무는 시간이 된다.
            #
            # [2026-08-02] **show_traj를 넘긴다** — 예전엔 shown['trajectory']
            # (정렬 구간만)를 넘기고 있었다. 이어 붙인 궤적은 publish_trajectory
            # 로만 나가는데 그건 RViz의 Trajectory 디스플레이용이고 그 디스플레이는
            # 꺼져 있는 것이 정상이라(8절), **--straight-in을 줘도 화면에는 직진이
            # 안 보였다.** 이걸 안 고치면 --full-cycle도 같은 이유로 안 보인다.
            if ok_results:
                node.animate_trajectory(show_traj,
                                        display_seconds, display_repeats)
            t_end = time.time() + hold
            while time.time() < t_end:
                rclpy.spin_once(node, timeout_sec=0.05)
        good = [r for r in results if r['ok']]
        j1 = [r['j1_deg'] for r in good]
        travel = [r['travel_deg'] for r in good]
        plan_s = [r['plan_s'] for r in good]
        row = {
            **{k: d[k] for k in ('class_name', 'base_x', 'base_y', 'base_z') if k in d},
            'reason': 'ok',
            'trials': repeat,
            'success': len(good),
            'elev_deg': round(best['elev_deg'], 1),
            'pred_j1_deg': round(best['min_j1_deg'], 1),
            'j1_mean_deg': round(statistics.fmean(j1), 1) if j1 else None,
            'j1_min_deg': round(min(j1), 1) if j1 else None,
            'j1_max_deg': round(max(j1), 1) if j1 else None,
            'travel_mean_deg': round(statistics.fmean(travel), 0) if travel else None,
            'travel_sd_deg': round(statistics.pstdev(travel), 0) if len(travel) > 1 else 0,
            'plan_s_mean': round(statistics.fmean(plan_s), 3) if plan_s else None,
            # [2026-08-02] 정렬 -> 대기 자세 복귀 이동량. 세 사이클(LPC/ASC/BSC)의
            # 차이가 여기 있는데 지금까지 스윕이 안 재던 값이다. --full-cycle일
            # 때만 채워진다.
            'return_travel_deg': (None if return_travel is None
                                  else round(return_travel, 0)),
        }
        rows.append(row)
        if verbose:
            j1_txt = (f'J1 {row["j1_min_deg"]:.0f}~{row["j1_max_deg"]:.0f}°'
                      if j1 else 'J1 —')
            # J6(flange 자전)를 같이 찍는다. 6축 합에 묻혀서 "플랜지가 180도
            # 도는" 목표를 로그만으로는 못 찾았다.
            spin = _spin(results)
            spin_txt = f'J6 {spin:.0f}°' if spin != float('inf') else 'J6 —'
            frac_txt = ('' if cart_fraction is None
                        else f'  직진 {100*cart_fraction:.0f}%')
            ret_txt = ('' if return_travel is None
                       else f'  복귀 {return_travel:.0f}°')
            print(f'{d.get("class_name", "?"):<8} z={d["base_z"]:.3f}  '
                  f'성공 {len(good)}/{repeat}  {j1_txt}  {spin_txt}  '
                  f'이동량 {row["travel_mean_deg"] or 0:.0f}°±{row["travel_sd_deg"]:.0f}  '
                  f'플래닝 {row["plan_s_mean"] or 0:.2f}s' + frac_txt + ret_txt)
    return rows


def collect_solutions(node, dets, repeat, out_path, branch_max_j1_deg=80.0):
    """[2026-08-01] armed pos를 도출하기 위해 목표별 **정렬 관절해**를 모은다.

    docs/new_concept.md §10 "그 경로의 시작점이 armed pos로 정의한다"를 위한
    입력이다. armed pos 후보를 J1 격자로 훑는 것은 의미가 없다 — 그 격자의
    기준이 될 look pose J1(-7.5°)이 손으로 맞춘 값이라 기구학적 의미가 없기
    때문(docs/look_pose.md:4). 대신 "실제로 가야 할 자세들"을 모아 그 관절공간
    중심을 구한다.

    **A분기만 모은다.** 같은 목표에 도달하는 관절 조합이 두 가지 있는데,
      A(정상)  J1 32~47°  — 베이스가 적당히 돌고 팔을 앞으로 뻗음, 이동량 453°
      B(뒤로)  J1 122~139° — 베이스가 목표 방위각을 지나쳐 돌고 팔꿈치를 접어
                             반대편에서 닿음, 이동량 577°(21% 많음)
    노드의 목적함수(6축 이동량 최소)는 이미 A를 고르려 하고, 실행 단계에서
    OMPL이 다시 뽑으면서 뒤집힐 뿐이다. 즉 A만 모으는 것은 타협이 아니라
    "원래 쓰기로 한 해만 모으는 것"이다.

    **단 폴백이 있다**: 어떤 목표는 A가 관절 한계에 걸려 B밖에 없을 수 있다.
    A가 하나도 안 나오면 B라도 받는다 — 무조건 금지하면 그 토마토를 통째로
    놓친다.
    """
    print(f'\n정렬 관절해 수집: 목표 {len(dets)}개 x 반복 {repeat}회, '
          f'A분기 기준 |J1| < {branch_max_j1_deg:.0f}°\n')
    collected = []
    for d in dets:
        best, reason = select_approach(d['base_x'], d['base_y'], d['base_z'])
        label = f'{d.get("class_name", "?"):<8} z={d["base_z"]:.3f}'
        if best is None:
            print(f'{label}  기하 게이트 탈락({reason})')
            continue

        sols = []
        for _ in range(repeat):
            r = node.plan_to(best['align'], best['quat'])
            if r['ok']:
                sols.append(r)
        if not sols:
            print(f'{label}  플래닝 {repeat}회 전부 실패 — 제외')
            continue

        a = [s for s in sols if abs(s['j1_deg']) < branch_max_j1_deg]
        used, branch = (a, 'A') if a else (sols, 'B(폴백)')
        # 같은 분기 안에서도 해가 흔들리므로 6축 이동량이 가장 적은 것을 대표로.
        rep = min(used, key=lambda s: s['travel_deg'])
        collected.append({
            'class_name': d.get('class_name', '?'),
            'base_x': d['base_x'], 'base_y': d['base_y'], 'base_z': d['base_z'],
            'branch': branch,
            'n_success': len(sols), 'n_branch_a': len(a), 'trials': repeat,
            'j1_deg': round(rep['j1_deg'], 2),
            'travel_deg': round(rep['travel_deg'], 1),
            'joints': [round(rep['final'][n], 6) for n in N.JOINT_NAMES],
        })
        print(f'{label}  성공 {len(sols)}/{repeat}  A분기 {len(a)}개  '
              f'채택 {branch}  J1 {rep["j1_deg"]:+.1f}°  이동량 {rep["travel_deg"]:.0f}°')

    n_a = sum(1 for c in collected if c['branch'] == 'A')
    print(f'\n수집 {len(collected)}/{len(dets)}개  (A분기 {n_a}, B폴백 {len(collected)-n_a})')
    if out_path:
        with open(out_path, 'w', encoding='utf-8') as fh:
            json.dump({'joint_names': list(N.JOINT_NAMES),
                       'start_pose': list(node.start_pose),
                       'solutions': collected}, fh, indent=2, ensure_ascii=False)
        print(f'저장: {out_path}')
    return collected


def run_tolerance_sweep(node, dets, repeat, tolerances_deg, csv_path):
    """[2026-08-01] 방향 허용오차를 훑으며 성공률 곡선을 만든다.

    왜 필요한가: 첫 실행(2.9°)에서 15개 중 5개가 전회 실패했는데 11.5°로 풀자
    1개로 줄었다. 즉 **성공률이 이 값 하나에 크게 좌우된다**. 그러면 "이 베드를
    수확할 수 있는가"라는 질문은 "그리퍼를 얼마나 정확히 겨눠야 하는가"와 같은
    질문이 된다 — 실물 정밀도 요구사항을 정하는 근거가 되므로 곡선으로 남긴다.

    주의: 허용오차를 키우면 플래닝은 쉬워지지만 그리퍼가 목표를 비스듬히 물게
    되어 실제 파지 성공률은 반대로 떨어질 수 있다. 이 곡선만으로 "크게 잡을수록
    좋다"고 읽으면 안 된다 — 실물 파지 검증이 따로 필요하다.
    """
    global ORIENTATION_TOLERANCE_RAD
    print(f'\n방향 허용오차 스윕: {len(tolerances_deg)}개 값 x 목표 {len(dets)}개 '
          f'x 반복 {repeat}회\n')
    print(f'{"허용오차":>8}{"성공률":>10}{"전회성공":>9}{"간헐":>6}{"전회실패":>9}'
          f'{"A무리":>7}{"B무리":>7}')
    summary = []
    for deg in tolerances_deg:
        ORIENTATION_TOLERANCE_RAD = math.radians(deg)
        rows = evaluate_all(node, dets, repeat, verbose=False)
        planned = [r for r in rows if r.get('trials')]
        trials = sum(r['trials'] for r in planned)
        ok = sum(r['success'] for r in planned)
        full = sum(1 for r in planned if r['success'] == r['trials'])
        never = sum(1 for r in planned if r['success'] == 0)
        partial = len(planned) - full - never
        j1 = [r['j1_mean_deg'] for r in planned if r['j1_mean_deg'] is not None]
        a = sum(1 for v in j1 if v < 80)
        b = len(j1) - a
        print(f'{deg:7.1f}°{100*ok/trials:9.1f}%{full:9d}{partial:6d}{never:9d}'
              f'{a:7d}{b:7d}')
        summary.append({'tolerance_deg': deg, 'success_pct': round(100*ok/trials, 1),
                        'trials': trials, 'success': ok, 'full': full,
                        'partial': partial, 'never': never,
                        'branch_a': a, 'branch_b': b})

    if csv_path:
        import csv
        with open(csv_path, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=list(summary[0]))
            w.writeheader()
            w.writerows(summary)
        print(f'\nCSV 저장: {csv_path} ({len(summary)}행)')
    return summary


def main():
    global POSITION_TOLERANCE_M, ORIENTATION_TOLERANCE_RAD
    p = argparse.ArgumentParser(description='검출 토마토별 실제 플래닝 평가')
    p.add_argument('--display-loop', type=int, default=1, metavar='N',
                   help='--display-pause 모드에서 스윕 전체를 N회 반복한다. '
                        '0이면 Ctrl+C까지 무한 반복 — RViz Trajectory 디스플레이는 '
                        'Loop Animation이라 그냥 두면 **마지막 궤적만** 계속 '
                        '재생되므로, 전체를 다시 보려면 이 옵션이 필요하다')
    # [2026-08-02] standoff를 인자로 뺀 이유: 이 값을 바꾸면 정렬 위치가
    # 베이스 쪽으로 당겨져 MIN_ALIGN_RADIUS_M 경계에 붙는 목표가 생기는데,
    # 그 대가를 노드 상수를 고쳐가며 재면 다른 변경과 섞인다. 한 번에 하나만
    # 바꿔 비교할 수 있어야 한다.
    p.add_argument('--standoff', type=float, default=None, metavar='M',
                   help='APPROACH_STANDOFF_M을 이 값으로 덮어쓴다(m). '
                        '기본은 노드 상수 그대로')
    p.add_argument('--straight-in', action='store_true',
                   help='[3/5] 직진 접근을 정렬 궤적 뒤에 이어 붙여 같이 재생하고 '
                        'Cartesian fraction을 찍는다. 이게 없으면 화면에 보이는 '
                        '것은 OMPL 자유공간 경로뿐이라 접근 방향을 판단할 수 없다')
    p.add_argument('--roll-symmetry', action='store_true',
                   help='그리퍼의 180도 대칭을 이용한다. roll 0과 roll 180을 둘 다 '
                        '풀어 보고 이동량이 적은 쪽을 쓴다 — 물리적으로 같은 자세라 '
                        '공짜다. scripts/eval_roll_symmetry.py 참고')
    p.add_argument('--display-seconds', type=float, default=3.0, metavar='SEC',
                   help='궤적 재생 1회에 걸릴 시간(초). **모든 목표를 이 길이로 '
                        '통일**한다 — 계획 궤적의 원래 길이는 목표마다 2.8~21.4초로 '
                        '널뛰어서 그대로 두면 속도 비교가 안 된다. 크게 줄수록 느리다. '
                        '0이면 계획된 시간 그대로. '
                        'RViz의 Trajectory > State Display Time이 REALTIME이어야 한다')
    p.add_argument('--display-repeats', type=int, default=3, metavar='N',
                   help='목표마다 궤적을 몇 번 반복 재생할지. '
                        '정지 시간 = --display-seconds x 이 값')
    p.add_argument('--label-scale', type=float, default=None, metavar='M',
                   help='화면 라벨 글자 높이(m). 기본 0.012. 월드 좌표라 크게 주면 '
                        '글자가 씬을 덮는다')
    p.add_argument('--label-pos', nargs=3, type=float, default=None,
                   metavar=('X', 'Y', 'Z'),
                   help='화면 라벨 위치(g_base, m). 기본은 베드 옆구리 '
                        '(0.26, -0.21, 0.30)')
    p.add_argument('--display-pause', type=float, default=0.0,
                   help='>0이면 목표마다 계획 궤적을 /display_planned_path로 '
                        '발행하고 이 초만큼 대기 — RViz 확인·영상 녹화용. '
                        '권장 3~5초(궤적 재생 시간보다 넉넉히)')
    p.add_argument('--start-pose', nargs=6, type=float, default=None,
                   metavar=('J1', 'J2', 'J3', 'J4', 'J5', 'J6'),
                   help='시작 자세를 **도 단위** 6개로 지정(기본: look pose). '
                        'armed pos 후보 평가용 — 노드는 고치지 않아도 된다')
    # [2026-08-02] 세 가지 수확 사이클을 이름으로 고른다. 값과 이름의 단일
    # 출처는 노드의 WAITING_POSES다 — 여기서 관절값을 따로 적으면 노드와
    # 어긋나는 순간 화면과 수치가 조용히 갈린다.
    p.add_argument('--cycle', choices=sorted(
                       set(N.CYCLE_ALIASES) | set(N.WAITING_POSES)),
                   default=None,
                   help='수확 사이클 = 시작(대기) 자세. lpc(look) | asc(armed) | '
                        'bsc(통 자세). --start-pose 대신 쓴다')
    # [2026-08-02] 한 사이클을 닫아서 재생한다. 세 방식의 차이가 가장 크게
    # 드러나는 곳이 **복귀 구간**이라, 정렬까지만 보여주면 세 영상이 "출발
    # 자세만 다른 그림"이 된다.
    p.add_argument('--full-cycle', action='store_true',
                   help='대기->정렬->직진->후퇴->복귀를 이어 재생하고 복귀 '
                        '이동량을 같이 찍는다(목표당 플래닝 1회 추가)')
    p.add_argument('--collect-solutions', type=str, default=None,
                   metavar='OUT.json',
                   help='목표별 정렬 관절해를 모아 JSON으로 저장(armed pos 도출용)')
    p.add_argument('--sweep-orientation-deg', nargs='+', type=float, default=None,
                   help='이 값들로 방향 허용오차를 훑으며 성공률 곡선을 만든다'
                        ' (예: --sweep-orientation-deg 2 4 6 8 10 12 15)')
    p.add_argument('--targets', default='bags/lab_bed_detections.json')
    # [2026-08-01] 씬에 장애물을 넣는 두 옵션. 안 주면 **빈 씬**이고, 그러면
    # 뚫고 가는 경로가 성공으로 잡힌다(2절 함정 3 — 세 번 데였다).
    # [2026-08-02] 사용자 지적 — "옆에 있는 토마토도 충돌한다. ACM 완화하면
    # 다른 토마토를 장애물로 인식하지 않는가?" 맞다. 기본은 열매 전부를
    # 완화하므로 이웃 회피를 재지 않는다(함정 17). 이 옵션은 목표마다 그
    # 열매 하나만 완화해 "이웃을 피하려면 무엇을 치러야 하는가"를 잰다.
    p.add_argument('--tomato-acm-target-only', action='store_true',
                   help='목표마다 그 열매만 ACM 완화(나머지는 장애물). '
                        '기본은 열매 전부 완화 — 이웃 회피를 재지 않는다')
    p.add_argument('--tomatoes', action='store_true',
                   help='검출 열매를 구 collision object로 넣고 ACM을 푼다')
    # [2026-08-02] octomap 쪽 ACM 완화. 열매(구)에는 --tomatoes가 이미 걸어
    # 주는데 octomap에는 안 걸고 있었다. 이걸 켜고 잰 값과 끄고 잰 값을
    # **섞어 보고하는 사고**를 한 번 냈으므로, 반드시 옵션으로 드러나게 둔다.
    p.add_argument('--octomap-acm', action='store_true',
                   help='그리퍼/손목/flange가 octomap voxel과 충돌해도 되게 한다. '
                        '팔뚝(joint2~4)은 그대로 금지. '
                        '실측: 정렬 70->100%%, 직진 fraction>=0.95가 3/13->14/15')
    p.add_argument('--octomap', default=None, metavar='FILE',
                   help='녹화 장면 octomap을 주입한다(octomap_io.py capture 결과). '
                        '줄기·지지대·잎이 여기 들어 있다')
    p.add_argument('--repeat', type=int, default=5,
                   help='목표당 반복 횟수. OMPL이 확률적이라 신뢰도 측정에 필요')
    p.add_argument('--planning-time', type=float, default=2.0)
    p.add_argument('--attempts', type=int, default=10)
    p.add_argument('--csv', default=None)
    # [2026-08-01] 허용오차를 인자로 뺀 이유: 첫 실행에서 15개 중 5개가 5회 전부
    # 실패했는데, 그게 "진짜 도달 불가"인지 "내가 준 허용오차가 빡빡해서"인지
    # 구분해야 결과를 신뢰할 수 있다. 값을 바꿔 재실행하는 민감도 확인용.
    p.add_argument('--position-tolerance', type=float, default=POSITION_TOLERANCE_M)
    p.add_argument('--orientation-tolerance', type=float,
                   default=ORIENTATION_TOLERANCE_RAD)
    args = p.parse_args()

    POSITION_TOLERANCE_M = args.position_tolerance
    ORIENTATION_TOLERANCE_RAD = args.orientation_tolerance
    if args.standoff is not None:
        # select_approach가 N._waypoints_along_forward를 거쳐 이 상수를 읽는다.
        print(f'standoff 덮어쓰기: {N.APPROACH_STANDOFF_M*1000:.0f}mm -> '
              f'{args.standoff*1000:.0f}mm')
        N.APPROACH_STANDOFF_M = args.standoff
    print(f'\n허용오차: 위치 {POSITION_TOLERANCE_M*1000:.0f}mm, '
          f'방향 {math.degrees(ORIENTATION_TOLERANCE_RAD):.1f}°')

    with open(args.targets, encoding='utf-8') as fh:
        dets = json.load(fh)['detections']

    # [2026-08-02] --cycle은 --start-pose의 이름 붙은 버전이다. 둘을 같이 주면
    # **에러로 막는다** — 어느 쪽이 이겼는지 모르는 채로 수치가 나오면 함정 15
    # ("조건을 결과에 반드시 같이 적을 것")를 그대로 다시 밟는다.
    if args.cycle and args.start_pose:
        raise SystemExit('--cycle과 --start-pose는 같이 줄 수 없다 — '
                         '어느 자세로 잰 값인지 모호해진다(함정 15)')
    if args.cycle:
        key = N.CYCLE_ALIASES.get(args.cycle, args.cycle)
        joints, name, short = N.WAITING_POSES[key]
        args.start_pose = [math.degrees(v) for v in joints]
        print(f'사이클: {short} — 대기 자세 {name} '
              f'({", ".join(f"{math.degrees(v):+.1f}" for v in joints)}도)')

    start_pose = ([math.radians(v) for v in args.start_pose]
                  if args.start_pose else None)

    rclpy.init()
    node = PlanProbe(args.planning_time, args.attempts, start_pose)
    if not node.wait_ready():
        node.get_logger().error(
            f'{PLAN_SERVICE} 없음 — move_group이 떠 있는지 확인할 것')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    # 씬 구성. 이 두 줄이 있느냐 없느냐로 수치가 크게 달라지므로 항상 출력한다.
    scene_label = '빈 씬'
    if args.tomatoes or args.octomap:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import scene_objects
        parts = []
        if args.tomatoes:
            scene_objects.publish_tomatoes(node, dets)
            scene_objects.allow_gripper_tomato_collisions(node, dets)
            parts.append(f'열매 {len(dets)}개(구)+ACM')
        if args.octomap:
            import octomap_io
            from moveit_msgs.msg import PlanningScene
            owp = octomap_io.load_octomap_file(args.octomap)
            octomap_io.inject_octomap(node, rclpy, PlanningScene, owp)
            leaves = octomap_io.decode_msg(owp.octomap)
            octomap_io.summarize(leaves, owp.octomap.resolution,
                                 label=f'octomap({args.octomap}): ')
            parts.append(f'octomap voxel {len(leaves)}개')
            # **항상 호출한다(끌 때도).** ACM은 move_group의 planning scene에
            # 붙어 있어 스크립트가 죽어도 남는다. 앞 실행이 kill -9로 끝나
            # teardown을 못 돌면 완화가 그대로 살아 있고, 다음 실행이 그걸
            # 물려받아 **끈 줄 알고 켠 상태로 재는 사고**가 난다(실제로 겪었다).
            # verify 쪽도 ACM은 안 보므로 여기서 명시적으로 맞춰 두는 수밖에 없다.
            scene_objects.allow_gripper_octomap_collisions(
                node, args.octomap_acm, quiet=not args.octomap_acm)
            if args.octomap_acm:
                parts.append('octomap ACM 완화')
            else:
                parts.append('octomap ACM 완화 없음')
        scene_label = ' + '.join(parts)
    print(f'\n씬: {scene_label}')

    def teardown():
        """씬을 원래대로 되돌린다.

        안 되돌리면 **다음 측정이 남은 장애물을 모른 채** 돌아간다 — 이게
        바로 함정 3이 생기는 경로다. 어느 종료 경로로 나가든 지나가게 둔다.
        """
        if args.tomatoes:
            scene_objects.publish_tomatoes(node, dets, remove=True)
        if args.octomap:
            if args.octomap_acm:
                scene_objects.allow_gripper_octomap_collisions(
                    node, False, quiet=True)
            octomap_io.clear_octomap(node, rclpy)
        node.destroy_node()
        rclpy.shutdown()

    print(f'\n목표 {len(dets)}개 x 반복 {args.repeat}회 '
          f'(플래닝 시간 {args.planning_time}s, 시도 {args.attempts}회)')
    # 노드가 쓰는 상수와 대조해 이름을 붙인다. 화면 라벨에 관절값 6개가 뜨면
    # "지금 보고 있는 게 armed pose인가"를 읽어낼 수 없다.
    #
    # [2026-08-02] 대조 대상을 N.WAITING_POSES로 바꿨다 — 예전엔 look/armed만
    # 알아서 통 자세로 돌리면 라벨에 관절값 6개가 그대로 떴다. 그리고 화면용과
    # 터미널용을 나눈다: **RViz 라벨은 ASCII만 쓸 수 있다**(함정 14 — 폰트
    # 아틀라스가 33~166만 담아서 한글도 `°`도 안 보이는데 자리는 차지한다).
    def _pose_label(start_pose_deg):
        """(화면용 ASCII, 터미널용) 짝을 돌려준다."""
        rad = ([math.radians(v) for v in start_pose_deg] if start_pose_deg
               else list(N.LOOK_POSE_JOINT_POSITIONS))
        for key, (joints, name, short) in N.WAITING_POSES.items():
            if all(abs(a - b) < 1e-3 for a, b in zip(rad, joints)):
                return f'{short}:{key}', f'{name} ({short})'
        joints_txt = '_'.join(f'{v:+.0f}' for v in start_pose_deg)
        return joints_txt, ('[' + ', '.join(f'{v:+.1f}' for v in start_pose_deg)
                            + ']도')

    pose_label, pose_label_full = _pose_label(args.start_pose)
    if args.label_scale is not None:
        node.LABEL_SCALE = args.label_scale
    if args.label_pos is not None:
        node.LABEL_ANCHOR = tuple(args.label_pos)
    span = ('시작 자세 -> 정렬 -> 직진 -> 후퇴 -> 복귀(한 사이클)'
            if args.full_cycle else '시작 자세 -> 정렬 위치')
    print(f'시작 자세: {pose_label_full} 고정 / 재생·측정 구간: {span}\n')

    if args.collect_solutions:
        collect_solutions(node, dets, args.repeat, args.collect_solutions)
        teardown()
        return

    if args.sweep_orientation_deg:
        run_tolerance_sweep(node, dets, args.repeat,
                            args.sweep_orientation_deg, args.csv)
        teardown()
        return

    if args.display_pause > 0:
        # 시작 시 전체 토마토를 한 번 그려 두고 잠깐 기다린다 — RViz 구독이
        # 붙기 전에 쏘면 아무것도 안 보인다(DDS 디스커버리).
        for _ in range(40):
            node.publish_markers(dets)
            rclpy.spin_once(node, timeout_sec=0.05)

        round_no = 0
        try:
            while args.display_loop == 0 or round_no < args.display_loop:
                round_no += 1
                label = (f'{round_no}회차' if args.display_loop == 0
                         else f'{round_no}/{args.display_loop}회차')
                print(f'\n===== {label} =====')
                rows = evaluate_all(node, dets, args.repeat,
                                    display_pause=args.display_pause,
                                    pose_label=pose_label,
                                    display_seconds=args.display_seconds,
                                    display_repeats=args.display_repeats,
                                    roll_symmetry=args.roll_symmetry,
                                    straight_in=args.straight_in,
                                    acm_only_target=args.tomato_acm_target_only,
                                    full_cycle=args.full_cycle)
                # 다음 회차 전에 마커 색을 초기화한다 — 안 그러면 전부 초록/회색인
                # 채로 시작해 "지금 어디를 보고 있는지"가 안 보인다.
                node.publish_markers(dets)
        except KeyboardInterrupt:
            print(f'\n중단 — {round_no}회차까지 실행함')
    else:
        rows = evaluate_all(node, dets, args.repeat,
                            roll_symmetry=args.roll_symmetry,
                            straight_in=args.straight_in,
                            acm_only_target=args.tomato_acm_target_only,
                            full_cycle=args.full_cycle)

    planned = [r for r in rows if r.get('trials')]
    total_trials = sum(r['trials'] for r in planned)
    total_ok = sum(r['success'] for r in planned)
    print(f'\n=== 요약 ===')
    print(f'플래닝 시도 {total_ok}/{total_trials} 성공 '
          f'({100*total_ok/total_trials:.1f}%)' if total_trials else '시도 없음')
    full = [r for r in planned if r['success'] == r['trials']]
    partial = [r for r in planned if 0 < r['success'] < r['trials']]
    never = [r for r in planned if r['success'] == 0]
    print(f'  전회 성공 {len(full)}개 / 간헐 성공 {len(partial)}개 / 전회 실패 {len(never)}개')

    j1_all = [r['j1_mean_deg'] for r in planned if r['j1_mean_deg'] is not None]
    if j1_all:
        # docs/ROSBAG_HANDOFF.md 4절의 A/B 분기 판별. A는 30~50°, B는 120~140°였다.
        a = [v for v in j1_all if v < 80]
        b = [v for v in j1_all if v >= 80]
        print(f'  J1 도달각 A무리(<80°) {len(a)}개, B무리(>=80°) {len(b)}개')
        if a:
            print(f'    A: {min(a):.1f}~{max(a):.1f}° (평균 {statistics.fmean(a):.1f})')
        if b:
            print(f'    B: {min(b):.1f}~{max(b):.1f}° (평균 {statistics.fmean(b):.1f}) '
                  f'<- 뒤로 감는 분기')

    # [2026-08-02] 세 사이클 비교용. 정렬 이동량은 목표별 평균의 중앙값이고
    # 복귀는 구간별 최소 1회이므로 **집계가 다르다** — 문서 6.1절의 "목표별
    # 최소의 중앙값"과 섞어 인용하지 말 것(함정 3·15).
    ret_all = [r['return_travel_deg'] for r in planned
               if r.get('return_travel_deg') is not None]
    if ret_all:
        print(f'  복귀(정렬 -> {pose_label_full}) 중앙 '
              f'{statistics.median(ret_all):.0f}° / 합 {sum(ret_all):.0f}° '
              f'({len(ret_all)}개 목표)')

    if args.csv:
        import csv
        keys = sorted({k for r in rows for k in r})
        with open(args.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=keys, extrasaction='ignore')
            w.writeheader()
            w.writerows(rows)
        print(f'\nCSV 저장: {args.csv} ({len(rows)}행)')

    teardown()


if __name__ == '__main__':
    main()
