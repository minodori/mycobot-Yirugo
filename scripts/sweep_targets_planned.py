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
장애물 회피까지 평가하려면 얼린 octomap을 주입한 뒤 `--with-octomap`으로 돌릴 것
(미구현).
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
        MotionPlanRequest,
        OrientationConstraint,
        PositionConstraint,
        RobotState,
        WorkspaceParameters,
    )
    from moveit_msgs.srv import GetMotionPlan
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from mycobot_280_pick import coord_to_goal_node as N
except ImportError as exc:  # pragma: no cover
    sys.exit(f'import 실패: {exc}\n  source /opt/ros/jazzy/setup.bash 를 먼저 할 것')

WRIST_LATERAL_OFFSET_M = 0.0732
SHOULDER = [0.0, 0.0, N.SHOULDER_HEIGHT_M]
PLAN_SERVICE = '/plan_kinematic_path'

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

    def __init__(self, planning_time, attempts):
        super().__init__('sweep_targets_planned')
        self._client = self.create_client(GetMotionPlan, PLAN_SERVICE)
        self._planning_time = planning_time
        self._attempts = attempts
        self._limits = None
        self.create_subscription(JointState, 'joint_states', self._on_js, 10)
        self._seen_joints = None

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
        """모든 목표를 **같은 시작 자세**(look pose)에서 플래닝한다.

        실제 스택의 현재 관절값을 쓰면 앞 목표의 결과에 따라 시작이 달라져
        목표 간 비교가 불가능해진다. 명시적으로 고정한다.
        """
        state = RobotState()
        state.joint_state.name = list(N.JOINT_NAMES)
        state.joint_state.position = list(N.LOOK_POSE_JOINT_POSITIONS)
        state.is_diff = False
        return state

    def plan_to(self, position, quat):
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

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = N.END_EFFECTOR_NAME
        pc.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [POSITION_TOLERANCE_M]
        volume = BoundingVolume()
        volume.primitives = [sphere]
        volume.primitive_poses = [pose.pose]
        pc.constraint_region = volume

        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = N.END_EFFECTOR_NAME
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        oc.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        oc.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        oc.weight = 1.0

        goal = Constraints()
        goal.position_constraints = [pc]
        goal.orientation_constraints = [oc]
        req.goal_constraints = [goal]

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
            'wall_s': elapsed,
            'plan_s': res.planning_time,
            'points': len(traj.points),
            'j1_deg': math.degrees(j1_final),
            'travel_deg': math.degrees(sum(travel)),
            'final': {n: final[idx[n]] for n in names},
            'peak': {n: max(abs(p.positions[idx[n]]) for p in traj.points) for n in names},
        }


def main():
    global POSITION_TOLERANCE_M, ORIENTATION_TOLERANCE_RAD
    p = argparse.ArgumentParser(description='검출 토마토별 실제 플래닝 평가')
    p.add_argument('--targets', default='bags/lab_bed_detections.json')
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
    print(f'\n허용오차: 위치 {POSITION_TOLERANCE_M*1000:.0f}mm, '
          f'방향 {math.degrees(ORIENTATION_TOLERANCE_RAD):.1f}°')

    with open(args.targets, encoding='utf-8') as fh:
        dets = json.load(fh)['detections']

    rclpy.init()
    node = PlanProbe(args.planning_time, args.attempts)
    if not node.wait_ready():
        node.get_logger().error(
            f'{PLAN_SERVICE} 없음 — move_group이 떠 있는지 확인할 것')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    print(f'\n목표 {len(dets)}개 x 반복 {args.repeat}회 '
          f'(플래닝 시간 {args.planning_time}s, 시도 {args.attempts}회)')
    print(f'시작 자세: look pose 고정 / 측정 구간: look pose -> 정렬 위치\n')

    rows = []
    for d in dets:
        best, reason = select_approach(d['base_x'], d['base_y'], d['base_z'])
        label = f'{d.get("class_name", "?"):<8} z={d["base_z"]:.3f}'
        if best is None:
            print(f'{label}  기하 게이트 탈락({reason}) — 플래닝 생략')
            rows.append({**d, 'reason': reason, 'success': 0, 'trials': 0})
            continue

        results = [node.plan_to(best['align'], best['quat']) for _ in range(args.repeat)]
        good = [r for r in results if r['ok']]
        j1 = [r['j1_deg'] for r in good]
        travel = [r['travel_deg'] for r in good]
        plan_s = [r['plan_s'] for r in good]

        row = {
            **{k: d[k] for k in ('class_name', 'base_x', 'base_y', 'base_z') if k in d},
            'reason': 'ok',
            'trials': args.repeat,
            'success': len(good),
            'elev_deg': round(best['elev_deg'], 1),
            'pred_j1_deg': round(best['min_j1_deg'], 1),
            'j1_mean_deg': round(statistics.fmean(j1), 1) if j1 else None,
            'j1_min_deg': round(min(j1), 1) if j1 else None,
            'j1_max_deg': round(max(j1), 1) if j1 else None,
            'travel_mean_deg': round(statistics.fmean(travel), 0) if travel else None,
            'travel_sd_deg': round(statistics.pstdev(travel), 0) if len(travel) > 1 else 0,
            'plan_s_mean': round(statistics.fmean(plan_s), 3) if plan_s else None,
        }
        rows.append(row)
        j1_txt = (f'J1 {row["j1_min_deg"]:.0f}~{row["j1_max_deg"]:.0f}°'
                  if j1 else 'J1 —')
        print(f'{label}  성공 {len(good)}/{args.repeat}  {j1_txt}  '
              f'이동량 {row["travel_mean_deg"] or 0:.0f}°±{row["travel_sd_deg"]:.0f}  '
              f'플래닝 {row["plan_s_mean"] or 0:.2f}s')

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

    if args.csv:
        import csv
        keys = sorted({k for r in rows for k in r})
        with open(args.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=keys, extrasaction='ignore')
            w.writeheader()
            w.writerows(rows)
        print(f'\nCSV 저장: {args.csv} ({len(rows)}행)')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
