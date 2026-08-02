#!/usr/bin/env python3
"""armed pose를 **작업공간에서** 정하고 IK로 자세를 구한다 (재도출).

왜 다시 하나 — 첫 시도(find_armed_pose.py)의 실패
------------------------------------------------
첫 시도는 15개 목표의 정렬 관절해를 모아 **관절공간 minimax 중심**을 구했다.
그런데 관절공간의 중심이 작업공간에서 안전한 위치라는 보장이 전혀 없다.
실제로 FK로 확인해보니 flange가 (+0.194, -0.024, +0.306), 반경 0.195m —
**가장 가까운 토마토까지 7.3cm**였다. 그리퍼가 flange에서 9cm 더 뻗으므로
손가락 끝은 이미 열매 사이에 들어가 있는 자세다(look pose는 38.9cm 떨어져 있다).

원인은 목적함수다. "모든 목표까지 관절 이동량 최소"는 제약이 없으면
**목표 한가운데로 파고드는 것**이 최적해다. 게다가 그 측정에는 베드도
토마토도 collision object가 없어서, 뚫고 가는 경로가 전부 성공으로 집계됐다.

이번 방식
---------
1. 토마토 15개를 **구 collision object로 씬에 넣는다** (검출 반지름 + 여유).
2. armed pose 후보를 **작업공간 격자**(반경 x 방위 x 높이)로 만든다.
   그리퍼는 베드 쪽(반경 바깥, 수평)을 향하게 한다 — 정렬 자세와 방향이
   비슷해야 이동이 적다.
3. 후보마다 (a) 도달 가능한가 (b) 토마토와 여유가 충분한가를 먼저 거른다.
4. 살아남은 후보에서 15개 목표 정렬 위치로 플래닝해 성공률·이동량을 잰다.
5. 두 가지를 같이 본다:
     단일    모든 목표에 공통으로 쓰는 armed pose 1개
     타겟별  목표마다 가장 좋은 armed pose (docs/new_concept.md 옵션2)

한계 (반드시 알고 볼 것)
-----------------------
씬에 넣는 것은 **토마토 열매뿐**이다. 줄기·지지대·잎은 형상을 모르므로 빠져
있다. 즉 여기 수치는 낙관적이다.

**[2026-08-01] 그 재확인을 했다** — `eval_bed_scene.py`가 녹화 장면의 octomap을
주입해 다시 쟀고, 결과는 `docs/ARMED_POSE_HANDOFF.md` 5절이다. 요약:
채택한 armed pose의 **상대 우위는 유지된다**(look pose 대비 -9%). 다만 성공률이
100%에서 77.8%로 떨어지므로, **여기서 나오는 "성공률 100%"는 이 씬 한정**이라는
점을 기억할 것. 후보를 고르는 용도로는 여전히 유효하지만(모든 후보가 같은 씬에서
평가되므로 순위는 공정하다), 절대값을 인용하면 안 된다.

사용법
------
    ros2 launch mycobot_280_moveit2 demo_octomap.launch.py \
        enable_octomap:=false enable_camera:=false use_rviz:=false
    python3 scripts/find_armed_pose_cartesian.py --repeat 3
"""

import argparse
import json
import math
import os
import statistics as st
import sys

# 파이프로 넘길 때 출력이 블록 버퍼링되면 진행 상황이 안 보인다.
# 후보 수십 개 x 목표 15개라 수십 분이 걸리므로 반드시 즉시 출력해야 한다.
import functools
print = functools.partial(print, flush=True)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
import sweep_targets_planned as S
from sweep_targets_planned import PlanProbe, select_approach
from mycobot_280_pick import coord_to_goal_node as N

# 씬을 만드는 코드는 scene_objects로 옮겼다 — octomap 재검증
# (eval_bed_scene.py)도 **똑같은 씬**을 만들어야 두 측정이 비교되기 때문이다.
# 이름은 그대로 두어 기존 호출부가 깨지지 않게 한다.
from scene_objects import (  # noqa: F401
    TOMATO_MARGIN_M,
    TOMATO_OBJECT_PREFIX,
    allow_gripper_tomato_collisions,
    publish_tomatoes,
    tip_clearance,
)

# armed pose가 지켜야 할 최소 여유(m). flange 기준이 아니라 **그리퍼 끝단**
# 기준이다 — 손가락이 flange에서 GRIPPER_LENGTH_OFFSET_M(9cm) 앞으로 나간다.
MIN_TIP_CLEARANCE_M = 0.08


def candidate_pose(radius, azimuth_deg, height, elevation_deg=0.0):
    """작업공간 좌표 -> (flange 위치, orientation).

    그리퍼는 반경 바깥(베드 쪽)을 향하게 한다 — coord_to_goal_node가 정렬에
    쓰는 것과 같은 look-at 규약이라 방향이 비슷해야 이동이 적다.
    """
    az = math.radians(azimuth_deg)
    pos = (radius * math.cos(az), radius * math.sin(az), height)
    fwd = N._forward_unit_vector(az, math.radians(elevation_deg))
    return pos, N._compute_look_at_quat_xyzw(fwd)


def fk_flange(node, joints):
    """관절값 -> flange 위치(g_base). move_group의 /compute_fk를 쓴다."""
    from moveit_msgs.srv import GetPositionFK
    from moveit_msgs.msg import RobotState
    from sensor_msgs.msg import JointState
    if not hasattr(node, '_fk_client'):
        node._fk_client = node.create_client(GetPositionFK, '/compute_fk')
        node._fk_client.wait_for_service(timeout_sec=10)
    req = GetPositionFK.Request()
    req.header.frame_id = N.BASE_LINK_NAME
    req.fk_link_names = [N.END_EFFECTOR_NAME]
    rs = RobotState()
    rs.joint_state = JointState()
    rs.joint_state.name = list(N.JOINT_NAMES)
    rs.joint_state.position = list(joints)
    req.robot_state = rs
    fut = node._fk_client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    res = fut.result()
    if res is None or not res.pose_stamped:
        return None
    p = res.pose_stamped[0].pose.position
    q = res.pose_stamped[0].pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def tip_clearance_from_joints(pos7, joints, dets):
    """FK 결과(위치+자세)에서 그리퍼 끝단 여유를 구한다."""
    return tip_clearance(pos7[:3], pos7[3:], dets)


def main():
    p = argparse.ArgumentParser(description='armed pose 작업공간 기반 재도출')
    p.add_argument('--targets', default='bags/lab_bed_detections.json')
    p.add_argument('--repeat', type=int, default=3)
    # look pose 기하를 유지한 채 J1을 훑는다. J1 한계는 +-168도라 여유가 크다.
    p.add_argument('--j1', nargs='+', type=float,
                   default=[-20, -10, 0, 10, 20, 30, 40, 50, 60, 70])
    # J2/J3를 조금 흔들어 "더 접거나 편" 변형도 본다(도).
    p.add_argument('--dj2', nargs='+', type=float, default=[-10, 0, 10])
    p.add_argument('--dj3', nargs='+', type=float, default=[-10, 0, 10])
    p.add_argument('--max-flange-radius', type=float, default=None,
                   help='flange가 베이스에서 이 반경 안에 있는 후보만 쓴다(m). '
                        '베드가 250mm, 토마토가 튀어나와 실질 220mm이므로 '
                        '팔을 최대한 접어두고 싶을 때 사용')
    p.add_argument('--orientation-tolerance', type=float, default=0.2)
    p.add_argument('--json-out', default=None)
    args = p.parse_args()

    S.ORIENTATION_TOLERANCE_RAD = args.orientation_tolerance
    with open(args.targets, encoding='utf-8') as fh:
        dets = json.load(fh)['detections']

    rclpy.init()
    node = PlanProbe(2.0, 10)
    if not node.wait_ready():
        raise SystemExit('move_group 없음')

    print(f'\n토마토 {len(dets)}개를 구 collision object로 씬에 추가 '
          f'(검출 반지름 + {TOMATO_MARGIN_M*100:.0f}cm 여유)')
    publish_tomatoes(node, dets)
    allow_gripper_tomato_collisions(node, dets)

    # 각 목표의 정렬 위치·자세를 미리 구해 둔다(모든 후보에 공통).
    goals = []
    for d in dets:
        best, reason = select_approach(d['base_x'], d['base_y'], d['base_z'])
        if best is not None:
            goals.append((d, best))
    print(f'기하 게이트 통과 목표 {len(goals)}/{len(dets)}개\n')

    # --- 1단계: 후보 생성 + 여유/도달 필터 ---
    #
    # [2026-08-01, 방향 수정] 처음엔 "flange를 반경 r, 방위 az, 높이 h에 놓고
    # 그리퍼가 베드를 향하게" 하는 작업공간 격자를 썼는데, 36개 중 34개가 여유
    # 부족으로 탈락하고 나머지 2개도 도달 불가였다. 이유가 구조적이다 —
    # 그리퍼가 베드를 향하면 끝단이 반경+9cm가 되어 토마토(반경 0.248~0.293)
    # 한가운데로 들어간다.
    #
    # look pose가 왜 안전한지 보면 답이 나온다: flange 방위각이 **-165.7도**,
    # 즉 팔이 베이스 뒤로 접혀 있고 카메라만 앞을 본다. 안전한 대기 자세는
    # "베드 쪽으로 뻗은" 것이 아니라 **접힌** 자세다.
    #
    # 그래서 후보를 "look pose의 접힌 기하를 유지하되 J1(+선택적으로 J2/J3)만
    # 바꾼" 족으로 만든다. 안전(여유)은 접힌 기하가 보장하고, 이득(J1 스윙
    # 감소)은 J1 회전이 만든다. 각 후보의 flange 위치와 끝단 여유를 FK 대신
    # 플래닝 결과로 확인하므로 작업공간 검증은 그대로 유지된다.
    cands = []
    for j1_deg in args.j1:
        for dj2 in args.dj2:
            for dj3 in args.dj3:
                q = list(N.LOOK_POSE_JOINT_POSITIONS)
                q[0] = math.radians(j1_deg)
                q[1] += math.radians(dj2)
                q[2] += math.radians(dj3)
                cands.append({'j1': j1_deg, 'dj2': dj2, 'dj3': dj3, 'joints': q})
    print(f'후보 {len(cands)}개 (look pose 기하 유지, J1/J2/J3 변형)')

    survivors = []
    for i, c in enumerate(cands, 1):
        print(f'  후보 {i}/{len(cands)}  J1 {c["j1"]:+.0f} dJ2 {c["dj2"]:+.0f} '
              f'dJ3 {c["dj3"]:+.0f} ... ', end='')
        # 후보 자세로 실제로 갈 수 있는지 + 그 자세가 충돌하지 않는지를
        # 플래닝으로 확인한다(토마토가 씬에 있으므로 충돌이면 실패한다).
        node.start_pose = list(N.LOOK_POSE_JOINT_POSITIONS)
        reach = node.plan_to_config(c['joints'])
        if not reach['ok']:
            print('도달 불가')
            continue
        c['pos'] = fk_flange(node, c['joints'])
        if c['pos'] is None:
            print('FK 실패')
            continue
        c['clear'] = tip_clearance_from_joints(c['pos'], c['joints'], dets)
        if c['clear'] < MIN_TIP_CLEARANCE_M:
            print(f'여유 {c["clear"]*100:.1f}cm 부족')
            continue
        c['flange_r'] = math.hypot(c['pos'][0], c['pos'][1])
        if args.max_flange_radius is not None and c['flange_r'] > args.max_flange_radius:
            print(f'flange 반경 {c["flange_r"]:.3f}m 초과')
            continue
        print(f'OK (여유 {c["clear"]*100:.1f}cm, flange 반경 {c["flange_r"]:.3f}m)')
        survivors.append(c)
    print(f'도달 가능 + 끝단 여유 {MIN_TIP_CLEARANCE_M*100:.0f}cm 이상: '
          f'{len(survivors)}개\n')
    if not survivors:
        raise SystemExit('조건을 만족하는 후보가 없다 — 격자나 여유 기준을 조정할 것')

    # --- 2단계: 살아남은 후보를 15개 목표로 평가 ---
    print(f'{"J1":>6}{"dJ2":>6}{"dJ3":>6}{"여유cm":>8}{"반경m":>7}'
          f'{"성공":>9}{"이동량 중앙":>12}{"최악":>8}')
    for c in survivors:
        # **이 후보를 시작 자세로 놓고** 평가해야 한다. 안 그러면 전부 look
        # pose에서 잰 값이 되어 후보 비교가 무의미해진다.
        node.start_pose = c['joints']
        per_target = []
        ok_total = trials = 0
        for d, best in goals:
            vals = []
            for _ in range(args.repeat):
                trials += 1
                r = node.plan_to(best['align'], best['quat'])
                if r['ok']:
                    ok_total += 1
                    vals.append(r['travel_deg'])
            # 반복 중 최소를 대표로 — Phase 3(N개 중 최소 선택)이 적용된
            # 상태를 가정한 값이다. 후보 간 비교에는 이쪽이 공정하다.
            per_target.append(min(vals) if vals else None)
        got = [v for v in per_target if v is not None]
        c['per_target'] = per_target
        c['success'] = ok_total / trials if trials else 0.0
        c['median'] = st.median(got) if got else float('nan')
        c['worst'] = max(got) if got else float('nan')
        print(f'{c["j1"]:6.0f}{c["dj2"]:+6.0f}{c["dj3"]:+6.0f}{c["clear"]*100:8.1f}'
              f'{c["flange_r"]:7.3f}{100*c["success"]:8.0f}%'
              f'{c["median"]:12.0f}{c["worst"]:8.0f}')

    # --- 3단계: 단일 armed pose vs 타겟별 armed pose ---
    usable = [c for c in survivors if c['success'] > 0.99]
    print()
    if not usable:
        print('모든 목표에 100% 성공하는 후보가 없다 — 아래는 성공률 포함 비교')
        usable = survivors

    single = min(usable, key=lambda c: c['worst'])
    print(f'[단일 armed pose] 최악 이동량 최소 기준')
    print(f'  J1 {single["j1"]:+.0f}도, dJ2 {single["dj2"]:+.0f}, dJ3 {single["dj3"]:+.0f}'
          f'  (끝단 여유 {single["clear"]*100:.1f}cm, '
          f'flange 반경 {math.hypot(single["pos"][0], single["pos"][1]):.3f}m)')
    print(f'  관절(도) ' + '  '.join(f'{math.degrees(v):+7.2f}' for v in single['joints']))
    print(f'  목표별 이동량 중앙 {single["median"]:.0f}, 최악 {single["worst"]:.0f}')

    # 타겟별로 가장 좋은 후보를 고르면 얼마나 더 줄어드는가(new_concept 옵션2)
    per_best, chosen = [], {}
    for i, (d, _) in enumerate(goals):
        vals = [(c['per_target'][i], c) for c in usable
                if c['per_target'][i] is not None]
        if not vals:
            continue
        v, c = min(vals, key=lambda t: t[0])
        per_best.append(v)
        key = (c['j1'], c['dj2'], c['dj3'])
        chosen[key] = chosen.get(key, 0) + 1
    print(f'\n[타겟별 armed pose] docs/new_concept.md 옵션2')
    print(f'  이동량 중앙 {st.median(per_best):.0f}, 최악 {max(per_best):.0f}')
    print(f'  단일 대비 중앙 {100*(st.median(per_best)-single["median"])/single["median"]:+.1f}%, '
          f'최악 {100*(max(per_best)-single["worst"])/single["worst"]:+.1f}%')
    print(f'  실제로 쓰인 서로 다른 자세 {len(chosen)}개:')
    for (j1, dj2, dj3), n in sorted(chosen.items(), key=lambda t: -t[1]):
        print(f'    J1 {j1:+.0f}도 dJ2 {dj2:+.0f} dJ3 {dj3:+.0f} -> {n}개 목표')

    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            json.dump({
                'single': {k: single[k] for k in
                           ('j1', 'dj2', 'dj3', 'clear', 'median', 'worst')} |
                          {'joints_rad': [round(v, 6) for v in single['joints']],
                           'joints_deg': [round(math.degrees(v), 3)
                                          for v in single['joints']]},
                'per_target_median': round(st.median(per_best), 1),
                'per_target_worst': round(max(per_best), 1),
                'n_distinct_poses': len(chosen),
            }, fh, indent=2, ensure_ascii=False)
        print(f'\n저장: {args.json_out}')

    publish_tomatoes(node, dets, remove=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
