#!/usr/bin/env python3
"""그리퍼 **180도 대칭**을 목표 제약에 반영하면 얼마나 이득인가.

무엇을 묻는 스크립트인가
------------------------
RViz 스윕을 보면 목표로 진입할 때 flange(J6)가 자기 축 둘레로 **180도 도는**
목표가 여럿 있다. 그런데 이 그리퍼는 2지 평행 그리퍼라 **roll과 roll+180도는
물리적으로 같은 자세**다(손가락 두 개가 자리를 맞바꿀 뿐이다). 즉 그 회전은
아무것도 바꾸지 않으면서 J6만 반 바퀴 돌린다.

왜 그런 일이 생기냐면, 목표를 **완전한 orientation**으로 주기 때문이다.
`_compute_look_at_quat_xyzw`는 그리퍼 정면(+Z)을 목표로 향하게 하면서 roll을
WORLD_UP 기준으로 **하나로 고정**한다. 플래너는 그리퍼가 대칭이라는 사실을
모르므로, 고정된 그 roll을 맞추려고 J6를 180도 돌리는 해라도 그대로 낸다.
"0도나 180도나 IK 측면에서 같지 않나"라는 질문의 답은 —
**물리적으로는 같고, 제약 조건상으로는 다르다.** 그래서 낭비가 생긴다.

여기서 재는 것
--------------
목표마다 두 가지로 플래닝해서 싼 쪽을 고르면 얼마나 줄어드는가.

    기준(roll 0)      지금 스윕이 하는 것. roll을 WORLD_UP 값 하나로 고정
    대칭(roll 0/180)  roll 0과 roll 180 둘 다 풀어보고 이동량이 적은 쪽

**yaw 자유(tolerance = pi)와는 다르다.** 그건 roll을 완전히 풀어 손가락이
어느 각도로도 들어오게 하는 것이라 대칭이 보장하는 범위를 넘어선다
(수확통 놓기에는 맞는 조건이다 — 거기선 방향이 정말 상관없다). 여기서는
**정확히 180도 두 값만** 본다. 그래야 "물리적으로 동일한 자세끼리의 선택"이다.

노드(`coord_to_goal_node`)와의 관계
-----------------------------------
노드는 이미 roll 후보를 12개(30도 간격) 훑는다(`ROLL_CANDIDATES_RAD`). 그
목록에는 i와 i+6이 180도 짝으로 들어 있으므로 **원리적으로는 싼 쪽을 고를 수
있다.** 다만 그 탐색의 1차 기준은 "IK가 풀리는가"이고, 스윕 스크립트는 그
탐색을 아예 재현하지 않는다(roll 0 고정 — docs/ARMED_POSE_HANDOFF.md 함정 5).
즉 화면에서 본 180도 회전은 **스윕 쪽 제약이 만든 것**일 수 있다. 이 스크립트는
그 차이가 실제로 얼마인지를 수치로 갈라 준다.

사용법
------
    python3 -u scripts/eval_roll_symmetry.py --repeat 10 \\
        --tomatoes --octomap bags/bed_look_octomap.bin \\
        --start-pose 40 104.06 -56.68 -50 15.99 1.4
"""

import argparse
import csv
import functools
import json
import math
import os
import statistics as st
import sys

print = functools.partial(print, flush=True)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from moveit_msgs.msg import PlanningScene

import octomap_io
import scene_objects
import sweep_targets_planned as S
from sweep_targets_planned import PlanProbe, select_approach
from mycobot_280_pick import coord_to_goal_node as N

J6 = N.JOINT_NAMES[5]          # 'joint6output_to_joint6' — flange 자전축


def best_of(results):
    good = [r for r in results if r['ok']]
    return min(good, key=lambda r: r['travel_deg']) if good else None


def main():
    p = argparse.ArgumentParser(description='그리퍼 180도 대칭의 이득 측정')
    p.add_argument('--targets', default='bags/lab_bed_detections.json')
    p.add_argument('--repeat', type=int, default=10)
    p.add_argument('--tomatoes', action='store_true')
    p.add_argument('--octomap', default=None)
    p.add_argument('--start-pose', nargs=6, type=float, default=None)
    p.add_argument('--orientation-tolerance', type=float, default=0.2)
    p.add_argument('--planning-time', type=float, default=2.0)
    p.add_argument('--attempts', type=int, default=10)
    p.add_argument('--csv', default=None)
    args = p.parse_args()

    S.ORIENTATION_TOLERANCE_RAD = args.orientation_tolerance
    with open(args.targets, encoding='utf-8') as fh:
        dets = json.load(fh)['detections']

    rclpy.init()
    start = ([math.radians(v) for v in args.start_pose]
             if args.start_pose else None)
    node = PlanProbe(args.planning_time, args.attempts, start)
    if not node.wait_ready():
        raise SystemExit('move_group 없음')

    scene_objects.publish_tomatoes(node, dets, remove=True)
    octomap_io.clear_octomap(node, rclpy)
    if args.tomatoes:
        scene_objects.publish_tomatoes(node, dets)
        scene_objects.allow_gripper_tomato_collisions(node, dets, quiet=True)
    if args.octomap:
        owp = octomap_io.load_octomap_file(args.octomap)
        octomap_io.inject_octomap(node, rclpy, PlanningScene, owp)
        print(f'씬: 열매 {len(dets) if args.tomatoes else 0}개 + octomap voxel '
              f'{len(octomap_io.decode_msg(owp.octomap))}개')

    pose = ('look pose' if not args.start_pose
            else '[' + ', '.join(f'{v:+.0f}' for v in args.start_pose) + ']')
    print(f'\n시작 자세 {pose} / 목표당 반복 {args.repeat}회 x roll 2종\n')
    print(f'{"z":>6}{"클래스":>9}{"roll0 이동량":>13}{"roll0 J6":>10}'
          f'{"roll180 이동량":>15}{"roll180 J6":>12}{"선택":>7}{"절감":>8}')

    rows = []
    for d in dets:
        best, reason = select_approach(d['base_x'], d['base_y'], d['base_z'])
        if best is None:
            continue
        quats = {
            0: best['quat'],
            180: N._compute_look_at_quat_xyzw(best['fwd'], roll_rad=math.pi),
        }
        out = {}
        for roll, q in quats.items():
            res = [node.plan_to(best['align'], q) for _ in range(args.repeat)]
            out[roll] = best_of(res)

        r0, r180 = out[0], out[180]
        def fmt(r):
            return ((f'{r["travel_deg"]:.0f}', f'{r["travel_by_joint"][J6]:.0f}')
                    if r else ('—', '—'))
        f0, f180 = fmt(r0), fmt(r180)

        cands = [(v['travel_deg'], k, v) for k, v in out.items() if v]
        if cands:
            bestv, bestroll, bestr = min(cands)
            saving = (r0['travel_deg'] - bestv) if r0 else 0.0
        else:
            bestroll, saving = None, 0.0

        print(f'{d["base_z"]:6.3f}{d.get("class_name","?"):>9}'
              f'{f0[0]:>13}{f0[1]:>10}{f180[0]:>15}{f180[1]:>12}'
              f'{(str(bestroll)+"도") if bestroll is not None else "—":>7}'
              f'{saving:8.0f}')
        rows.append({
            'class_name': d.get('class_name', '?'),
            'base_x': d['base_x'], 'base_y': d['base_y'], 'base_z': d['base_z'],
            'roll0_travel': r0['travel_deg'] if r0 else None,
            'roll0_j6': r0['travel_by_joint'][J6] if r0 else None,
            'roll180_travel': r180['travel_deg'] if r180 else None,
            'roll180_j6': r180['travel_by_joint'][J6] if r180 else None,
            'chosen_roll': bestroll,
            'saving_deg': round(saving, 1),
        })

    print('\n=== 요약 ===')
    have0 = [r for r in rows if r['roll0_travel'] is not None]
    both = [r for r in rows if r['roll0_travel'] is not None
            and r['roll180_travel'] is not None]
    if have0:
        print(f'roll 0으로 도달한 목표 {len(have0)}/{len(rows)}개, '
              f'이동량 중앙 {st.median(r["roll0_travel"] for r in have0):.0f}도, '
              f'J6 중앙 {st.median(r["roll0_j6"] for r in have0):.0f}도')
    only180 = [r for r in rows if r['roll0_travel'] is None
               and r['roll180_travel'] is not None]
    if only180:
        print(f'**roll 0으로는 못 가고 roll 180으로만 가는 목표 {len(only180)}개** '
              '— 대칭을 쓰면 도달 가능성 자체가 늘어난다')
    chosen = [r for r in rows if r['chosen_roll'] is not None]
    if chosen:
        n180 = sum(1 for r in chosen if r['chosen_roll'] == 180)
        print(f'싼 쪽을 고르면: 180도가 채택된 목표 {n180}/{len(chosen)}개')
        base = [r['roll0_travel'] for r in chosen if r['roll0_travel'] is not None]
        mixed = [min(v for v in (r['roll0_travel'], r['roll180_travel']) if v is not None)
                 for r in chosen]
        if base:
            b, m = st.median(base), st.median(mixed)
            print(f'이동량 중앙 {b:.0f} -> {m:.0f}도 ({100*(m-b)/b:+.1f}%)')
    if both:
        j6_0 = st.median(r['roll0_j6'] for r in both)
        j6_min = st.median(min(r['roll0_j6'], r['roll180_j6']) for r in both)
        print(f'J6 이동량 중앙 {j6_0:.0f} -> {j6_min:.0f}도 '
              f'({100*(j6_min-j6_0)/j6_0:+.1f}%)  <- "플랜지 180도 회전"의 비용')

    if args.csv and rows:
        with open(args.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f'\nCSV 저장: {args.csv} ({len(rows)}행)')

    scene_objects.publish_tomatoes(node, dets, remove=True)
    octomap_io.clear_octomap(node, rclpy)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
