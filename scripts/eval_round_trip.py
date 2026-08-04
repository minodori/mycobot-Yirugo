#!/usr/bin/env python3
"""수확 사이클 **왕복** 비용을 잰다 — 지금까지는 가는 길만 쟀다.

왜 왕복이어야 하나
------------------
`docs/ROSBAG_HANDOFF.md` §4 실측에서 복귀 구간은 6축 평균 542도로 **정렬(520도)
보다 크다**. 사이클 시간의 42%다. 그런데 지금까지의 측정은 전부 "시작 자세 ->
정렬 위치" 편도였다.

복귀에는 구조적 비대칭이 있다:
  가는 길  pose goal (위치+방향) -> OMPL이 목표 상태를 **매번 다시 샘플링**한다.
           그래서 A/B 분기가 무작위로 갈린다.
  오는 길  joint goal (`move_to_configuration`) -> 목표가 확정이라 결정적이다.
           **하지만 출발 자세가 가는 길이 어디에 착지했는지에 달려 있다.**

즉 복귀 비용은 가는 길의 분기와 결합돼 있다. B분기로 갔으면 B자세에서
돌아와야 한다. 이 스크립트는 그 결합을 분기별로 나눠 측정한다.

무엇을 비교하나
---------------
  look pose 왕복   (현재 동작)
  armed pos 왕복   (docs/new_concept.md 제안)
각각 A분기/B분기로 나눠서 본다.

사용법
------
    ros2 launch mycobot_280_moveit2 demo_octomap.launch.py \
        enable_octomap:=false enable_camera:=false use_rviz:=false
    python3 scripts/eval_round_trip.py --repeat 12
"""

import argparse
import json
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
import sweep_targets_planned as S
from sweep_targets_planned import PlanProbe, select_approach
from mycobot_280_pick import coord_to_goal_node as N

# scripts/find_armed_pose.py 산출 (bags/armed_pose.json)
ARMED_POSE_DEG = [27.08, -2.39, -63.53, 1.70, -80.44, 64.83]

# A/B 분기 판별. 베드 전용 기준이다 — 목표 방위각이 -14~+20도라 정상 분기는
# J1이 그 근처에 머문다. 통(방위각 +90도)에는 이 기준을 쓰면 안 된다.
BRANCH_A_MAX_J1_DEG = 80.0


def summarize(values):
    if not values:
        return '—'
    s = sorted(values)
    q = lambda p: s[int(p * (len(s) - 1))]
    return f'{st.median(s):5.0f} (IQR {q(.75) - q(.25):3.0f})'


def main():
    p = argparse.ArgumentParser(description='수확 사이클 왕복 비용 측정')
    p.add_argument('--targets', default='bags/lab_bed_detections.json')
    p.add_argument('--repeat', type=int, default=12)
    p.add_argument('--orientation-tolerance', type=float, default=0.2)
    p.add_argument('--planning-time', type=float, default=2.0)
    p.add_argument('--attempts', type=int, default=10)
    p.add_argument('--csv', default=None)
    args = p.parse_args()

    S.ORIENTATION_TOLERANCE_RAD = args.orientation_tolerance
    with open(args.targets, encoding='utf-8') as fh:
        dets = json.load(fh)['detections']

    poses = {
        'look pose': list(N.LOOK_POSE_JOINT_POSITIONS),
        'armed pos': [math.radians(v) for v in ARMED_POSE_DEG],
    }

    rclpy.init()
    node = PlanProbe(args.planning_time, args.attempts)
    if not node.wait_ready():
        raise SystemExit('move_group 없음 — 스택을 먼저 띄울 것')

    print(f'\n목표 {len(dets)}개 x 반복 {args.repeat}회, '
          f'방향 허용오차 {math.degrees(args.orientation_tolerance):.1f}°')
    print('가는 길 = pose goal(정렬 위치), 오는 길 = joint goal(시작 자세로 복귀)\n')

    rows = []
    for label, home in poses.items():
        node.start_pose = home
        out_a, out_b, back_a, back_b, trip_a, trip_b = [], [], [], [], [], []
        fail_out = fail_back = 0

        for d in dets:
            best, reason = select_approach(d['base_x'], d['base_y'], d['base_z'])
            if best is None:
                continue
            for _ in range(args.repeat):
                go = node.plan_to(best['align'], best['quat'])
                if not go['ok']:
                    fail_out += 1
                    continue
                landed = [go['final'][n] for n in N.JOINT_NAMES]
                back = node.plan_to_config(home, from_joints=landed)
                if not back['ok']:
                    fail_back += 1
                    continue
                is_a = abs(go['j1_deg']) < BRANCH_A_MAX_J1_DEG
                (out_a if is_a else out_b).append(go['travel_deg'])
                (back_a if is_a else back_b).append(back['travel_deg'])
                (trip_a if is_a else trip_b).append(
                    go['travel_deg'] + back['travel_deg'])

        n_a, n_b = len(trip_a), len(trip_b)
        total = n_a + n_b
        print(f'[{label}]  가는 길 실패 {fail_out}, 오는 길 실패 {fail_back}')
        print(f'  분기 비율      A {n_a}/{total} ({100*n_a/total:.0f}%)   '
              f'B {n_b}/{total} ({100*n_b/total:.0f}%)')
        print(f'  {"":14}{"A분기":>18}{"B분기":>18}{"전체":>18}')
        for name, a, b in (('가는 길', out_a, out_b),
                           ('오는 길', back_a, back_b),
                           ('왕복 합', trip_a, trip_b)):
            print(f'  {name:<14}{summarize(a):>18}{summarize(b):>18}'
                  f'{summarize(a + b):>18}')
        print()
        rows.append({
            'home': label, 'n_a': n_a, 'n_b': n_b,
            'out_a': round(st.median(out_a), 1) if out_a else None,
            'out_b': round(st.median(out_b), 1) if out_b else None,
            'back_a': round(st.median(back_a), 1) if back_a else None,
            'back_b': round(st.median(back_b), 1) if back_b else None,
            'trip_a': round(st.median(trip_a), 1) if trip_a else None,
            'trip_b': round(st.median(trip_b), 1) if trip_b else None,
            'trip_all': round(st.median(trip_a + trip_b), 1),
        })

    base = next(r for r in rows if r['home'] == 'look pose')['trip_all']
    print('왕복 총량 (중앙값) 비교')
    for r in rows:
        print(f'  {r["home"]:<11}{r["trip_all"]:7.0f}°   '
              f'{100*(r["trip_all"]-base)/base:+6.1f}%')
    best = min(rows, key=lambda r: r['trip_a'] or 1e9)
    print(f'\n분기까지 고정한다면 최선: {best["home"]} + A분기 = '
          f'{best["trip_a"]:.0f}°  ({100*(best["trip_a"]-base)/base:+.1f}%)')

    if args.csv:
        import csv
        with open(args.csv, 'w', newline='', encoding='utf-8') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f'\nCSV 저장: {args.csv}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
