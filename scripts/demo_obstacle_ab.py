#!/usr/bin/env python3
"""장애물 있을 때 / 없을 때를 **같은 목표에 대해 번갈아** 보여주는 데모.

왜 따로 만드나
--------------
`sweep_targets_planned.py`는 목표 15개를 **순회**한다. 그래서 화면에는 매번 다른
목표의 궤적이 지나가고, "장애물 때문에 경로가 이렇게 달라졌다"가 안 보인다.
비교가 되려면 **목표를 고정하고 조건만 바꿔야** 한다.

이 스크립트는 목표 하나를 잡고 무한 반복한다:

    [장애물 ON]  대기 자세 -> 정렬 -> 직진 -> 후퇴 -> 복귀   (주황 라벨)
    [장애물 OFF] 같은 구간                                    (파랑 라벨)

장애물은 **collision object**로 넣고 뺀다(octomap voxel이 아니라). 이유:
재생 스택은 bag 클라우드가 octomap을 계속 갱신하므로 주입한 voxel이 덮어써지고,
실물에서는 거기에 `/clear_octomap`까지 더해진다(문서 5.6b). collision object는
둘 다에 안 지워진다 — 실물에서도 같은 명령이 그대로 통한다.

ACM은 장애물에 안 건다. **모든 링크가 피해야** 데모가 성립한다
(`--octomap-acm`이 그리퍼·손목을 통과시키는 것과 대비된다).

사용법
------
    # 재생 스택이나 측정 스택이 떠 있는 상태에서
    python3 -u scripts/demo_obstacle_ab.py --target 4 --cycle bsc

    # 벽까지 같이 세우고, 목표를 바꿔 가며
    python3 -u scripts/demo_obstacle_ab.py --target 4 --preset both
"""

import argparse
import functools
import json
import math
import os
import sys
import time

print = functools.partial(print, flush=True)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy

import octomap_io
import scene_objects
import sweep_targets_planned as S
from sweep_targets_planned import PlanProbe, select_approach, _concat, _reversed
from mycobot_280_pick import coord_to_goal_node as N


def plan_leg(node, det, waiting, repeat):
    """대기 자세 -> 정렬 -> 직진 -> 후퇴 -> 복귀를 계획해 이어 붙인다.

    (재생용 궤적, 정렬 이동량, 복귀 이동량)을 돌려준다. 실패하면 (None, ...).
    """
    best, why = select_approach(det['base_x'], det['base_y'], det['base_z'])
    if best is None:
        return None, None, None, f'기하 게이트 탈락({why})'

    node.start_pose = list(waiting)
    good = []
    for _ in range(repeat):
        r = node.plan_to(best['align'], best['quat'])
        if r['ok']:
            good.append(r)
    if not good:
        return None, None, None, '정렬 플래닝 실패'

    # 화면에 그릴 해는 **J6가 가장 안 도는 것**으로 고른다(스윕과 같은 기준) —
    # 그래야 조건이 바뀔 때 그리퍼가 제자리에서 반 바퀴 도는 잡음이 안 섞인다.
    pick = min(good, key=lambda r: (r['travel_by_joint'][N.JOINT_NAMES[5]],
                                    r['travel_deg']))
    align_joints = [pick['final'][n] for n in N.JOINT_NAMES]
    traj = pick['trajectory']

    cart = node.plan_straight_in(align_joints, best)
    if cart and cart['fraction'] > 0.0:
        traj = _concat(traj, cart['trajectory'])
        traj = _concat(traj, _reversed(cart['trajectory']))   # [5/5] 후퇴

    back_travel, back_traj = None, None
    for _ in range(repeat):
        b = node.plan_to_config(list(waiting), from_joints=align_joints)
        if b['ok'] and (back_travel is None or b['travel_deg'] < back_travel):
            back_travel, back_traj = b['travel_deg'], b['trajectory']
    if back_traj is not None:
        traj = _concat(traj, back_traj)

    return traj, pick['travel_deg'], back_travel, None


def main():
    p = argparse.ArgumentParser(description='장애물 유무 A/B 데모(목표 고정)')
    p.add_argument('--targets', default='bags/lab_bed_detections.json')
    p.add_argument('--target', type=int, default=4,
                   help='몇 번 목표를 쓸 것인가(1부터). 여러 개를 돌려보고 '
                        '경로 차이가 큰 것을 고를 것')
    p.add_argument('--cycle', choices=sorted(
                       set(N.CYCLE_ALIASES) | set(N.WAITING_POSES)),
                   default='bsc', help='대기 자세 = 사이클 종류')
    p.add_argument('--preset', choices=['pillar', 'wall', 'both'],
                   default='pillar', help='세울 장애물')
    p.add_argument('--repeat', type=int, default=5,
                   help='조건마다 계획 횟수(최소를 그린다)')
    p.add_argument('--orientation-tolerance', type=float, default=0.2)
    p.add_argument('--tomatoes', action='store_true', default=True,
                   help='열매를 구로 넣고 ACM을 푼다(기본 켜짐)')
    p.add_argument('--no-tomatoes', dest='tomatoes', action='store_false')
    p.add_argument('--marker-alpha', type=float, default=0.25)
    p.add_argument('--display-seconds', type=float, default=6.0)
    p.add_argument('--display-repeats', type=int, default=2)
    p.add_argument('--pause', type=float, default=1.5,
                   help='조건이 바뀌기 전 멈춤(초) — 화면에서 전환이 읽히도록')
    p.add_argument('--rounds', type=int, default=0,
                   help='0이면 Ctrl+C까지 무한 반복')
    args = p.parse_args()

    S.ORIENTATION_TOLERANCE_RAD = args.orientation_tolerance
    with open(args.targets, encoding='utf-8') as fh:
        dets = json.load(fh)['detections']
    if not 1 <= args.target <= len(dets):
        raise SystemExit(f'--target은 1~{len(dets)} 범위여야 한다')
    det = dets[args.target - 1]

    key = N.CYCLE_ALIASES.get(args.cycle, args.cycle)
    waiting, waiting_name, cycle_short = N.WAITING_POSES[key]
    print(f'사이클: {cycle_short} — 대기 자세 {waiting_name}')
    print(f'목표 #{args.target}: {det.get("class_name", "?")} '
          f'({det["base_x"]:+.3f}, {det["base_y"]:+.3f}, {det["base_z"]:+.3f})')

    boxes = {'pillar': [('pillar', octomap_io.obstacle_box(),
                         scene_objects.OBSTACLE_OBJECT_ID)],
             'wall': [('wall', octomap_io.wall_box(),
                       scene_objects.WALL_OBJECT_ID)]}
    boxes['both'] = boxes['pillar'] + boxes['wall']
    items = boxes[args.preset]
    print(f'장애물: {", ".join(name for name, _b, _i in items)}\n')

    rclpy.init()
    node = PlanProbe(2.0, 10, list(waiting))
    node.MARKER_ALPHA = args.marker_alpha
    node.MARKER_ALPHA_CURRENT = min(1.0, args.marker_alpha * 2)
    if not node.wait_ready():
        raise SystemExit('move_group 없음')

    if args.tomatoes:
        scene_objects.publish_tomatoes(node, dets)
        scene_objects.allow_gripper_tomato_collisions(node, dets, quiet=True)

    def set_obstacle(on):
        for _name, (lo, hi), oid in items:
            scene_objects.publish_obstacle(node, lo, hi, remove=not on,
                                           object_id=oid)

    round_no = 0
    try:
        while args.rounds == 0 or round_no < args.rounds:
            round_no += 1
            for on in (True, False):
                set_obstacle(on)
                state = 'ON' if on else 'OFF'
                traj, align_travel, back_travel, err = plan_leg(
                    node, det, waiting, args.repeat)

                # 라벨은 ASCII만(함정 14 — 한글·공백·도 기호는 안 보인다).
                lines = [f'{cycle_short}:{key}', f'OBSTACLE:{state}',
                         f'#{args.target}:{det.get("class_name", "?")[:4]}'
                         f':z{det["base_z"]:.2f}']
                if err:
                    lines.append(f'FAILED:{err[:12]}')
                    rgb = (0.95, 0.35, 0.35)
                else:
                    lines.append(f'trv{align_travel:.0f}'
                                 + (f':ret{back_travel:.0f}'
                                    if back_travel is not None else ''))
                    # ON은 주황, OFF는 파랑 — 색만 보고도 조건이 읽히도록.
                    rgb = (1.0, 0.6, 0.15) if on else (0.4, 0.8, 1.0)

                node.publish_markers(dets, args.target - 1, None)
                node.publish_status_label(
                    lines, rgb,
                    (det['base_x'], det['base_y'], det['base_z']))
                print(f'{round_no:3d}회차 장애물 {state:3s}  '
                      + (f'실패({err})' if err else
                         f'정렬 {align_travel:5.0f}도  복귀 '
                         + (f'{back_travel:5.0f}도' if back_travel else '  실패')))

                if traj is not None:
                    node.animate_trajectory(traj, args.display_seconds,
                                            args.display_repeats)
                end = time.time() + args.pause
                while time.time() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print(f'\n중단 — {round_no}회차까지')
    finally:
        # 씬을 원래대로. 안 되돌리면 다음 측정이 남은 장애물을 모른 채 돈다(함정 3).
        set_obstacle(False)
        if args.tomatoes:
            scene_objects.publish_tomatoes(node, dets, remove=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
