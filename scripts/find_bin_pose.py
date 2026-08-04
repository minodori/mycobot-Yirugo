#!/usr/bin/env python3
"""통 자세(= 수확통에 놓는 자세 = 대기 자세) **관절벡터 하나**를 고른다.

왜 이 스크립트가 필요한가
-------------------------
`docs/ARMED_POSE_HANDOFF.md` 1.5절 결론 3(D)이 "통 자세를 대기 자세로 쓰면
전이 −30.6%"를 냈지만, 그 측정에서 통 자세는 **pose goal(6×6cm 상자 + yaw
자유)의 IK 해**다 — 플래너가 매번 다른 관절해를 고른다. 그런데 노드는

  * 대기 자세로 `move_to_configuration`(관절 목표)으로 가고,
  * `_is_near_armed_pose`로 "이미 그 자세면 경유를 건너뛴다"를 판정한다.

둘 다 **고정된 관절벡터**를 전제한다. 그래서 그 하나를 골라야 한다.
armed pose(`ARMED_POSE_JOINT_POSITIONS`)가 그랬듯, 자세 하나를 상수로 박는
이상 **작업공간에서 무엇이 되는지 반드시 FK로 확인해야 한다**(1.1절의 교훈 —
관절공간에서 만든 자세가 베드에 7.3cm까지 파고들었던 사고).

무엇을 재나
-----------
후보를 모으고(플래너가 실제로 내놓는 IK 해들), 각 후보에 대해:

  1. **작업공간 안전** — FK로 그리퍼 끝단 위치를 구해 통 입구(g_base
     (0, 0.15, 0.130)) 안에 있는지, 그리퍼가 아래를 보는지, 열매까지 여유가
     얼마인지. armed pose는 33.3cm, 1.5절이 잰 통 자세는 27.7cm였다.
  2. **자세 자체의 유효성** — full 씬(열매+octomap+벽)에서 충돌 검사.
     armed pose가 벽 20mm에 깨졌던 것처럼(6.1절) 이 자세도 벽과 가까울 수 있다.
  3. **사슬 비용** — 15개 정렬 자세 각각에서 후보까지의 전이 합. 사이클에서
     팔은 `정렬ᵢ → 통 → 정렬ᵢ₊₁`을 반복하므로 이 합의 2배가 곧 대기 자세
     비용이다(왕복이 편도의 2배인 것은 관절 목표라 대칭이기 때문 — 4절).

마지막으로 **"고정하는 대가"**를 같이 잰다. pose goal(상자+yaw 자유)로 매번
자유롭게 풀게 두었을 때의 합과 비교하면, 관절벡터 하나로 못박아서 잃는 양이
나온다. 그 값이 크면 노드 쪽을 pose goal로 가야 한다는 뜻이다.

사용법 (스택은 문서 8절 실행 순서 1)번 그대로)
    python3 -u scripts/find_bin_pose.py --repeat 5 \\
        --octomap bags/bed_look_octomap_wall.bin --octomap-acm \\
        --out bags/bin_pose.json
"""

import argparse
import functools
import json
import math
import os
import sys

print = functools.partial(print, flush=True)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from moveit_msgs.msg import PlanningScene

import octomap_io
import scene_objects
import sweep_targets_planned as S
from sweep_targets_planned import PlanProbe, select_approach
from eval_armed_chain import BIN_FLANGE, state_valid, transit
from find_armed_pose_cartesian import fk_flange
from mycobot_280_pick import coord_to_goal_node as N

LOOK = list(N.LOOK_POSE_JOINT_POSITIONS)
ARMED = list(N.ARMED_POSE_JOINT_POSITIONS)

# 그리퍼가 아래를 보는 자세. forward가 WORLD_UP(-Z)과 평행이라 함수가 대체
# 기준축(+X)으로 빠지는데, yaw를 자유롭게 둘 것이므로 어느 쪽이든 상관없다.
DOWN_QUAT = N._compute_look_at_quat_xyzw((0.0, 0.0, -1.0))


def tip_from_joints(pos7):
    """FK 결과(flange 위치+자세) -> 그리퍼 끝단 위치와 정면 단위벡터.

    [2026-08-03] 닫힌 상태 끝단(0.11)을 쓴다 — 통 자세에 올 때 그리퍼는 열매를
    쥐고 닫혀 있다. 0.09(열린 상태 파지점)로 뽑은 이전 해는 끝단 높이를 20mm
    과대평가했다.
    """
    x, y, z, w = pos7[3:]
    fz = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
    tip = tuple(p + N.GRIPPER_TIP_CLOSED_M * f for p, f in zip(pos7[:3], fz))
    return tip, fz


def joint_dist_deg(a, b):
    """두 관절벡터의 최대 축 차이(도). 후보 군집화용."""
    return max(abs(math.degrees(p - q)) for p, q in zip(a, b))


def main():
    p = argparse.ArgumentParser(description='통 자세 관절벡터 도출')
    p.add_argument('--targets', default='bags/lab_bed_detections.json')
    p.add_argument('--octomap', default='bags/bed_look_octomap_wall.bin')
    p.add_argument('--octomap-acm', action='store_true',
                   help='그리퍼/손목이 octomap voxel과 충돌해도 되게 한다')
    p.add_argument('--repeat', type=int, default=5,
                   help='정렬 관절해 수집·후보 수집·pose goal 비교용 반복')
    # [2026-08-02] 후보 순위는 수집보다 **더 많은 반복**을 요구한다. 반복 10회와
    # 15회에서 같은 J1대 후보의 편도합이 4817 <-> 5431도(13%)로 뒤집혔다 —
    # 함정 6("반복 10회로는 IK 분기 무작위성에 묻힌다")이 그대로 재현된 것이다.
    # 수집은 후보를 넓게 모으는 것이 목적이라 반복이 적어도 되지만, 순위는
    # 여기서 갈리므로 따로 올린다.
    p.add_argument('--rank-repeat', type=int, default=None,
                   help='[4] 사슬 비용 순위용 반복(기본: --repeat와 같음)')
    p.add_argument('--orientation-tolerance', type=float, default=0.2)
    # [2026-08-02] 상자 z를 4cm로 두면 끝단이 림 위 1.5cm에 앉는 해가 뽑힌다.
    # 4절이 "림 위 1cm면 point 조건이 12회 중 5회 실패"를 이유로 3cm를 골랐으므로
    # 그 값은 위험 쪽 가장자리다. **대기 자세는 매 사이클 들르는 자리**라 한 번
    # 뽑아 상수로 박는 이상 여유 있는 쪽이어야 한다. z만 ±1cm로 좁힌다.
    p.add_argument('--bin-box', nargs=3, type=float, default=[0.06, 0.06, 0.02],
                   help='통 목표 상자 크기(m). x y z')
    p.add_argument('--max-down-deg', type=float, default=8.0,
                   help='그리퍼 정면이 연직 아래에서 이 각도 이상 기울면 탈락')
    p.add_argument('--cluster-deg', type=float, default=12.0,
                   help='이 각도 안이면 같은 후보로 본다')
    p.add_argument('--max-candidates', type=int, default=8,
                   help='사슬 비용까지 재는 후보 수(모으는 순서대로)')
    p.add_argument('--out', default='bags/bin_pose.json')
    args = p.parse_args()

    S.ORIENTATION_TOLERANCE_RAD = args.orientation_tolerance
    BIN_BOX = list(args.bin_box)
    rank_repeat = args.rank_repeat or args.repeat
    with open(args.targets, encoding='utf-8') as fh:
        dets = json.load(fh)['detections']

    rclpy.init()
    node = PlanProbe(2.0, 10, ARMED)
    if not node.wait_ready():
        raise SystemExit('move_group 없음')

    # ---- 씬 (함정 8: 시작 상태를 가정하지 않는다) ----
    scene_objects.publish_tomatoes(node, dets, remove=True)
    scene_objects.allow_gripper_octomap_collisions(node, False, quiet=True)
    octomap_io.clear_octomap(node, rclpy)
    scene_objects.publish_tomatoes(node, dets)
    scene_objects.allow_gripper_tomato_collisions(node, dets, quiet=True)
    owp = octomap_io.load_octomap_file(args.octomap)
    octomap_io.inject_octomap(node, rclpy, PlanningScene, owp)
    scene_objects.allow_gripper_octomap_collisions(node, args.octomap_acm,
                                                   quiet=True)
    print(f'씬: 열매 {len(dets)}개 + octomap '
          f'{len(octomap_io.decode_msg(owp.octomap))} voxel'
          f' + octomap ACM {"완화" if args.octomap_acm else "완화 없음"}')
    print(f'통 목표: flange {tuple(round(c, 3) for c in BIN_FLANGE)}, '
          f'상자 {BIN_BOX}, yaw 자유\n')

    # ---- 1) 목표별 정렬 관절해 ----
    print(f'[1] 목표별 정렬 관절해 (반복 {args.repeat}회, armed pose 출발)')
    goals = []
    for i, d in enumerate(dets, 1):
        b, why = select_approach(d['base_x'], d['base_y'], d['base_z'])
        if b is None:
            print(f'  #{i} 기하 게이트 탈락({why})')
            continue
        node.start_pose = ARMED
        best = None
        for _ in range(args.repeat):
            r = node.plan_to(b['align'], b['quat'])
            if r['ok'] and (best is None or r['travel_deg'] < best['travel_deg']):
                best = r
        if best is None:
            print(f'  #{i} z={d["base_z"]:.3f} 정렬 실패 — 제외')
            continue
        goals.append({
            'idx': i,
            'joints': [best['final'][n] for n in N.JOINT_NAMES],
            'align_j1_deg': best['j1_deg'],
            'z': d['base_z'],
        })
    print(f'  사슬에 들어간 목표 {len(goals)}/{len(dets)}개\n')
    if not goals:
        raise SystemExit('정렬 자세가 하나도 없다 — 씬을 확인할 것')

    # ---- 2) 후보 모으기 ----
    # 플래너가 실제로 내놓는 IK 해를 후보로 쓴다. 출발 자세를 여러 개 주는
    # 이유: pose goal의 해는 출발 자세에 따라 갈리기 때문(A/B 분기, 7절).
    # 사이클에서 통으로 가는 것은 **파지 직후**이므로 정렬 자세 출발이 본류다.
    print(f'[2] 후보 IK 해 모으기 (출발 {len(goals) + 2}종 × 반복 {args.repeat}회)')
    starts = [('look', LOOK), ('armed', ARMED)]
    starts += [(f'정렬#{g["idx"]}', g['joints']) for g in goals]
    cands = []
    for label, frm in starts:
        node.start_pose = list(frm)
        for _ in range(args.repeat):
            r = node.plan_to(BIN_FLANGE, DOWN_QUAT, box_xyz=BIN_BOX,
                             yaw_free=True)
            if not r['ok']:
                continue
            q = [r['final'][n] for n in N.JOINT_NAMES]
            for c in cands:
                if joint_dist_deg(c['joints'], q) <= args.cluster_deg:
                    c['hits'] += 1
                    break
            else:
                cands.append({'joints': q, 'hits': 1, 'from': label})
    print(f'  서로 다른 후보 {len(cands)}개')
    cands.sort(key=lambda c: -c['hits'])
    cands = cands[:args.max_candidates]

    # ---- 3) 작업공간 안전 + 자세 유효성 ----
    print(f'\n[3] 후보 검사 (FK · 끝단 · 충돌)')
    print(f'{"J1":>8}{"빈도":>6}{"끝단 xyz":>26}{"아래보기":>9}'
          f'{"열매여유":>9}{"유효":>7}  충돌 링크')
    ok_cands = []
    for c in cands:
        pos7 = fk_flange(node, c['joints'])
        tip, fz = tip_from_joints(pos7)
        # 그리퍼 정면이 −Z에 얼마나 가까운가(0도 = 정확히 아래)
        down_deg = math.degrees(math.acos(max(-1.0, min(1.0, -fz[2]))))
        clr = scene_objects.tip_clearance(pos7[:3], pos7[3:], dets)
        valid, hits = state_valid(node, c['joints'])
        # 통 입구 안인가 — 상자 절반 + 여유로 판정
        in_box = (abs(tip[0] - BIN_FLANGE[0]) <= BIN_BOX[0] / 2 + 0.005 and
                  abs(tip[1] - BIN_FLANGE[1]) <= BIN_BOX[1] / 2 + 0.005 and
                  # BIN_FLANGE[2] = 놓는 지점 + 닫힌 그리퍼 길이이므로 같은
                  # 값을 빼야 놓는 지점(0.130)이 나온다. 0.09를 빼면 0.150이
                  # 되어 판정 기준이 20mm 위로 밀린다. [2026-08-03]
                  abs(tip[2] - (BIN_FLANGE[2] - N.GRIPPER_TIP_CLOSED_M))
                  <= BIN_BOX[2] / 2 + 0.005)
        c.update(fk=pos7, tip=tip, down_deg=down_deg, clearance=clr,
                 valid=valid, hits_links=hits, in_box=in_box)
        print(f'{math.degrees(c["joints"][0]):+8.1f}{c["hits"]:6d}'
              f'  ({tip[0]:+.3f},{tip[1]:+.3f},{tip[2]:+.3f})'
              f'{"통안" if in_box else "통밖":>7}'
              f'{down_deg:8.1f}도{clr * 100:8.1f}cm'
              f'{"OK" if valid else "충돌":>7}  {hits}')
        if valid and in_box and down_deg <= 25.0:
            ok_cands.append(c)
    print(f'\n  안전·유효 후보 {len(ok_cands)}/{len(cands)}개')
    if not ok_cands:
        raise SystemExit('쓸 수 있는 후보가 없다')

    # ---- 4) 사슬 비용 ----
    # 사이클은 정렬ᵢ → 통 → 정렬ᵢ₊₁의 반복이다. 관절 목표는 대칭이므로
    # (문서 4절) 편도 합의 2배가 대기 자세 비용의 근사다.
    print(f'\n[4] 사슬 비용 — 정렬 {len(goals)}개에서 후보까지 편도 합 '
          f'(반복 {rank_repeat}회, 구간별 최소)')
    for c in ok_cands:
        total = 0.0
        fail = 0
        for g in goals:
            t = transit(node, g['joints'], c['joints'], rank_repeat)
            if t is None:
                fail += 1
                continue
            total += t
        c['transit_sum'] = total
        c['transit_fail'] = fail
        to_look = transit(node, c['joints'], LOOK, rank_repeat)
        c['to_look'] = to_look
        print(f'  J1 {math.degrees(c["joints"][0]):+7.1f}도  편도합 {total:7.0f}도'
              f'  (도달 실패 {fail}개)  통→look {to_look if to_look else float("nan"):.0f}도')

    # ---- 5) 고정하는 대가 — pose goal로 자유롭게 두면 ----
    print('\n[5] "관절벡터로 고정하는 대가" — 같은 구간을 pose goal'
          '(상자+yaw 자유)로 풀었을 때')
    free_total = 0.0
    free_fail = 0
    for g in goals:
        node.start_pose = list(g['joints'])
        best = None
        for _ in range(rank_repeat):
            r = node.plan_to(BIN_FLANGE, DOWN_QUAT, box_xyz=BIN_BOX,
                             yaw_free=True)
            if r['ok'] and (best is None or r['travel_deg'] < best['travel_deg']):
                best = r
        if best is None:
            free_fail += 1
            continue
        free_total += best['travel_deg']
    print(f'  pose goal 편도합 {free_total:7.0f}도 (실패 {free_fail}개)')

    # ---- 6) 결론 ----
    # **자세 품질은 통과 조건, 비용은 목적함수다.**
    #
    # 처음엔 반대로 짰다 — 실행마다 순위가 뒤집히길래(4817 <-> 5431도) 함정 6의
    # IK 분기 잡음으로 보고 비용을 동률 처리했다. 그건 오독이었다. 뒤집힌 것은
    # **같은 자세의 재측정이 아니라 매 실행이 뽑아낸 서로 다른 관절벡터**였다.
    # 후보들은 J1만이 아니라 J2~J6가 통째로 다르고, 편도합은 그 벡터와 정렬
    # 15개 사이의 관절공간 거리라 반복 30회의 최소를 쓰면 대체로 재현된다.
    # 즉 후보 간 비용 차이는 실재한다.
    #
    # 반대로 자세 품질(그리퍼가 아래를 얼마나 정확히 보는가)은 **정도의 문제가
    # 아니라 되고 안 되고의 문제**다 — 통 위에서 기울어진 채 열면 열매가 통
    # 밖으로 튄다. 그래서 임계값으로 거르고, 통과한 것 중에서 가장 싼 것을 쓴다.
    usable = [c for c in ok_cands
              if c['transit_fail'] == 0 and c['to_look']
              and c['down_deg'] <= args.max_down_deg]
    if not usable:
        print(f'  아래보기 편차 {args.max_down_deg}도 이하인 후보가 없다 — '
              '임계값을 완화해 진행')
        usable = [c for c in ok_cands if c['transit_fail'] == 0 and c['to_look']]
    if not usable:
        usable = ok_cands
    best = min(usable, key=lambda c: c['transit_sum'])
    print(f'\n  아래보기 편차 {args.max_down_deg}도 이하 {len(usable)}개 통과 — '
          '그 안에서 편도합 최소를 채택')
    print('\n=== 채택 ===')
    print('  관절(도): [' + ', '.join(
        f'{math.degrees(v):+.2f}' for v in best['joints']) + ']')
    print('  관절(rad): [' + ', '.join(f'{v:.6f}' for v in best['joints']) + ']')
    print(f'  그리퍼 끝단 {tuple(round(v, 4) for v in best["tip"])}, '
          f'아래보기 편차 {best["down_deg"]:.1f}도')
    print(f'  열매까지 여유 {best["clearance"] * 100:.1f}cm '
          f'(armed pose 33.3cm, 1.5절 통 자세 27.7cm)')
    print(f'  정렬 15개 편도합 {best["transit_sum"]:.0f}도, '
          f'pose goal 자유 {free_total:.0f}도 '
          f'(고정 대가 {100 * (best["transit_sum"] - free_total) / free_total:+.1f}%)')

    out = {
        'joints_rad': best['joints'],
        'joints_deg': [math.degrees(v) for v in best['joints']],
        'flange': list(best['fk'][:3]),
        'flange_quat_xyzw': list(best['fk'][3:]),
        'tip': list(best['tip']),
        'down_deg': best['down_deg'],
        'tip_clearance_m': best['clearance'],
        'transit_sum_deg': best['transit_sum'],
        'transit_sum_free_deg': free_total,
        'to_look_deg': best['to_look'],
        'scene': {
            'octomap': args.octomap,
            'octomap_acm': args.octomap_acm,
            'repeat': args.repeat,
            'rank_repeat': rank_repeat,
            'orientation_tolerance': args.orientation_tolerance,
            'n_goals': len(goals),
        },
        'candidates': [
            {
                'joints_deg': [math.degrees(v) for v in c['joints']],
                'hits': c['hits'],
                'tip': list(c['tip']),
                'down_deg': c['down_deg'],
                'clearance_m': c['clearance'],
                'valid': c['valid'],
                'in_box': c['in_box'],
                'transit_sum_deg': c.get('transit_sum'),
            }
            for c in cands
        ],
    }
    with open(args.out, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, indent=2, ensure_ascii=False)
    print(f'\n저장: {args.out}')

    # 함정 15 — ACM 완화를 켠 채로 두지 않는다.
    scene_objects.allow_gripper_octomap_collisions(node, False, quiet=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
